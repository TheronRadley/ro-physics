//! Time of Impact (TOI) calculation.
//!
//! This module provides algorithms for calculating the exact time when two
//! shapes will collide during their motion.

use std::fmt;

use crate::constants::EPSILON;
use crate::math::{Rot2, Transform, Vec2};
use crate::shapes::{Circle, Polygon};
use crate::geometry::Aabb;

/// Result of a time of impact calculation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct ToiResult {
    /// Time of impact (0.0 to 1.0)
    pub toi: f32,
    /// Whether a collision was found
    pub colliding: bool,
    /// Number of iterations used
    pub iterations: usize,
}

impl ToiResult {
    /// Create a new TOI result
    #[inline]
    #[must_use]
    pub const fn new(toi: f32, colliding: bool, iterations: usize) -> Self {
        Self {
            toi,
            colliding,
            iterations,
        }
    }

    /// Check if the result indicates a collision
    #[inline]
    #[must_use]
    pub const fn is_colliding(&self) -> bool {
        self.colliding
    }

    /// Get the TOI, clamped to valid range
    #[inline]
    #[must_use]
    pub fn clamped_toi(&self) -> f32 {
        self.toi.clamp(0.0, 1.0)
    }
}

impl Default for ToiResult {
    fn default() -> Self {
        Self::new(1.0, false, 0)
    }
}

impl fmt::Display for ToiResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ToiResult(toi: {:.6}, colliding: {}, iterations: {})",
            self.toi, self.colliding, self.iterations
        )
    }
}

/// Time of Impact calculation for various shape pairs.
#[derive(Debug, Clone)]
pub struct TimeOfImpact;

impl TimeOfImpact {
    /// Calculate TOI between two circles moving with constant velocity.
    ///
    /// Uses analytical solution for circle-circle TOI.
    #[inline]
    #[must_use]
    pub fn circle_circle(
        circle_a: &Circle,
        position_a: Vec2,
        velocity_a: Vec2,
        circle_b: &Circle,
        position_b: Vec2,
        velocity_b: Vec2,
        max_iterations: usize,
        tolerance: f32,
    ) -> ToiResult {
        // Relative motion
        let relative_position = position_b - position_a;
        let relative_velocity = velocity_b - velocity_a;
        
        let sum_radius = circle_a.radius() + circle_b.radius();
        
        // If already overlapping, TOI is 0
        if relative_position.length_squared() <= sum_radius * sum_radius {
            return ToiResult::new(0.0, true, 0);
        }
        
        // If not moving toward each other, no collision
        let relative_speed_squared = relative_velocity.length_squared();
        if relative_speed_squared < EPSILON {
            return ToiResult::new(1.0, false, 0);
        }
        
        // Solve quadratic equation: |p + v*t|^2 = r^2
        // Where p = relative_position, v = relative_velocity, r = sum_radius
        let p = relative_position;
        let v = relative_velocity;
        let r_squared = sum_radius * sum_radius;
        
        // Coefficients: a*t^2 + b*t + c = 0
        let a = v.dot(v);
        let b = 2.0 * p.dot(v);
        let c = p.dot(p) - r_squared;
        
        // Discriminant
        let discriminant = b * b - 4.0 * a * c;
        
        // No real solution
        if discriminant < 0.0 {
            return ToiResult::new(1.0, false, 0);
        }
        
        // Calculate roots
        let sqrt_discriminant = discriminant.sqrt();
        let t1 = (-b - sqrt_discriminant) / (2.0 * a);
        let t2 = (-b + sqrt_discriminant) / (2.0 * a);
        
        // Find the smallest positive root in [0, 1]
        let mut toi = 1.0;
        let mut found = false;
        
        if t1 >= 0.0 && t1 <= 1.0 {
            toi = t1;
            found = true;
        }
        if t2 >= 0.0 && t2 <= 1.0 && t2 < toi {
            toi = t2;
            found = true;
        }
        
        ToiResult::new(toi, found, 0)
    }

    /// Calculate TOI between a circle and a polygon using conservative advancement.
    ///
    /// This uses an iterative approach to find the TOI.
    #[inline]
    #[must_use]
    pub fn circle_polygon(
        circle: &Circle,
        position_a: Vec2,
        velocity_a: Vec2,
        polygon: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        max_iterations: usize,
        tolerance: f32,
    ) -> ToiResult {
        // For now, use a simplified approach
        // Full implementation would require more complex geometry
        
        // Check if already overlapping
        if Self::circle_polygon_overlap(
            circle, position_a, polygon, position_b, rotation_b
        ) {
            return ToiResult::new(0.0, true, 0);
        }
        
        // Conservative advancement
        Self::conservative_advancement(
            circle, position_a, velocity_a,
            polygon, position_b, velocity_b, rotation_b, angular_velocity_b,
            max_iterations, tolerance
        )
    }

    /// Calculate TOI between two polygons using conservative advancement.
    #[inline]
    #[must_use]
    pub fn polygon_polygon(
        polygon_a: &Polygon,
        position_a: Vec2,
        velocity_a: Vec2,
        rotation_a: f32,
        angular_velocity_a: f32,
        polygon_b: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        max_iterations: usize,
        tolerance: f32,
    ) -> ToiResult {
        // Check if already overlapping
        if Self::polygon_polygon_overlap(
            polygon_a, position_a, rotation_a,
            polygon_b, position_b, rotation_b
        ) {
            return ToiResult::new(0.0, true, 0);
        }
        
        // Conservative advancement
        Self::conservative_advancement_polygon_polygon(
            polygon_a, position_a, velocity_a, rotation_a, angular_velocity_a,
            polygon_b, position_b, velocity_b, rotation_b, angular_velocity_b,
            max_iterations, tolerance
        )
    }

    /// Conservative advancement for circle-polygon TOI.
    ///
    /// This iteratively advances the circle toward the polygon,
    /// checking for overlap at each step.
    #[inline]
    fn conservative_advancement(
        circle: &Circle,
        position_a: Vec2,
        velocity_a: Vec2,
        polygon: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        max_iterations: usize,
        tolerance: f32,
    ) -> ToiResult {
        let mut toi = 0.0;
        let mut remaining = 1.0;
        
        for iteration in 0..max_iterations {
            // Check for overlap at current position
            let current_pos_a = position_a + velocity_a * toi;
            let current_pos_b = position_b + velocity_b * toi;
            let current_rotation_b = rotation_b + angular_velocity_b * toi;
            
            if Self::circle_polygon_overlap(
                circle, current_pos_a, polygon, current_pos_b, current_rotation_b
            ) {
                // Binary search to find exact TOI
                let result = Self::binary_search_toi(
                    circle, position_a, velocity_a,
                    polygon, position_b, velocity_b, rotation_b, angular_velocity_b,
                    toi - remaining, toi,
                    max_iterations - iteration,
                    tolerance
                );
                return result;
            }
            
            // Advance by a fraction of the remaining time
            let advance = Self::estimate_advancement(
                circle, position_a, velocity_a,
                polygon, position_b, velocity_b, rotation_b, angular_velocity_b,
                toi, remaining
            );
            
            toi += advance * remaining;
            remaining *= (1.0 - advance);
            
            if advance < tolerance {
                // No collision found
                return ToiResult::new(1.0, false, iteration);
            }
        }
        
        ToiResult::new(1.0, false, max_iterations)
    }

    /// Conservative advancement for polygon-polygon TOI.
    #[inline]
    fn conservative_advancement_polygon_polygon(
        polygon_a: &Polygon,
        position_a: Vec2,
        velocity_a: Vec2,
        rotation_a: f32,
        angular_velocity_a: f32,
        polygon_b: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        max_iterations: usize,
        tolerance: f32,
    ) -> ToiResult {
        let mut toi = 0.0;
        let mut remaining = 1.0;
        
        for iteration in 0..max_iterations {
            // Check for overlap at current position
            let current_pos_a = position_a + velocity_a * toi;
            let current_rotation_a = rotation_a + angular_velocity_a * toi;
            let current_pos_b = position_b + velocity_b * toi;
            let current_rotation_b = rotation_b + angular_velocity_b * toi;
            
            if Self::polygon_polygon_overlap(
                polygon_a, current_pos_a, current_rotation_a,
                polygon_b, current_pos_b, current_rotation_b
            ) {
                // Binary search to find exact TOI
                let result = Self::binary_search_toi_polygon_polygon(
                    polygon_a, position_a, velocity_a, rotation_a, angular_velocity_a,
                    polygon_b, position_b, velocity_b, rotation_b, angular_velocity_b,
                    toi - remaining, toi,
                    max_iterations - iteration,
                    tolerance
                );
                return result;
            }
            
            // Advance by a fraction of the remaining time
            let advance = Self::estimate_advancement_polygon_polygon(
                polygon_a, position_a, velocity_a, rotation_a, angular_velocity_a,
                polygon_b, position_b, velocity_b, rotation_b, angular_velocity_b,
                toi, remaining
            );
            
            toi += advance * remaining;
            remaining *= (1.0 - advance);
            
            if advance < tolerance {
                // No collision found
                return ToiResult::new(1.0, false, iteration);
            }
        }
        
        ToiResult::new(1.0, false, max_iterations)
    }

    /// Binary search to find exact TOI between circle and polygon.
    #[inline]
    fn binary_search_toi(
        circle: &Circle,
        position_a: Vec2,
        velocity_a: Vec2,
        polygon: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        mut low: f32,
        mut high: f32,
        max_iterations: usize,
        tolerance: f32,
    ) -> ToiResult {
        let mut toi = high;
        
        for iteration in 0..max_iterations {
            let mid = (low + high) / 2.0;
            
            let pos_a = position_a + velocity_a * mid;
            let pos_b = position_b + velocity_b * mid;
            let rot_b = rotation_b + angular_velocity_b * mid;
            
            if Self::circle_polygon_overlap(circle, pos_a, polygon, pos_b, rot_b) {
                toi = mid;
                high = mid;
            } else {
                low = mid;
            }
            
            if (high - low) < tolerance {
                return ToiResult::new(toi, true, iteration);
            }
        }
        
        ToiResult::new(toi, true, max_iterations)
    }

    /// Binary search to find exact TOI between two polygons.
    #[inline]
    fn binary_search_toi_polygon_polygon(
        polygon_a: &Polygon,
        position_a: Vec2,
        velocity_a: Vec2,
        rotation_a: f32,
        angular_velocity_a: f32,
        polygon_b: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        mut low: f32,
        mut high: f32,
        max_iterations: usize,
        tolerance: f32,
    ) -> ToiResult {
        let mut toi = high;
        
        for iteration in 0..max_iterations {
            let mid = (low + high) / 2.0;
            
            let pos_a = position_a + velocity_a * mid;
            let rot_a = rotation_a + angular_velocity_a * mid;
            let pos_b = position_b + velocity_b * mid;
            let rot_b = rotation_b + angular_velocity_b * mid;
            
            if Self::polygon_polygon_overlap(
                polygon_a, pos_a, rot_a,
                polygon_b, pos_b, rot_b
            ) {
                toi = mid;
                high = mid;
            } else {
                low = mid;
            }
            
            if (high - low) < tolerance {
                return ToiResult::new(toi, true, iteration);
            }
        }
        
        ToiResult::new(toi, true, max_iterations)
    }

    /// Estimate advancement for circle-polygon.
    #[inline]
    fn estimate_advancement(
        circle: &Circle,
        position_a: Vec2,
        velocity_a: Vec2,
        polygon: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        toi: f32,
        remaining: f32,
    ) -> f32 {
        // Calculate relative velocity
        let relative_velocity = velocity_b - velocity_a;
        
        // Find the closest point on polygon to circle center
        let current_pos_a = position_a + velocity_a * toi;
        let current_pos_b = position_b + velocity_b * toi;
        let current_rotation_b = rotation_b + angular_velocity_b * toi;
        
        // Work in the polygon's local frame (polygon centred on the origin
        // and axis aligned), then transform the result back to world space.
        let local_point = (current_pos_a - current_pos_b).rotate(-current_rotation_b);
        let closest_point = polygon.closest_point(local_point).rotate(current_rotation_b)
            + current_pos_b;
        
        let delta = closest_point - current_pos_a;
        let distance = delta.length();
        let sum_radius = circle.radius();
        
        // If moving toward each other
        if delta.dot(relative_velocity) < 0.0 {
            let relative_speed = relative_velocity.length();
            if relative_speed > EPSILON {
                let time_to_contact = (distance - sum_radius) / relative_speed;
                (time_to_contact / remaining).clamp(0.1, 1.0)
            } else {
                0.1
            }
        } else {
            0.1
        }
    }

    /// Estimate advancement for polygon-polygon.
    #[inline]
    fn estimate_advancement_polygon_polygon(
        polygon_a: &Polygon,
        position_a: Vec2,
        velocity_a: Vec2,
        rotation_a: f32,
        angular_velocity_a: f32,
        polygon_b: &Polygon,
        position_b: Vec2,
        velocity_b: Vec2,
        rotation_b: f32,
        angular_velocity_b: f32,
        toi: f32,
        remaining: f32,
    ) -> f32 {
        // Use a conservative estimate
        let relative_velocity = velocity_b - velocity_a;
        let relative_speed = relative_velocity.length();
        
        if relative_speed > EPSILON {
            // Estimate based on distance between AABBs
            let current_pos_a = position_a + velocity_a * toi;
            let current_pos_b = position_b + velocity_b * toi;
            
            let aabb_a = polygon_a.aabb(Transform::new(
                current_pos_a,
                Rot2::new(rotation_a + angular_velocity_a * toi),
            ));
            let aabb_b = polygon_b.aabb(Transform::new(
                current_pos_b,
                Rot2::new(rotation_b + angular_velocity_b * toi),
            ));
            
            let distance = Self::aabb_distance(&aabb_a, &aabb_b);
            
            if distance > 0.0 {
                let time_to_contact = distance / relative_speed;
                (time_to_contact / remaining).clamp(0.1, 1.0)
            } else {
                0.1
            }
        } else {
            0.1
        }
    }

    /// Calculate distance between two AABBs.
    #[inline]
    fn aabb_distance(aabb1: &Aabb, aabb2: &Aabb) -> f32 {
        let mut distance = 0.0;
        
        // Check left/right
        if aabb1.max.x < aabb2.min.x {
            distance += (aabb2.min.x - aabb1.max.x).powi(2);
        } else if aabb1.min.x > aabb2.max.x {
            distance += (aabb1.min.x - aabb2.max.x).powi(2);
        }
        
        // Check top/bottom
        if aabb1.max.y < aabb2.min.y {
            distance += (aabb2.min.y - aabb1.max.y).powi(2);
        } else if aabb1.min.y > aabb2.max.y {
            distance += (aabb1.min.y - aabb2.max.y).powi(2);
        }
        
        distance.sqrt()
    }

    /// Check if circle and polygon overlap.
    #[inline]
    fn circle_polygon_overlap(
        circle: &Circle,
        position_a: Vec2,
        polygon: &Polygon,
        position_b: Vec2,
        rotation_b: f32,
    ) -> bool {
        // Transform circle position to polygon local space
        let local_point = position_a - position_b;
        let local_point_rotated = local_point.rotate(-rotation_b);
        
        // Find closest point on polygon
        let closest = polygon.closest_point(local_point_rotated);
        
        // Check distance
        let delta = local_point_rotated - closest;
        delta.length_squared() <= circle.radius() * circle.radius()
    }

    /// Check if two polygons overlap using SAT.
    #[inline]
    fn polygon_polygon_overlap(
        polygon_a: &Polygon,
        position_a: Vec2,
        rotation_a: f32,
        polygon_b: &Polygon,
        position_b: Vec2,
        rotation_b: f32,
    ) -> bool {
        // Use a simple AABB check for now
        // Full SAT implementation would be more accurate but slower
        
        let aabb_a = polygon_a.aabb(Transform::new(position_a, Rot2::new(rotation_a)));
        let aabb_b = polygon_b.aabb(Transform::new(position_b, Rot2::new(rotation_b)));

        aabb_a.overlaps(aabb_b)
    }
}

impl Default for TimeOfImpact {
    fn default() -> Self {
        Self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::Circle;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_toi_result() {
        let result = ToiResult::new(0.5, true, 10);
        assert!(approx_eq(result.toi, 0.5));
        assert!(result.is_colliding());
        assert_eq!(result.iterations, 10);
    }

    #[test]
    fn test_toi_result_default() {
        let result = ToiResult::default();
        assert!(approx_eq(result.toi, 1.0));
        assert!(!result.is_colliding());
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn test_circle_circle_toi_colliding() {
        let toi = TimeOfImpact;
        
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);
        
        // Circle A at origin, Circle B at (3, 0) moving toward A at 2 m/s
        let position_a = Vec2::zero();
        let velocity_a = Vec2::zero();
        let position_b = Vec2::new(3.0, 0.0);
        let velocity_b = Vec2::new(-2.0, 0.0);
        
        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );
        
        assert!(result.is_colliding());
        assert!(result.toi > 0.0);
        assert!(result.toi < 1.0);
    }

    #[test]
    fn test_circle_circle_toi_not_moving_toward() {
        let toi = TimeOfImpact;
        
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);
        
        // Circle A at origin, Circle B at (3, 0) moving away from A
        let position_a = Vec2::zero();
        let velocity_a = Vec2::zero();
        let position_b = Vec2::new(3.0, 0.0);
        let velocity_b = Vec2::new(1.0, 0.0);
        
        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );
        
        assert!(!result.is_colliding());
    }

    #[test]
    fn test_circle_circle_toi_already_overlapping() {
        let toi = TimeOfImpact;
        
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);
        
        // Circles already overlapping
        let position_a = Vec2::zero();
        let velocity_a = Vec2::zero();
        let position_b = Vec2::new(1.0, 0.0);
        let velocity_b = Vec2::zero();
        
        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );
        
        assert!(result.is_colliding());
        assert!(approx_eq(result.toi, 0.0));
    }

    #[test]
    fn test_toi_display() {
        let result = ToiResult::new(0.123456, true, 5);
        let result_str = format!("{}", result);
        assert!(result_str.contains("ToiResult"));
        assert!(result_str.contains("0.123456"));
        assert!(result_str.contains("true"));
        assert!(result_str.contains("5"));
    }
}
