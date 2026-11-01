//! Swept collision detection.
//!
//! This module provides swept collision detection for preventing tunneling
//! in fast-moving objects.

use crate::bodies::Body;
use crate::collision::Collider;
use crate::math::{Transform, Vec2};
use crate::shapes::Shape;
use super::{TimeOfImpact, ToiResult, CcdConfig};

/// Swept collision detector.
///
/// This detector checks for collisions between moving shapes over a time interval,
/// preventing fast-moving objects from tunneling through other objects.
#[derive(Debug, Clone)]
pub struct SweptCollisionDetector {
    /// Configuration
    config: CcdConfig,
}

impl SweptCollisionDetector {
    /// Create a new swept collision detector
    #[inline]
    #[must_use]
    pub fn new(config: CcdConfig) -> Self {
        Self { config }
    }

    /// Get the configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &CcdConfig {
        &self.config
    }

    /// Check for swept collision between two bodies.
    ///
    /// Returns the time of impact if a collision will occur within the time step,
    /// or None if no collision will occur.
    #[inline]
    #[must_use]
    pub fn check_swept_collision(
        &self,
        body_a: &Body,
        collider_a: &Collider,
        body_b: &Body,
        collider_b: &Collider,
        dt: f32,
    ) -> Option<ToiResult> {
        if !self.config.enabled {
            return None;
        }

        // A sweep is only meaningful over a positive, finite timestep.
        if !(dt > 0.0 && dt.is_finite()) {
            return None;
        }

        // CCD exists to stop fast movers from passing through each other; when
        // neither body can move there is nothing to sweep, and any existing
        // overlap is the narrow phase's job.
        if !body_a.is_dynamic() && !body_b.is_dynamic() {
            return None;
        }

        let shape_a = collider_a.shape();
        let shape_b = collider_b.shape();

        let transform_a = body_a.transform();
        let transform_b = body_b.transform();

        let velocity_a = body_a.linear_velocity();
        let velocity_b = body_b.linear_velocity();
        let angular_velocity_a = body_a.angular_velocity();
        let angular_velocity_b = body_b.angular_velocity();

        let toi = self.calculate_toi(
            shape_a, transform_a, velocity_a, angular_velocity_a,
            shape_b, transform_b, velocity_b, angular_velocity_b,
        );

        if toi.is_colliding() && toi.toi < 1.0 {
            Some(toi)
        } else {
            None
        }
    }

    /// Calculate time of impact between two shapes.
    #[inline]
    fn calculate_toi(
        &self,
        shape_a: &Shape,
        transform_a: Transform,
        velocity_a: Vec2,
        angular_velocity_a: f32,
        shape_b: &Shape,
        transform_b: Transform,
        velocity_b: Vec2,
        angular_velocity_b: f32,
    ) -> ToiResult {
        match (shape_a, shape_b) {
            (Shape::Circle(circle_a), Shape::Circle(circle_b)) => {
                TimeOfImpact::circle_circle(
                    circle_a,
                    transform_a.position,
                    velocity_a,
                    circle_b,
                    transform_b.position,
                    velocity_b,
                    self.config.max_toi_iterations,
                    self.config.toi_tolerance,
                )
            }
            (Shape::Circle(circle_a), Shape::Polygon(polygon_b)) => {
                TimeOfImpact::circle_polygon(
                    circle_a,
                    transform_a.position,
                    velocity_a,
                    polygon_b,
                    transform_b.position,
                    velocity_b,
                    transform_b.rotation.angle(),
                    angular_velocity_b,
                    self.config.max_toi_iterations,
                    self.config.toi_tolerance,
                )
            }
            (Shape::Polygon(polygon_a), Shape::Circle(circle_b)) => {
                TimeOfImpact::circle_polygon(
                    circle_b,
                    transform_b.position,
                    velocity_b,
                    polygon_a,
                    transform_a.position,
                    velocity_a,
                    transform_a.rotation.angle(),
                    angular_velocity_a,
                    self.config.max_toi_iterations,
                    self.config.toi_tolerance,
                )
            }
            (Shape::Polygon(polygon_a), Shape::Polygon(polygon_b)) => {
                TimeOfImpact::polygon_polygon(
                    polygon_a,
                    transform_a.position,
                    velocity_a,
                    transform_a.rotation.angle(),
                    angular_velocity_a,
                    polygon_b,
                    transform_b.position,
                    velocity_b,
                    transform_b.rotation.angle(),
                    angular_velocity_b,
                    self.config.max_toi_iterations,
                    self.config.toi_tolerance,
                )
            }
        }
    }

    /// Check if a body will tunnel through another body during the time step.
    ///
    /// This is a simplified check that compares the distance moved to the size
    /// of the colliders.
    #[inline]
    #[must_use]
    pub fn will_tunnel(
        &self,
        body_a: &Body,
        collider_a: &Collider,
        body_b: &Body,
        collider_b: &Collider,
        dt: f32,
    ) -> bool {
        if !self.config.enabled {
            return false;
        }

        if !(dt > 0.0 && dt.is_finite()) || (!body_a.is_dynamic() && !body_b.is_dynamic()) {
            return false;
        }

        // What matters is how far the pair moves relative to each other: a
        // collision is missed when that displacement is larger than the
        // smaller of the two shapes, because the pair can then step straight
        // through its own contact configuration.
        let relative_distance = (body_b.linear_velocity() - body_a.linear_velocity()).length() * dt;
        let smallest_size = collider_a
            .characteristic_size()
            .min(collider_b.characteristic_size());

        relative_distance > smallest_size
    }
}

impl Default for SweptCollisionDetector {
    fn default() -> Self {
        Self::new(CcdConfig::default())
    }
}

/// CCD result containing TOI information.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct CcdResult {
    /// Whether CCD was performed
    pub ccd_performed: bool,
    /// Time of impact (if collision detected)
    pub toi: f32,
    /// Whether a collision will occur
    pub will_collide: bool,
    /// Whether tunneling would occur without CCD
    pub would_tunnel: bool,
}

impl CcdResult {
    /// Create a new CCD result
    #[inline]
    #[must_use]
    pub const fn new(
        ccd_performed: bool,
        toi: f32,
        will_collide: bool,
        would_tunnel: bool,
    ) -> Self {
        Self {
            ccd_performed,
            toi,
            will_collide,
            would_tunnel,
        }
    }

    /// Create a result indicating no CCD was performed
    #[inline]
    #[must_use]
    pub const fn no_ccd() -> Self {
        Self {
            ccd_performed: false,
            toi: 1.0,
            will_collide: false,
            would_tunnel: false,
        }
    }
}

impl Default for CcdResult {
    fn default() -> Self {
        Self::no_ccd()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::collision::ColliderHandle;
    use crate::shapes::Circle;

    #[test]
    fn test_construction() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());
        assert!(detector.config().enabled);

        let detector2 = SweptCollisionDetector::default();
        assert!(detector2.config().enabled);
    }

    #[test]
    fn test_swept_collision_circles_moving_toward() {
        let detector = SweptCollisionDetector::new(
            CcdConfig::new()
                .with_enabled(true)
                .with_max_toi_iterations(20)
        );

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(10.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(5.0, 0.0));
        body_b.set_linear_velocity(Vec2::new(-10.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(&body_a, &collider_a, &body_b, &collider_b, 1.0 / 60.0);

        assert!(result.is_some());
        let toi_result = result.unwrap();
        assert!(toi_result.is_colliding());
        assert!(toi_result.toi > 0.0);
        assert!(toi_result.toi < 1.0);
    }

    #[test]
    fn test_swept_collision_circles_moving_away() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(-10.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(5.0, 0.0));
        body_b.set_linear_velocity(Vec2::new(10.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(&body_a, &collider_a, &body_b, &collider_b, 1.0 / 60.0);

        assert!(result.is_none());
    }

    #[test]
    fn test_will_tunnel() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(100.0, 0.0)); // Very fast!

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(5.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let will_tunnel = detector.will_tunnel(&body_a, &collider_a, &body_b, &collider_b, 1.0 / 60.0);

        assert!(will_tunnel);
    }

    #[test]
    fn test_ccd_result() {
        let result = CcdResult::new(true, 0.5, true, false);
        assert!(result.ccd_performed);
        assert!(result.will_collide);
        assert!(!result.would_tunnel);

        let no_ccd = CcdResult::no_ccd();
        assert!(!no_ccd.ccd_performed);
        assert!(!no_ccd.will_collide);
    }

    #[test]
    fn test_ccd_disabled() {
        let detector = SweptCollisionDetector::new(
            CcdConfig::new().with_enabled(false)
        );

        let handle_a = BodyHandle::new(1, 0);
        let body_a = Body::new_dynamic(handle_a);

        let handle_b = BodyHandle::new(2, 0);
        let body_b = Body::new_dynamic(handle_b);

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(&body_a, &collider_a, &body_b, &collider_b, 1.0 / 60.0);

        assert!(result.is_none());
    }
}
