//! Axis-Aligned Bounding Box implementation.
//!
//! This module provides an AABB type for broad-phase collision detection
//! and spatial queries.

use std::fmt;

use crate::constants::EPSILON;
use crate::math::{utils::approx_eq, Vec2};

/// An axis-aligned bounding box.
///
/// AABBs are used extensively in the physics engine for:
/// - Broad-phase collision detection
/// - Spatial queries
/// - Visibility determination
/// - Bounding volume hierarchy
///
/// The AABB is defined by its minimum and maximum corners.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Aabb {
    /// Minimum corner (bottom-left in standard coordinates)
    pub min: Vec2,
    /// Maximum corner (top-right in standard coordinates)
    pub max: Vec2,
}

impl Aabb {
    /// Create a new AABB from minimum and maximum corners
    ///
    /// # Panics
    ///
    /// Panics if min > max in any dimension, or if either corner is
    /// non-finite.
    #[inline]
    #[must_use]
    pub fn new(min: Vec2, max: Vec2) -> Self {
        assert!(
            min.x <= max.x && min.y <= max.y,
            "AABB min must be <= max"
        );
        assert!(
            min.is_finite() && max.is_finite(),
            "AABB corners must be finite"
        );
        Self { min, max }
    }

    /// Create an AABB from a center point and half-extents
    #[inline]
    #[must_use]
    pub fn from_center_half_extents(center: Vec2, half_extents: Vec2) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Create an AABB from a center point and size (width and height)
    #[inline]
    #[must_use]
    pub fn from_center_size(center: Vec2, size: Vec2) -> Self {
        Self::from_center_half_extents(center, size / 2.0)
    }

    /// Create an AABB that contains a single point
    #[inline]
    #[must_use]
    pub fn from_point(point: Vec2) -> Self {
        Self {
            min: point,
            max: point,
        }
    }

    /// Create an invalid AABB (min > max)
    ///
    /// This is useful for initializing AABBs that will be expanded later.
    #[inline]
    #[must_use]
    pub fn invalid() -> Self {
        Self {
            min: Vec2::new(f32::INFINITY, f32::INFINITY),
            max: Vec2::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    /// Get the center of the AABB
    #[inline]
    #[must_use]
    pub fn center(self) -> Vec2 {
        Vec2::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
        )
    }

    /// Get the size (width and height) of the AABB
    #[inline]
    #[must_use]
    pub fn size(self) -> Vec2 {
        self.max - self.min
    }

    /// Get the width of the AABB
    #[inline]
    #[must_use]
    pub fn width(self) -> f32 {
        self.max.x - self.min.x
    }

    /// Get the height of the AABB
    #[inline]
    #[must_use]
    pub fn height(self) -> f32 {
        self.max.y - self.min.y
    }

    /// Get the half-extents of the AABB
    #[inline]
    #[must_use]
    pub fn half_extents(self) -> Vec2 {
        self.size() / 2.0
    }

    /// Get the perimeter of the AABB
    #[inline]
    #[must_use]
    pub fn perimeter(self) -> f32 {
        2.0 * (self.width() + self.height())
    }

    /// Get the area of the AABB
    #[inline]
    #[must_use]
    pub fn area(self) -> f32 {
        self.width() * self.height()
    }

    /// Check if the AABB is valid (min <= max)
    #[inline]
    #[must_use]
    pub fn is_valid(self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y
    }

    /// Check if the AABB has zero area
    #[inline]
    #[must_use]
    pub fn is_degenerate(self) -> bool {
        self.is_degenerate_eps(EPSILON)
    }

    /// Check if the AABB has zero area within an explicit tolerance
    #[inline]
    #[must_use]
    pub fn is_degenerate_eps(self, epsilon: f32) -> bool {
        self.width() <= epsilon || self.height() <= epsilon
    }

    /// Check if the AABB contains a point
    #[inline]
    #[must_use]
    pub fn contains_point(self, point: Vec2) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }

    /// Check if the AABB contains another AABB
    #[inline]
    #[must_use]
    pub fn contains_aabb(self, other: Self) -> bool {
        self.min.x <= other.min.x
            && self.max.x >= other.max.x
            && self.min.y <= other.min.y
            && self.max.y >= other.max.y
    }

    /// Check if this AABB overlaps with another AABB
    #[inline]
    #[must_use]
    pub fn overlaps(self, other: Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    /// Calculate the intersection of this AABB with another
    ///
    /// Returns None if the AABBs don't overlap.
    #[inline]
    #[must_use]
    pub fn intersection(self, other: Self) -> Option<Self> {
        let min_x = self.min.x.max(other.min.x);
        let min_y = self.min.y.max(other.min.y);
        let max_x = self.max.x.min(other.max.x);
        let max_y = self.max.y.min(other.max.y);

        if min_x > max_x || min_y > max_y {
            None
        } else {
            Some(Self::new(Vec2::new(min_x, min_y), Vec2::new(max_x, max_y)))
        }
    }

    /// Calculate the union of this AABB with another
    #[inline]
    #[must_use]
    pub fn union(self, other: Self) -> Self {
        // Unions are used by the spatial structures to accumulate bounds, so
        // they must never panic: invalid (empty) operands are simply ignored.
        if !self.is_valid() {
            return other;
        }
        if !other.is_valid() {
            return self;
        }

        Self {
            min: Vec2::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: Vec2::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }

    /// Get the four corners of the AABB, counter-clockwise from `min`
    #[inline]
    #[must_use]
    pub fn corners(self) -> [Vec2; 4] {
        [
            self.min,
            Vec2::new(self.max.x, self.min.y),
            self.max,
            Vec2::new(self.min.x, self.max.y),
        ]
    }

    /// Expand the AABB to include a point
    #[inline]
    pub fn expand_by_point(&mut self, point: Vec2) {
        self.min.x = self.min.x.min(point.x);
        self.min.y = self.min.y.min(point.y);
        self.max.x = self.max.x.max(point.x);
        self.max.y = self.max.y.max(point.y);
    }

    /// Expand the AABB to include another AABB
    #[inline]
    pub fn expand_by_aabb(&mut self, other: Self) {
        self.min.x = self.min.x.min(other.min.x);
        self.min.y = self.min.y.min(other.min.y);
        self.max.x = self.max.x.max(other.max.x);
        self.max.y = self.max.y.max(other.max.y);
    }

    /// Expand the AABB by a uniform margin
    #[inline]
    pub fn expand_by_margin(&mut self, margin: f32) {
        self.min.x -= margin;
        self.min.y -= margin;
        self.max.x += margin;
        self.max.y += margin;
    }

    /// Expand the AABB by a per-axis margin
    #[inline]
    pub fn expand_by_vector(&mut self, margin: Vec2) {
        self.min.x -= margin.x;
        self.min.y -= margin.y;
        self.max.x += margin.x;
        self.max.y += margin.y;
    }

    /// Translate the AABB by a vector
    #[inline]
    pub fn translate(&mut self, vector: Vec2) {
        self.min += vector;
        self.max += vector;
    }

    /// Scale the AABB by a scalar (from the center)
    #[inline]
    pub fn scale(&mut self, scalar: f32) {
        let center = self.center();
        let half_size = self.size() * scalar / 2.0;
        self.min = center - half_size;
        self.max = center + half_size;
    }

    /// Calculate the closest point on the AABB to a given point
    #[inline]
    #[must_use]
    pub fn closest_point(self, point: Vec2) -> Vec2 {
        Vec2::new(
            point.x.clamp(self.min.x, self.max.x),
            point.y.clamp(self.min.y, self.max.y),
        )
    }

    /// Calculate the distance from a point to the AABB
    #[inline]
    #[must_use]
    pub fn distance_to_point(self, point: Vec2) -> f32 {
        if self.contains_point(point) {
            0.0
        } else {
            let closest = self.closest_point(point);
            (point - closest).length()
        }
    }

    /// Calculate the squared distance from a point to the AABB
    #[inline]
    #[must_use]
    pub fn distance_squared_to_point(self, point: Vec2) -> f32 {
        if self.contains_point(point) {
            0.0
        } else {
            let closest = self.closest_point(point);
            (point - closest).length_squared()
        }
    }

    /// Ray cast against the AABB
    ///
    /// Returns the fraction of the ray direction to the intersection point,
    /// or None if there's no intersection.
    #[inline]
    #[must_use]
    pub fn ray_cast(
        self,
        origin: Vec2,
        direction: Vec2,
        max_fraction: f32,
    ) -> Option<f32> {
        use crate::math::utils::safe_divide;

        let inv_dir = Vec2::new(
            safe_divide(1.0, direction.x),
            safe_divide(1.0, direction.y),
        );
        
        let mut t_min = 0.0;
        let mut t_max = max_fraction;

        // X axis
        let t1 = (self.min.x - origin.x) * inv_dir.x;
        let t2 = (self.max.x - origin.x) * inv_dir.x;
        
        t_min = t1.min(t2).max(t_min);
        t_max = t1.max(t2).min(t_max);

        if t_min > t_max {
            return None;
        }

        // Y axis
        let t1 = (self.min.y - origin.y) * inv_dir.y;
        let t2 = (self.max.y - origin.y) * inv_dir.y;
        
        t_min = t1.min(t2).max(t_min);
        t_max = t1.max(t2).min(t_max);

        if t_min > t_max {
            None
        } else if t_min < 0.0 {
            // Ray origin is inside the AABB
            Some(0.0)
        } else {
            Some(t_min)
        }
    }

    /// Check if the AABB is finite (no NaN or infinity)
    #[inline]
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.min.is_finite() && self.max.is_finite()
    }

    /// Validate that the AABB is finite and within reasonable bounds
    #[inline]
    #[must_use]
    pub fn validate(self) -> bool {
        if !self.is_finite() {
            return false;
        }
        self.min.validate() && self.max.validate()
    }

    /// Convert to an array of 4 values [min_x, min_y, max_x, max_y]
    #[inline]
    #[must_use]
    pub fn to_array(self) -> [f32; 4] {
        [self.min.x, self.min.y, self.max.x, self.max.y]
    }

    /// Create from an array of 4 values [min_x, min_y, max_x, max_y]
    #[inline]
    #[must_use]
    pub fn from_array(arr: [f32; 4]) -> Self {
        Self {
            min: Vec2::new(arr[0], arr[1]),
            max: Vec2::new(arr[2], arr[3]),
        }
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::invalid()
    }
}

impl fmt::Display for Aabb {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Aabb(min: {}, max: {})",
            self.min, self.max
        )
    }
}

impl PartialEq for Aabb {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        approx_eq(self.min.x, other.min.x)
            && approx_eq(self.min.y, other.min.y)
            && approx_eq(self.max.x, other.max.x)
            && approx_eq(self.max.y, other.max.y)
    }
}

impl Eq for Aabb {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::EPSILON;
    use crate::math::utils::{approx_eq, approx_eq_eps};

    #[test]
    fn test_construction() {
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        assert_eq!(aabb1.min, Vec2::new(0.0, 0.0));
        assert_eq!(aabb1.max, Vec2::new(10.0, 10.0));

        let aabb2 = Aabb::from_center_half_extents(Vec2::new(5.0, 5.0), Vec2::new(5.0, 5.0));
        assert_eq!(aabb2.min, Vec2::new(0.0, 0.0));
        assert_eq!(aabb2.max, Vec2::new(10.0, 10.0));

        let aabb3 = Aabb::from_center_size(Vec2::new(5.0, 5.0), Vec2::new(10.0, 10.0));
        assert_eq!(aabb3.min, Vec2::new(0.0, 0.0));
        assert_eq!(aabb3.max, Vec2::new(10.0, 10.0));

        let aabb4 = Aabb::from_point(Vec2::new(5.0, 5.0));
        assert_eq!(aabb4.min, Vec2::new(5.0, 5.0));
        assert_eq!(aabb4.max, Vec2::new(5.0, 5.0));
    }

    #[test]
    fn test_properties() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 20.0));
        
        assert_eq!(aabb.center(), Vec2::new(5.0, 10.0));
        assert_eq!(aabb.size(), Vec2::new(10.0, 20.0));
        assert_eq!(aabb.width(), 10.0);
        assert_eq!(aabb.height(), 20.0);
        assert_eq!(aabb.half_extents(), Vec2::new(5.0, 10.0));
        assert_eq!(aabb.perimeter(), 60.0);
        assert_eq!(aabb.area(), 200.0);
    }

    #[test]
    fn test_validation() {
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        assert!(aabb1.is_valid());
        assert!(!aabb1.is_degenerate());
        assert!(aabb1.is_finite());
        assert!(aabb1.validate());

        let aabb2 = Aabb::invalid();
        assert!(!aabb2.is_valid());

        let aabb3 = Aabb::new(Vec2::new(10.0, 10.0), Vec2::new(0.0, 0.0));
        assert!(!aabb3.is_valid());

        let aabb4 = Aabb::from_point(Vec2::new(5.0, 5.0));
        assert!(aabb4.is_valid());
        assert!(aabb4.is_degenerate());

        let aabb5 = Aabb::new(Vec2::new(f32::NAN, 0.0), Vec2::new(10.0, 10.0));
        assert!(!aabb5.is_finite());
        assert!(!aabb5.validate());

        let aabb6 = Aabb::new(Vec2::new(1e30, 1e30), Vec2::new(1e30, 1e30));
        assert!(aabb6.is_finite());
        assert!(!aabb6.validate());
    }

    #[test]
    fn test_contains() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        
        assert!(aabb.contains_point(Vec2::new(5.0, 5.0)));
        assert!(aabb.contains_point(Vec2::new(0.0, 0.0)));
        assert!(aabb.contains_point(Vec2::new(10.0, 10.0)));
        assert!(!aabb.contains_point(Vec2::new(-1.0, 5.0)));
        assert!(!aabb.contains_point(Vec2::new(5.0, 15.0)));
        assert!(!aabb.contains_point(Vec2::new(15.0, 5.0)));

        let smaller = Aabb::new(Vec2::new(2.0, 2.0), Vec2::new(8.0, 8.0));
        assert!(aabb.contains_aabb(smaller));
        
        let larger = Aabb::new(Vec2::new(-5.0, -5.0), Vec2::new(15.0, 15.0));
        assert!(!aabb.contains_aabb(larger));
    }

    #[test]
    fn test_overlaps() {
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        let aabb3 = Aabb::new(Vec2::new(-10.0, -10.0), Vec2::new(-5.0, -5.0));
        let aabb4 = Aabb::new(Vec2::new(15.0, 15.0), Vec2::new(20.0, 20.0));
        
        assert!(aabb1.overlaps(aabb2));
        assert!(!aabb1.overlaps(aabb3));
        assert!(!aabb1.overlaps(aabb4));
        assert!(aabb1.overlaps(aabb1));
    }

    #[test]
    fn test_intersection() {
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        
        let intersection = aabb1.intersection(aabb2);
        assert!(intersection.is_some());
        let int = intersection.unwrap();
        assert_eq!(int.min, Vec2::new(5.0, 5.0));
        assert_eq!(int.max, Vec2::new(10.0, 10.0));

        let aabb3 = Aabb::new(Vec2::new(-10.0, -10.0), Vec2::new(-5.0, -5.0));
        let intersection2 = aabb1.intersection(aabb3);
        assert!(intersection2.is_none());
    }

    #[test]
    fn test_union() {
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        
        let union = aabb1.union(aabb2);
        assert_eq!(union.min, Vec2::new(0.0, 0.0));
        assert_eq!(union.max, Vec2::new(15.0, 15.0));
    }

    #[test]
    fn test_expand() {
        let mut aabb = Aabb::from_point(Vec2::new(5.0, 5.0));
        
        aabb.expand_by_point(Vec2::new(10.0, 10.0));
        assert_eq!(aabb.min, Vec2::new(5.0, 5.0));
        assert_eq!(aabb.max, Vec2::new(10.0, 10.0));

        aabb.expand_by_point(Vec2::new(0.0, 0.0));
        assert_eq!(aabb.min, Vec2::new(0.0, 0.0));
        assert_eq!(aabb.max, Vec2::new(10.0, 10.0));

        let aabb2 = Aabb::new(Vec2::new(-5.0, -5.0), Vec2::new(-1.0, -1.0));
        aabb.expand_by_aabb(aabb2);
        assert_eq!(aabb.min, Vec2::new(-5.0, -5.0));
        assert_eq!(aabb.max, Vec2::new(10.0, 10.0));

        aabb.expand_by_margin(2.0);
        assert_eq!(aabb.min, Vec2::new(-7.0, -7.0));
        assert_eq!(aabb.max, Vec2::new(12.0, 12.0));

        aabb.expand_by_vector(Vec2::new(1.0, 2.0));
        assert_eq!(aabb.min, Vec2::new(-8.0, -9.0));
        assert_eq!(aabb.max, Vec2::new(13.0, 14.0));
    }

    #[test]
    fn test_translate() {
        let mut aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        aabb.translate(Vec2::new(5.0, 5.0));
        assert_eq!(aabb.min, Vec2::new(5.0, 5.0));
        assert_eq!(aabb.max, Vec2::new(15.0, 15.0));
    }

    #[test]
    fn test_scale() {
        let mut aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        aabb.scale(2.0);
        assert_eq!(aabb.min, Vec2::new(-5.0, -5.0));
        assert_eq!(aabb.max, Vec2::new(15.0, 15.0));

        aabb.scale(0.5);
        assert_eq!(aabb.min, Vec2::new(-2.5, -2.5));
        assert_eq!(aabb.max, Vec2::new(7.5, 7.5));
    }

    #[test]
    fn test_closest_point() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        
        // Point inside
        assert_eq!(aabb.closest_point(Vec2::new(5.0, 5.0)), Vec2::new(5.0, 5.0));
        
        // Point outside (right)
        assert_eq!(aabb.closest_point(Vec2::new(15.0, 5.0)), Vec2::new(10.0, 5.0));
        
        // Point outside (left)
        assert_eq!(aabb.closest_point(Vec2::new(-5.0, 5.0)), Vec2::new(0.0, 5.0));
        
        // Point outside (top)
        assert_eq!(aabb.closest_point(Vec2::new(5.0, 15.0)), Vec2::new(5.0, 10.0));
        
        // Point outside (bottom)
        assert_eq!(aabb.closest_point(Vec2::new(5.0, -5.0)), Vec2::new(5.0, 0.0));
        
        // Point outside (corner)
        assert_eq!(aabb.closest_point(Vec2::new(15.0, 15.0)), Vec2::new(10.0, 10.0));
    }

    #[test]
    fn test_distance() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        
        // Point inside
        assert_eq!(aabb.distance_to_point(Vec2::new(5.0, 5.0)), 0.0);
        
        // Point outside
        assert!(approx_eq_eps(aabb.distance_to_point(Vec2::new(15.0, 15.0)), 5.0 * (2.0_f32).sqrt(), 1e-5));
        
        // Point at edge
        assert!(approx_eq(aabb.distance_to_point(Vec2::new(10.0, 5.0)), 0.0));
    }

    #[test]
    fn test_ray_cast() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        
        // Ray from outside, hitting the AABB
        let hit = aabb.ray_cast(Vec2::new(-5.0, 5.0), Vec2::new(1.0, 0.0), 100.0);
        assert!(hit.is_some());
        assert!(approx_eq_eps(hit.unwrap(), 5.0, 1e-5));
        
        // Ray from inside
        let hit2 = aabb.ray_cast(Vec2::new(5.0, 5.0), Vec2::new(1.0, 0.0), 100.0);
        assert!(hit2.is_some());
        assert!(approx_eq_eps(hit2.unwrap(), 0.0, 1e-5));
        
        // Ray missing the AABB
        let hit3 = aabb.ray_cast(Vec2::new(-5.0, 15.0), Vec2::new(1.0, 0.0), 100.0);
        assert!(hit3.is_none());
        
        // Ray with limited range
        let hit4 = aabb.ray_cast(Vec2::new(-5.0, 5.0), Vec2::new(1.0, 0.0), 3.0);
        assert!(hit4.is_none());
    }

    #[test]
    fn test_array_conversion() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let arr = aabb.to_array();
        assert_eq!(arr, [0.0, 0.0, 10.0, 10.0]);

        let aabb2 = Aabb::from_array(arr);
        assert_eq!(aabb2, aabb);
    }

    #[test]
    fn test_equality() {
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let aabb2 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let aabb3 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0 + EPSILON, 10.0));
        
        assert_eq!(aabb1, aabb2);
        assert_eq!(aabb1, aabb3);
    }
}
