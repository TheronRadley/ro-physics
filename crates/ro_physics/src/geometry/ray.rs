//! Ray implementation for spatial queries.
//!
//! This module provides a ray type for ray casting operations.

use std::fmt;

use crate::math::Vec2;

/// A ray in 2D space.
///
/// A ray is defined by an origin point and a direction vector.
/// The direction does not need to be normalized.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Ray {
    /// Origin point of the ray
    pub origin: Vec2,
    /// Direction vector of the ray
    pub direction: Vec2,
}

impl Ray {
    /// Create a new ray with the given origin and direction
    #[inline]
    #[must_use]
    pub const fn new(origin: Vec2, direction: Vec2) -> Self {
        Self { origin, direction }
    }

    /// Get the origin point
    #[inline]
    #[must_use]
    pub const fn origin(self) -> Vec2 {
        self.origin
    }

    /// Get the direction vector
    #[inline]
    #[must_use]
    pub const fn direction(self) -> Vec2 {
        self.direction
    }

    /// Get a point at a given distance along the ray
    #[inline]
    #[must_use]
    pub fn point_at(self, distance: f32) -> Vec2 {
        self.origin + self.direction.normalize() * distance
    }

    /// Get a point at a given fraction of the direction vector
    #[inline]
    #[must_use]
    pub fn point_at_fraction(self, fraction: f32) -> Vec2 {
        self.origin + self.direction * fraction
    }

    /// Check if the ray is valid (direction is not zero)
    #[inline]
    #[must_use]
    pub fn is_valid(self) -> bool {
        !self.direction.is_zero()
    }

    /// Get the normalized direction
    #[inline]
    #[must_use]
    pub fn normalized_direction(self) -> Vec2 {
        self.direction.normalize()
    }

    /// Check if the ray is finite (no NaN or infinity)
    #[inline]
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.origin.is_finite() && self.direction.is_finite()
    }

    /// Validate that the ray is finite and within reasonable bounds
    #[inline]
    #[must_use]
    pub fn validate(self) -> bool {
        self.origin.validate() && self.direction.validate()
    }
}

impl Default for Ray {
    fn default() -> Self {
        Self {
            origin: Vec2::zero(),
            direction: Vec2::unit_x(),
        }
    }
}

impl fmt::Display for Ray {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Ray(origin: {}, direction: {})",
            self.origin, self.direction
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::EPSILON;
    use crate::math::utils::{approx_eq, approx_eq_eps};

    #[test]
    fn test_construction() {
        let ray1 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        assert_eq!(ray1.origin, Vec2::new(0.0, 0.0));
        assert_eq!(ray1.direction, Vec2::new(1.0, 0.0));

        let ray2 = Ray::default();
        assert_eq!(ray2.origin, Vec2::zero());
        assert_eq!(ray2.direction, Vec2::unit_x());
    }

    #[test]
    fn test_point_at() {
        let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        
        let p1 = ray.point_at(5.0);
        assert_eq!(p1, Vec2::new(5.0, 0.0));

        let p2 = ray.point_at_fraction(0.5);
        assert_eq!(p2, Vec2::new(0.5, 0.0));
    }

    #[test]
    fn test_normalized_direction() {
        let ray1 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 0.0));
        assert_eq!(ray1.normalized_direction(), Vec2::new(1.0, 0.0));

        let ray2 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 3.0));
        assert_eq!(ray2.normalized_direction(), Vec2::new(0.0, 1.0));

        let ray3 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(3.0, 4.0));
        let dir = ray3.normalized_direction();
        assert!(approx_eq(dir.length(), 1.0));
    }

    #[test]
    fn test_validation() {
        let ray1 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        assert!(ray1.is_valid());
        assert!(ray1.is_finite());
        assert!(ray1.validate());

        let ray2 = Ray::new(Vec2::new(0.0, 0.0), Vec2::zero());
        assert!(!ray2.is_valid());

        let ray3 = Ray::new(Vec2::new(f32::NAN, 0.0), Vec2::new(1.0, 0.0));
        assert!(!ray3.is_finite());
        assert!(!ray3.validate());

        let ray4 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(f32::INFINITY, 0.0));
        assert!(!ray4.is_finite());
        assert!(!ray4.validate());

        let ray5 = Ray::new(Vec2::new(1e30, 1e30), Vec2::new(1e30, 1e30));
        assert!(ray5.is_finite());
        assert!(!ray5.validate());
    }

    #[test]
    fn test_equality() {
        let ray1 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        let ray2 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        let ray3 = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0 + EPSILON, 0.0));
        
        assert_eq!(ray1, ray2);
        assert_eq!(ray1, ray3);
    }
}
