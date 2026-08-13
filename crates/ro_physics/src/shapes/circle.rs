//! Circle shape implementation.
//!
//! This module provides a circle shape for collision detection
//! and physics simulation.

use std::fmt;

use crate::constants::EPSILON;
use crate::geometry::Aabb;
use crate::math::Vec2;
use crate::shapes::{MassProperties, ShapeType};

/// A circle shape.
///
/// Circles are defined by their radius and are the simplest
/// shape for collision detection.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle {
    /// Radius of the circle
    pub radius: f32,
}

impl Circle {
    /// Create a new circle with the given radius
    ///
    /// # Panics
    ///
    /// Panics if radius is negative.
    #[inline]
    #[must_use]
    pub fn new(radius: f32) -> Self {
        debug_assert!(
            radius >= 0.0,
            "Circle radius must be non-negative"
        );
        Self { radius }
    }

    /// Get the radius
    #[inline]
    #[must_use]
    pub const fn radius(self) -> f32 {
        self.radius
    }

    /// Get the diameter
    #[inline]
    #[must_use]
    pub fn diameter(self) -> f32 {
        self.radius * 2.0
    }

    /// Get the area
    #[inline]
    #[must_use]
    pub fn area(self) -> f32 {
        crate::constants::PI * self.radius * self.radius
    }

    /// Get the circumference
    #[inline]
    #[must_use]
    pub fn circumference(self) -> f32 {
        2.0 * crate::constants::PI * self.radius
    }

    /// Calculate the moment of inertia for a given mass
    #[inline]
    #[must_use]
    pub fn inertia(self, mass: f32) -> f32 {
        0.5 * mass * self.radius * self.radius
    }

    /// Calculate mass properties for a given density
    #[inline]
    #[must_use]
    pub fn mass_properties(self, density: f32) -> MassProperties {
        let area = self.area();
        let mass = area * density;
        let inertia = self.inertia(mass);

        MassProperties::new(mass, inertia, Vec2::zero())
    }

    /// Calculate the AABB for the circle at a given position
    #[inline]
    #[must_use]
    pub fn aabb(self, position: Vec2) -> Aabb {
        Aabb::from_center_half_extents(position, Vec2::splat(self.radius))
    }

    /// Check if a point is inside the circle
    #[inline]
    #[must_use]
    pub fn contains_point(self, point: Vec2, center: Vec2) -> bool {
        (point - center).length_squared() <= self.radius * self.radius
    }

    /// Find the closest point on the circle to a given point
    #[inline]
    #[must_use]
    pub fn closest_point(self, point: Vec2, center: Vec2) -> Vec2 {
        let to_point = point - center;
        let distance = to_point.length();

        if distance <= self.radius {
            point
        } else {
            center + to_point.normalize() * self.radius
        }
    }

    /// Calculate the distance from a point to the circle
    #[inline]
    #[must_use]
    pub fn distance_to_point(self, point: Vec2, center: Vec2) -> f32 {
        let distance = (point - center).length();
        (distance - self.radius).max(0.0)
    }

    /// Check if the circle is valid
    #[inline]
    #[must_use]
    pub fn is_valid(self) -> bool {
        self.radius >= 0.0 && self.radius.is_finite()
    }

    /// Validate the circle
    #[inline]
    #[must_use]
    pub fn validate(self) -> bool {
        if !self.radius.is_finite() {
            return false;
        }
        self.radius >= 0.0 && self.radius < crate::constants::MAX_FLOAT
    }

    /// Get the shape type
    #[inline]
    #[must_use]
    pub const fn shape_type() -> ShapeType {
        ShapeType::Circle
    }
}

impl Default for Circle {
    fn default() -> Self {
        Self { radius: 1.0 }
    }
}

impl fmt::Display for Circle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Circle(radius: {:.4})", self.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_construction() {
        let circle1 = Circle::new(5.0);
        assert_eq!(circle1.radius, 5.0);

        let circle2 = Circle::default();
        assert_eq!(circle2.radius, 1.0);
    }

    #[test]
    #[should_panic]
    fn test_negative_radius() {
        Circle::new(-1.0);
    }

    #[test]
    fn test_properties() {
        let circle = Circle::new(5.0);
        
        assert_eq!(circle.diameter(), 10.0);
        assert!(approx_eq(circle.area(), 78.53981633974483));
        assert!(approx_eq(circle.circumference(), 31.41592653589793));
    }

    #[test]
    fn test_inertia() {
        let circle = Circle::new(5.0);
        let inertia = circle.inertia(10.0);
        // I = 0.5 * m * r^2 = 0.5 * 10 * 25 = 125
        assert_eq!(inertia, 125.0);
    }

    #[test]
    fn test_mass_properties() {
        let circle = Circle::new(5.0);
        let density = 2.0;
        let mp = circle.mass_properties(density);
        
        let expected_area = circle.area();
        let expected_mass = expected_area * density;
        let expected_inertia = circle.inertia(expected_mass);
        
        assert!(approx_eq(mp.mass, expected_mass));
        assert!(approx_eq(mp.inertia, expected_inertia));
        assert_eq!(mp.center_of_mass, Vec2::zero());
    }

    #[test]
    fn test_aabb() {
        let circle = Circle::new(5.0);
        let center = Vec2::new(10.0, 20.0);
        let aabb = circle.aabb(center);
        
        assert_eq!(aabb.min, Vec2::new(5.0, 15.0));
        assert_eq!(aabb.max, Vec2::new(15.0, 25.0));
    }

    #[test]
    fn test_contains_point() {
        let circle = Circle::new(5.0);
        let center = Vec2::new(10.0, 10.0);
        
        // Point at center
        assert!(circle.contains_point(center, center));
        
        // Point on edge
        assert!(circle.contains_point(Vec2::new(15.0, 10.0), center));
        
        // Point inside
        assert!(circle.contains_point(Vec2::new(12.0, 10.0), center));
        
        // Point outside
        assert!(!circle.contains_point(Vec2::new(20.0, 10.0), center));
    }

    #[test]
    fn test_closest_point() {
        let circle = Circle::new(5.0);
        let center = Vec2::new(10.0, 10.0);
        
        // Point at center
        assert_eq!(
            circle.closest_point(center, center),
            center
        );
        
        // Point inside
        assert_eq!(
            circle.closest_point(Vec2::new(12.0, 10.0), center),
            Vec2::new(12.0, 10.0)
        );
        
        // Point outside
        let closest = circle.closest_point(Vec2::new(20.0, 10.0), center);
        assert!(approx_eq(closest.x, 15.0));
        assert!(approx_eq(closest.y, 10.0));
    }

    #[test]
    fn test_distance_to_point() {
        let circle = Circle::new(5.0);
        let center = Vec2::new(10.0, 10.0);
        
        // Point at center
        assert_eq!(circle.distance_to_point(center, center), 0.0);
        
        // Point on edge
        assert_eq!(circle.distance_to_point(Vec2::new(15.0, 10.0), center), 0.0);
        
        // Point outside
        assert_eq!(circle.distance_to_point(Vec2::new(20.0, 10.0), center), 5.0);
    }

    #[test]
    fn test_validation() {
        let circle1 = Circle::new(5.0);
        assert!(circle1.is_valid());
        assert!(circle1.validate());

        let circle2 = Circle::new(0.0);
        assert!(circle2.is_valid());
        assert!(circle2.validate());

        let circle3 = Circle::new(f32::NAN);
        assert!(!circle3.is_valid());
        assert!(!circle3.validate());

        let circle4 = Circle::new(f32::INFINITY);
        assert!(!circle4.is_valid());
        assert!(!circle4.validate());

        let circle5 = Circle::new(1e30);
        assert!(circle5.is_valid());
        assert!(!circle5.validate());
    }

    #[test]
    fn test_shape_type() {
        assert_eq!(Circle::shape_type(), ShapeType::Circle);
    }
}
