//! Shape trait and enum definitions.
//!
//! This module provides the Shape trait and ShapeType enum for
//! working with different shape types in a unified way.

use std::fmt;

use crate::math::{Rot2, Transform, Vec2};
use crate::geometry::Aabb;
use crate::shapes::{Circle, MassProperties, Polygon};

/// Type of shape
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub enum ShapeType {
    /// Circle shape
    Circle,
    /// Polygon shape
    Polygon,
}

impl fmt::Display for ShapeType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ShapeType::Circle => write!(f, "Circle"),
            ShapeType::Polygon => write!(f, "Polygon"),
        }
    }
}

/// Enum representing any shape type
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub enum Shape {
    /// Circle shape variant
    Circle(Circle),
    /// Polygon shape variant
    Polygon(Polygon),
}

impl Shape {
    /// Get the shape type
    #[inline]
    #[must_use]
    pub fn shape_type(&self) -> ShapeType {
        match self {
            Shape::Circle(_) => ShapeType::Circle,
            Shape::Polygon(_) => ShapeType::Polygon,
        }
    }

    /// Check if the shape is a circle
    #[inline]
    #[must_use]
    pub fn is_circle(&self) -> bool {
        matches!(self, Shape::Circle(_))
    }

    /// Check if the shape is a polygon
    #[inline]
    #[must_use]
    pub fn is_polygon(&self) -> bool {
        matches!(self, Shape::Polygon(_))
    }

    /// Get the circle shape, if this is a circle
    #[inline]
    #[must_use]
    pub fn as_circle(&self) -> Option<&Circle> {
        match self {
            Shape::Circle(c) => Some(c),
            _ => None,
        }
    }

    /// Get the polygon shape, if this is a polygon
    #[inline]
    #[must_use]
    pub fn as_polygon(&self) -> Option<&Polygon> {
        match self {
            Shape::Polygon(p) => Some(p),
            _ => None,
        }
    }

    /// Get the circle shape mutably, if this is a circle
    #[inline]
    #[must_use]
    pub fn as_circle_mut(&mut self) -> Option<&mut Circle> {
        match self {
            Shape::Circle(c) => Some(c),
            _ => None,
        }
    }

    /// Get the polygon shape mutably, if this is a polygon
    #[inline]
    #[must_use]
    pub fn as_polygon_mut(&mut self) -> Option<&mut Polygon> {
        match self {
            Shape::Polygon(p) => Some(p),
            _ => None,
        }
    }

    /// Calculate the AABB for the shape in local space
    #[inline]
    #[must_use]
    pub fn local_aabb(&self) -> Aabb {
        match self {
            Shape::Circle(c) => c.aabb(Vec2::zero()),
            Shape::Polygon(p) => p.local_aabb(),
        }
    }

    /// Calculate the AABB for the shape at a given transform
    #[inline]
    #[must_use]
    pub fn aabb(&self, transform: Transform) -> Aabb {
        match self {
            Shape::Circle(c) => c.aabb(transform.position),
            Shape::Polygon(p) => p.aabb(transform),
        }
    }

    /// Check if a point is inside the shape
    #[inline]
    #[must_use]
    pub fn contains_point(&self, point: Vec2, transform: Transform) -> bool {
        match self {
            Shape::Circle(c) => c.contains_point(point, transform.position),
            Shape::Polygon(p) => {
                let local_point = transform.inverse_transform_point(point);
                p.contains_point(local_point)
            }
        }
    }

    /// Find the closest point on the shape to a given point
    #[inline]
    #[must_use]
    pub fn closest_point(&self, point: Vec2, transform: Transform) -> Vec2 {
        match self {
            Shape::Circle(c) => {
                // A circle is symmetric about its centre, so the closest point
                // can be computed directly in world space.
                c.closest_point(point, transform.position)
            }
            Shape::Polygon(p) => {
                let local_point = transform.inverse_transform_point(point);
                let closest_local = p.closest_point(local_point);
                transform.transform_point(closest_local)
            }
        }
    }

    /// Calculate mass properties for a given density
    #[inline]
    #[must_use]
    pub fn mass_properties(&self, density: f32) -> MassProperties {
        match self {
            Shape::Circle(c) => c.mass_properties(density),
            Shape::Polygon(p) => p.mass_properties(density),
        }
    }

    /// Calculate the moment of inertia for a given mass
    #[inline]
    #[must_use]
    pub fn inertia(&self, mass: f32) -> f32 {
        match self {
            Shape::Circle(c) => c.inertia(mass),
            Shape::Polygon(p) => p.inertia(mass),
        }
    }

    /// Get the support point in a given direction
    #[inline]
    #[must_use]
    pub fn support(&self, direction: Vec2) -> Vec2 {
        match self {
            Shape::Circle(c) => {
                // For a circle, the support point is in the direction from center
                direction.normalize() * c.radius
            }
            Shape::Polygon(p) => p.support(direction),
        }
    }

    /// Check if the shape is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        match self {
            Shape::Circle(c) => c.is_valid(),
            Shape::Polygon(p) => p.is_valid(),
        }
    }

    /// Validate the shape
    #[inline]
    #[must_use]
    pub fn validate(&self) -> bool {
        match self {
            Shape::Circle(c) => c.validate(),
            Shape::Polygon(p) => p.validate(),
        }
    }

    /// Get the area of the shape
    #[inline]
    #[must_use]
    pub fn area(&self) -> f32 {
        match self {
            Shape::Circle(c) => c.area(),
            Shape::Polygon(p) => p.area(),
        }
    }

    /// Clone the shape as a boxed trait object
    #[inline]
    #[must_use]
    pub fn clone_box(&self) -> Box<dyn ShapeTrait> {
        match self {
            Shape::Circle(c) => Box::new(c.clone()),
            Shape::Polygon(p) => Box::new(p.clone()),
        }
    }
}

impl Default for Shape {
    fn default() -> Self {
        Shape::Circle(Circle::default())
    }
}

impl fmt::Display for Shape {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Shape::Circle(c) => write!(f, "Shape::{}", c),
            Shape::Polygon(p) => write!(f, "Shape::{}", p),
        }
    }
}

/// Trait for shape operations
pub trait ShapeTrait: std::fmt::Debug + Send + Sync {
    /// Get the shape type
    fn shape_type(&self) -> ShapeType;

    /// Calculate the AABB for the shape in local space
    fn local_aabb(&self) -> Aabb;

    /// Calculate the AABB for the shape at a given transform
    fn aabb(&self, transform: Transform) -> Aabb;

    /// Check if a point is inside the shape
    fn contains_point(&self, point: Vec2, transform: Transform) -> bool;

    /// Find the closest point on the shape to a given point
    fn closest_point(&self, point: Vec2, transform: Transform) -> Vec2;

    /// Calculate mass properties for a given density
    fn mass_properties(&self, density: f32) -> MassProperties;

    /// Calculate the moment of inertia for a given mass
    fn inertia(&self, mass: f32) -> f32;

    /// Get the support point in a given direction
    fn support(&self, direction: Vec2) -> Vec2;

    /// Check if the shape is valid
    fn is_valid(&self) -> bool;

    /// Validate the shape
    fn validate(&self) -> bool;

    /// Get the area of the shape
    fn area(&self) -> f32;

    /// Clone the shape
    fn clone_box(&self) -> Box<dyn ShapeTrait>;
}

impl ShapeTrait for Circle {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Circle
    }

    fn local_aabb(&self) -> Aabb {
        self.aabb(Vec2::zero())
    }

    fn aabb(&self, transform: Transform) -> Aabb {
        self.aabb(transform.position)
    }

    fn contains_point(&self, point: Vec2, transform: Transform) -> bool {
        self.contains_point(point, transform.position)
    }

    fn closest_point(&self, point: Vec2, transform: Transform) -> Vec2 {
        // A circle is centred on the transform origin, so the closest point is
        // already expressed in world space.
        self.closest_point(point, transform.position)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        self.mass_properties(density)
    }

    fn inertia(&self, mass: f32) -> f32 {
        self.inertia(mass)
    }

    fn support(&self, direction: Vec2) -> Vec2 {
        direction.normalize() * self.radius
    }

    fn is_valid(&self) -> bool {
        self.is_valid()
    }

    fn validate(&self) -> bool {
        self.validate()
    }

    fn area(&self) -> f32 {
        self.area()
    }

    fn clone_box(&self) -> Box<dyn ShapeTrait> {
        Box::new(self.clone())
    }
}

impl ShapeTrait for Polygon {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Polygon
    }

    fn local_aabb(&self) -> Aabb {
        self.local_aabb()
    }

    fn aabb(&self, transform: Transform) -> Aabb {
        self.aabb(transform)
    }

    fn contains_point(&self, point: Vec2, transform: Transform) -> bool {
        let local_point = transform.inverse_transform_point(point);
        self.contains_point(local_point)
    }

    fn closest_point(&self, point: Vec2, transform: Transform) -> Vec2 {
        let local_point = transform.inverse_transform_point(point);
        let closest_local = self.closest_point(local_point);
        transform.transform_point(closest_local)
    }

    fn mass_properties(&self, density: f32) -> MassProperties {
        self.mass_properties(density)
    }

    fn inertia(&self, mass: f32) -> f32 {
        self.inertia(mass)
    }

    fn support(&self, direction: Vec2) -> Vec2 {
        self.support(direction)
    }

    fn is_valid(&self) -> bool {
        self.is_valid()
    }

    fn validate(&self) -> bool {
        self.validate()
    }

    fn area(&self) -> f32 {
        self.area()
    }

    fn clone_box(&self) -> Box<dyn ShapeTrait> {
        Box::new(self.clone())
    }
}

// Implement Clone for Box<dyn ShapeTrait>
impl Clone for Box<dyn ShapeTrait> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shape_enum() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        assert!(circle.is_circle());
        assert!(!circle.is_polygon());
        assert!(!polygon.is_circle());
        assert!(polygon.is_polygon());

        assert_eq!(circle.shape_type(), ShapeType::Circle);
        assert_eq!(polygon.shape_type(), ShapeType::Polygon);
    }

    #[test]
    fn test_shape_as_ref() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        assert!(circle.as_circle().is_some());
        assert!(circle.as_polygon().is_none());
        assert!(polygon.as_circle().is_none());
        assert!(polygon.as_polygon().is_some());
    }

    #[test]
    fn test_shape_aabb() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        let transform = Transform::identity();

        let circle_aabb = circle.aabb(transform);
        assert_eq!(circle_aabb.min, Vec2::new(-5.0, -5.0));
        assert_eq!(circle_aabb.max, Vec2::new(5.0, 5.0));

        let polygon_aabb = polygon.aabb(transform);
        assert_eq!(polygon_aabb.min, Vec2::new(-1.0, -1.0));
        assert_eq!(polygon_aabb.max, Vec2::new(1.0, 1.0));
    }

    #[test]
    fn test_shape_contains() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        let transform = Transform::identity();

        assert!(circle.contains_point(Vec2::new(0.0, 0.0), transform));
        assert!(!circle.contains_point(Vec2::new(10.0, 0.0), transform));

        assert!(polygon.contains_point(Vec2::new(0.0, 0.0), transform));
        assert!(!polygon.contains_point(Vec2::new(10.0, 0.0), transform));
    }

    #[test]
    fn test_shape_mass_properties() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        let density = 1.0;

        let circle_mp = circle.mass_properties(density);
        assert!(circle_mp.mass > 0.0);

        let polygon_mp = polygon.mass_properties(density);
        assert!(polygon_mp.mass > 0.0);
    }

    #[test]
    fn test_shape_validation() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        assert!(circle.is_valid());
        assert!(circle.validate());
        assert!(polygon.is_valid());
        assert!(polygon.validate());
    }

    #[test]
    fn test_shape_support() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        let direction = Vec2::new(1.0, 0.0);

        let circle_support = circle.support(direction);
        assert!(approx_eq(circle_support.length(), 5.0));

        let polygon_support = polygon.support(direction);
        assert_eq!(polygon_support, Vec2::new(1.0, -1.0));
    }

    #[test]
    fn test_shape_area() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        let circle_area = circle.area();
        assert!(approx_eq(circle_area, 78.53981633974483));

        let polygon_area = polygon.area();
        assert!(approx_eq(polygon_area, 4.0));
    }

    #[test]
    fn test_shape_default() {
        let shape = Shape::default();
        assert!(shape.is_circle());
    }

    #[test]
    fn test_shape_type_display() {
        assert_eq!(format!("{}", ShapeType::Circle), "Circle");
        assert_eq!(format!("{}", ShapeType::Polygon), "Polygon");
    }

    #[test]
    fn test_shape_display() {
        let circle = Shape::Circle(Circle::new(5.0));
        let polygon = Shape::Polygon(Polygon::from_rectangle(2.0, 2.0));

        let circle_str = format!("{}", circle);
        assert!(circle_str.contains("Circle"));

        let polygon_str = format!("{}", polygon);
        assert!(polygon_str.contains("Polygon"));
    }

    // Helper function for tests
    #[allow(dead_code)]
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= 1e-6
    }

    fn approx_eq_eps(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() <= epsilon
    }
}
