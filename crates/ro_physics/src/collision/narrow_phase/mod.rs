//! Narrow-phase collision detection.
//!
//! This module provides precise collision detection between pairs of shapes
//! identified by the broad-phase.

mod circle_circle;
mod circle_polygon;
mod polygon_polygon;
mod config;

pub use circle_circle::circle_circle_collision;
pub use circle_polygon::circle_polygon_collision;
pub use polygon_polygon::polygon_polygon_collision;
pub use config::NarrowPhaseConfig;

use crate::bodies::BodyHandle;
use crate::collision::{Collider, ColliderHandle};
use crate::math::{Transform, Vec2};
use super::ContactManifold;

/// Narrow-phase collision detection system.
///
/// The narrow-phase performs precise collision detection between pairs
/// of shapes that were identified as potentially overlapping by the broad-phase.
#[derive(Debug)]
pub struct NarrowPhase {
    /// Configuration
    config: NarrowPhaseConfig,
}

impl NarrowPhase {
    /// Create a new narrow-phase detector
    #[inline]
    #[must_use]
    pub fn new(config: NarrowPhaseConfig) -> Self {
        Self { config }
    }

    /// Create a new narrow-phase detector with default configuration
    #[inline]
    #[must_use]
    pub fn default() -> Self {
        Self::new(NarrowPhaseConfig::default())
    }

    /// Get the configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &NarrowPhaseConfig {
        &self.config
    }

    /// Detect collision between two colliders
    ///
    /// Returns a contact manifold if the colliders are overlapping, None otherwise.
    #[inline]
    #[must_use]
    pub fn collide(
        &self,
        collider_a: &Collider,
        transform_a: Transform,
        collider_b: &Collider,
        transform_b: Transform,
    ) -> Option<ContactManifold> {
        let shape_a = collider_a.shape()?;
        let shape_b = collider_b.shape()?;

        match (shape_a, shape_b) {
            (crate::shapes::Shape::Circle(circle_a), crate::shapes::Shape::Circle(circle_b)) => {
                circle_circle_collision(
                    circle_a,
                    transform_a,
                    circle_b,
                    transform_b,
                    &self.config,
                )
            }
            (crate::shapes::Shape::Circle(circle_a), crate::shapes::Shape::Polygon(polygon_b)) => {
                circle_polygon_collision(
                    circle_a,
                    transform_a,
                    polygon_b,
                    transform_b,
                    &self.config,
                )
            }
            (crate::shapes::Shape::Polygon(polygon_a), crate::shapes::Shape::Circle(circle_b)) => {
                circle_polygon_collision(
                    circle_b,
                    transform_b,
                    polygon_a,
                    transform_a,
                    &self.config,
                ).map(|mut manifold| {
                    manifold.flip();
                    manifold
                })
            }
            (crate::shapes::Shape::Polygon(polygon_a), crate::shapes::Shape::Polygon(polygon_b)) => {
                polygon_polygon_collision(
                    polygon_a,
                    transform_a,
                    polygon_b,
                    transform_b,
                    &self.config,
                )
            }
            _ => None,
        }
    }

    /// Detect collision between two shapes directly
    #[inline]
    #[must_use]
    pub fn collide_shapes(
        &self,
        shape_a: &crate::shapes::Shape,
        transform_a: Transform,
        shape_b: &crate::shapes::Shape,
        transform_b: Transform,
    ) -> Option<ContactManifold> {
        match (shape_a, shape_b) {
            (crate::shapes::Shape::Circle(circle_a), crate::shapes::Shape::Circle(circle_b)) => {
                circle_circle_collision(
                    circle_a,
                    transform_a,
                    circle_b,
                    transform_b,
                    &self.config,
                )
            }
            (crate::shapes::Shape::Circle(circle_a), crate::shapes::Shape::Polygon(polygon_b)) => {
                circle_polygon_collision(
                    circle_a,
                    transform_a,
                    polygon_b,
                    transform_b,
                    &self.config,
                )
            }
            (crate::shapes::Shape::Polygon(polygon_a), crate::shapes::Shape::Circle(circle_b)) => {
                circle_polygon_collision(
                    circle_b,
                    transform_b,
                    polygon_a,
                    transform_a,
                    &self.config,
                ).map(|mut manifold| {
                    manifold.flip();
                    manifold
                })
            }
            (crate::shapes::Shape::Polygon(polygon_a), crate::shapes::Shape::Polygon(polygon_b)) => {
                polygon_polygon_collision(
                    polygon_a,
                    transform_a,
                    polygon_b,
                    transform_b,
                    &self.config,
                )
            }
            _ => None,
        }
    }
}

impl Default for NarrowPhase {
    fn default() -> Self {
        Self::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::{Circle, Polygon};
    use crate::math::Rot2;

    #[test]
    fn test_circle_circle_collision() {
        let narrow = NarrowPhase::default();

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(1.5, 0.0));

        let shape_a = crate::shapes::Shape::Circle(circle_a);
        let shape_b = crate::shapes::Shape::Circle(circle_b);

        let result = narrow.collide_shapes(&shape_a, transform_a, &shape_b, transform_b);

        assert!(result.is_some());
        let manifold = result.unwrap();
        assert_eq!(manifold.contact_count(), 1);
    }

    #[test]
    fn test_circle_circle_no_collision() {
        let narrow = NarrowPhase::default();

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(3.0, 0.0));

        let shape_a = crate::shapes::Shape::Circle(circle_a);
        let shape_b = crate::shapes::Shape::Circle(circle_b);

        let result = narrow.collide_shapes(&shape_a, transform_a, &shape_b, transform_b);

        assert!(result.is_none());
    }

    #[test]
    fn test_circle_polygon_collision() {
        let narrow = NarrowPhase::default();

        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(1.0, 0.0));

        let shape_a = crate::shapes::Shape::Circle(circle);
        let shape_b = crate::shapes::Shape::Polygon(polygon);

        let result = narrow.collide_shapes(&shape_a, transform_a, &shape_b, transform_b);

        assert!(result.is_some());
    }

    #[test]
    fn test_polygon_polygon_collision() {
        let narrow = NarrowPhase::default();

        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(1.5, 0.0));

        let shape_a = crate::shapes::Shape::Polygon(polygon_a);
        let shape_b = crate::shapes::Shape::Polygon(polygon_b);

        let result = narrow.collide_shapes(&shape_a, transform_a, &shape_b, transform_b);

        assert!(result.is_some());
    }

    #[test]
    fn test_polygon_polygon_no_collision() {
        let narrow = NarrowPhase::default();

        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(5.0, 0.0));

        let shape_a = crate::shapes::Shape::Polygon(polygon_a);
        let shape_b = crate::shapes::Shape::Polygon(polygon_b);

        let result = narrow.collide_shapes(&shape_a, transform_a, &shape_b, transform_b);

        assert!(result.is_none());
    }
}
