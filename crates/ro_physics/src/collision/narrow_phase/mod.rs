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

use crate::collision::{Collider, ContactManifold};
use crate::math::Transform;

/// Narrow-phase collision detection system.
///
/// The narrow-phase performs precise collision detection between pairs
/// of shapes that were identified as potentially overlapping by the broad-phase.
///
/// Every manifold it produces is expressed for the pair (A, B): the normal
/// points from A to B and the penetration is positive while the shapes
/// overlap.
#[derive(Debug, Clone)]
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

    /// Get the configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &NarrowPhaseConfig {
        &self.config
    }

    /// Detect collision between two colliders
    ///
    /// The returned manifold carries the body and collider handles of the pair
    /// as well as the combined friction and restitution of the two colliders.
    /// Friction is combined as `sqrt(f_a * f_b)` and restitution as
    /// `max(e_a, e_b)`, matching the usual mixing rules.
    ///
    /// Returns `None` when the shapes do not overlap.
    #[inline]
    #[must_use]
    pub fn collide(
        &self,
        collider_a: &Collider,
        transform_a: Transform,
        collider_b: &Collider,
        transform_b: Transform,
    ) -> Option<ContactManifold> {
        let mut manifold =
            self.collide_shapes(collider_a.shape(), transform_a, collider_b.shape(), transform_b)?;

        manifold.set_pair(
            collider_a.body_handle(),
            collider_b.body_handle(),
            collider_a.handle(),
            collider_b.handle(),
        );
        manifold.set_friction((collider_a.friction() * collider_b.friction()).sqrt());
        manifold.set_restitution(collider_a.restitution().max(collider_b.restitution()));
        manifold.set_sensor(collider_a.is_sensor() || collider_b.is_sensor());

        Some(manifold)
    }

    /// Detect collision between two shapes directly
    ///
    /// The resulting manifold has no handles attached and its friction and
    /// restitution are left at zero; use [`NarrowPhase::collide`] when the
    /// colliders are available.
    #[inline]
    #[must_use]
    pub fn collide_shapes(
        &self,
        shape_a: &crate::shapes::Shape,
        transform_a: Transform,
        shape_b: &crate::shapes::Shape,
        transform_b: Transform,
    ) -> Option<ContactManifold> {
        use crate::shapes::Shape::{Circle, Polygon};

        match (shape_a, shape_b) {
            (Circle(circle_a), Circle(circle_b)) => circle_circle_collision(
                circle_a,
                transform_a,
                circle_b,
                transform_b,
                &self.config,
            ),
            (Circle(circle_a), Polygon(polygon_b)) => circle_polygon_collision(
                circle_a,
                transform_a,
                polygon_b,
                transform_b,
                &self.config,
            ),
            (Polygon(polygon_a), Circle(circle_b)) => circle_polygon_collision(
                circle_b,
                transform_b,
                polygon_a,
                transform_a,
                &self.config,
            )
            .map(|mut manifold| {
                // The shapes were swapped to keep (circle, polygon) ordering,
                // so the manifold has to be flipped back to (A, B).
                manifold.flip();
                manifold
            }),
            (Polygon(polygon_a), Polygon(polygon_b)) => polygon_polygon_collision(
                polygon_a,
                transform_a,
                polygon_b,
                transform_b,
                &self.config,
            ),
        }
    }
}

impl Default for NarrowPhase {
    fn default() -> Self {
        Self::new(NarrowPhaseConfig::default())
    }
}
