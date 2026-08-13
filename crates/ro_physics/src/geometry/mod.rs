//! Geometric primitives for the physics engine.
//!
//! This module provides fundamental geometric types used in collision
//! detection and spatial queries.

mod aabb;
mod ray;

pub use aabb::Aabb;
pub use ray::Ray;

/// Geometric utilities
pub mod utils {
    use super::Aabb;
    use crate::math::Vec2;

    /// Calculate the minimum AABB that contains both AABBs
    #[inline]
    #[must_use]
    pub fn aabb_union(a: Aabb, b: Aabb) -> Aabb {
        Aabb::new(
            Vec2::new(a.min.x.min(b.min.x), a.min.y.min(b.min.y)),
            Vec2::new(a.max.x.max(b.max.x), a.max.y.max(b.max.y)),
        )
    }

    /// Calculate the intersection of two AABBs
    ///
    /// Returns None if the AABBs don't overlap.
    #[inline]
    #[must_use]
    pub fn aabb_intersection(a: Aabb, b: Aabb) -> Option<Aabb> {
        let min_x = a.min.x.max(b.min.x);
        let min_y = a.min.y.max(b.min.y);
        let max_x = a.max.x.min(b.max.x);
        let max_y = a.max.y.min(b.max.y);

        if min_x > max_x || min_y > max_y {
            None
        } else {
            Some(Aabb::new(
                Vec2::new(min_x, min_y),
                Vec2::new(max_x, max_y),
            ))
        }
    }

    /// Check if two AABBs overlap
    #[inline]
    #[must_use]
    pub fn aabb_overlaps(a: Aabb, b: Aabb) -> bool {
        a.min.x <= b.max.x
            && a.max.x >= b.min.x
            && a.min.y <= b.max.y
            && a.max.y >= b.min.y
    }

    /// Calculate the distance between two AABBs
    #[inline]
    #[must_use]
    pub fn aabb_distance(a: Aabb, b: Aabb) -> f32 {
        if aabb_overlaps(a, b) {
            0.0
        } else {
            let dx = if a.max.x < b.min.x {
                b.min.x - a.max.x
            } else if a.min.x > b.max.x {
                a.min.x - b.max.x
            } else {
                0.0
            };

            let dy = if a.max.y < b.min.y {
                b.min.y - a.max.y
            } else if a.min.y > b.max.y {
                a.min.y - b.max.y
            } else {
                0.0
            };

            (dx * dx + dy * dy).sqrt()
        }
    }

    /// Calculate the closest point on an AABB to a given point
    #[inline]
    #[must_use]
    pub fn closest_point_on_aabb(point: Vec2, aabb: Aabb) -> Vec2 {
        Vec2::new(
            point.x.clamp(aabb.min.x, aabb.max.x),
            point.y.clamp(aabb.min.y, aabb.max.y),
        )
    }

    /// Check if a point is inside an AABB
    #[inline]
    #[must_use]
    pub fn point_in_aabb(point: Vec2, aabb: Aabb) -> bool {
        point.x >= aabb.min.x
            && point.x <= aabb.max.x
            && point.y >= aabb.min.y
            && point.y <= aabb.max.y
    }
}

pub use utils::*;
