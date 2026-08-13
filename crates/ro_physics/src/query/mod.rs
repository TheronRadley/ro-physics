//! Spatial query module for the physics engine.
//!
//! This module provides spatial query operations for the physics world.

mod point_query;
mod aabb_query;
mod ray_cast;
mod bvh;
mod tests;

pub use point_query::{PointQuery, PointQueryResult};
pub use aabb_query::{AabbQuery, AabbQueryResult};
pub use ray_cast::{RayCast, RayCastResult};
pub use bvh::Bvh;

use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;
use crate::geometry::{Aabb, Ray};
use crate::math::Vec2;

/// Trait for performing spatial queries
pub trait SpatialQuery: Send + Sync {
    /// Query for bodies/colliders that contain a point
    fn point_query(&self, point: Vec2) -> Vec<PointQueryResult>;

    /// Query for bodies/colliders that overlap an AABB
    fn aabb_query(&self, aabb: Aabb) -> Vec<AabbQueryResult>;

    /// Cast a ray and return the first hit
    fn ray_cast(&self, ray: Ray, max_distance: f32) -> Option<RayCastResult>;

    /// Cast a ray and return all hits
    fn ray_cast_all(&self, ray: Ray, max_distance: f32) -> Vec<RayCastResult>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::collision::ColliderHandle;

    #[test]
    fn test_point_query_result() {
        let result = PointQueryResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
            true,
            0.0,
        );

        assert_eq!(result.body_handle, BodyHandle::new(1, 0));
        assert_eq!(result.collider_handle, ColliderHandle::new(1, 0));
        assert!(result.is_inside);
    }

    #[test]
    fn test_aabb_query_result() {
        let result = AabbQueryResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
        );

        assert_eq!(result.body_handle, BodyHandle::new(1, 0));
        assert_eq!(result.collider_handle, ColliderHandle::new(1, 0));
    }

    #[test]
    fn test_ray_cast_result() {
        let result = RayCastResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            5.0,
            0.5,
        );

        assert_eq!(result.body_handle, BodyHandle::new(1, 0));
        assert_eq!(result.hit_point, Vec2::new(1.0, 2.0));
        assert_eq!(result.distance, 5.0);
        assert_eq!(result.fraction, 0.5);
    }
}
