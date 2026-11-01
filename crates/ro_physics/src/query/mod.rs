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
pub use ray_cast::{RayCast, RayCastOptions, RayCastResult};
pub use bvh::Bvh;

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

