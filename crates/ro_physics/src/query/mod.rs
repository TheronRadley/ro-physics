//! Spatial query module for the physics engine.
//!
//! This module provides spatial query operations for the physics world.

mod aabb_query;
mod bvh;
mod point_query;
mod ray_cast;
#[cfg(test)]
mod tests;

pub use aabb_query::{AabbQuery, AabbQueryResult};
pub use bvh::Bvh;
pub use point_query::{PointQuery, PointQueryResult};
pub use ray_cast::{RayCast, RayCastOptions, RayCastResult};

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
