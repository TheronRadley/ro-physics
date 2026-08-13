//! Collision detection system for the physics engine.
//!
//! This module provides broad-phase and narrow-phase collision detection
//! for rigid bodies.

mod broad_phase;
mod narrow_phase;
mod manifolds;
mod collider;
mod filter;
mod collision_event;

pub use broad_phase::{BroadPhase, BroadPhaseConfig};
pub use narrow_phase::{NarrowPhase, NarrowPhaseConfig};
pub use manifolds::{ContactManifold, Contact, ContactId};
pub use collider::{Collider, ColliderHandle};
pub use filter::Filter;
pub use collision_event::{CollisionEvent, CollisionEventHandler, CollisionEventType, SimpleCollisionEventHandler};

/// Collision utilities
pub mod utils {
    use super::*;
    use crate::math::Vec2;

    /// Check if two colliders can collide based on their filters
    #[inline]
    #[must_use]
    pub fn can_collide(collider1: &Collider, collider2: &Collider) -> bool {
        collider1.filter.can_collide_with(&collider2.filter)
    }

    /// Calculate the relative velocity between two bodies at a contact point
    #[inline]
    #[must_use]
    pub fn relative_velocity(
        velocity1: Vec2,
        angular1: f32,
        position1: Vec2,
        velocity2: Vec2,
        angular2: f32,
        position2: Vec2,
        contact_point: Vec2,
    ) -> Vec2 {
        let r1 = contact_point - position1;
        let r2 = contact_point - position2;
        
        let v1 = velocity1 + Vec2::new(-angular1 * r1.y, angular1 * r1.x);
        let v2 = velocity2 + Vec2::new(-angular2 * r2.y, angular2 * r2.x);
        
        v1 - v2
    }

    /// Calculate the normal relative velocity (scalar)
    #[inline]
    #[must_use]
    pub fn normal_relative_velocity(
        velocity1: Vec2,
        angular1: f32,
        position1: Vec2,
        velocity2: Vec2,
        angular2: f32,
        position2: Vec2,
        contact_point: Vec2,
        normal: Vec2,
    ) -> f32 {
        let rel_vel = Self::relative_velocity(
            velocity1, angular1, position1,
            velocity2, angular2, position2,
            contact_point,
        );
        rel_vel.dot(normal)
    }

    /// Check if two AABBs are separated along an axis
    #[inline]
    #[must_use]
    pub fn aabb_separated(aabb1: &crate::geometry::Aabb, aabb2: &crate::geometry::Aabb, axis: Vec2) -> bool {
        let axis_normalized = axis.normalize();
        
        // Project both AABBs onto the axis
        let min1 = Self::project_aabb(aabb1, axis_normalized).0;
        let max1 = Self::project_aabb(aabb1, axis_normalized).1;
        let min2 = Self::project_aabb(aabb2, axis_normalized).0;
        let max2 = Self::project_aabb(aabb2, axis_normalized).1;
        
        // Check for separation
        max1 < min2 || max2 < min1
    }

    /// Project an AABB onto an axis
    #[inline]
    #[must_use]
    pub fn project_aabb(aabb: &crate::geometry::Aabb, axis: Vec2) -> (f32, f32) {
        // Get the 4 corners of the AABB
        let corners = [
            Vec2::new(aabb.min.x, aabb.min.y),
            Vec2::new(aabb.max.x, aabb.min.y),
            Vec2::new(aabb.max.x, aabb.max.y),
            Vec2::new(aabb.min.x, aabb.max.y),
        ];
        
        // Project each corner and find min/max
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        
        for corner in corners {
            let proj = corner.dot(axis);
            min = min.min(proj);
            max = max.max(proj);
        }
        
        (min, max)
    }
}

pub use utils::*;
