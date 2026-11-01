//! World management for the physics engine.
//!
//! This module provides the World type which is the main entry point
//! for the physics simulation.

mod config;
mod world;
mod tests;

pub use config::WorldConfig;
pub use world::World;

/// World utilities
pub mod utils {
    use super::{World, WorldConfig};
    use crate::math::Vec2;

    /// Create a simple world with default settings
    #[inline]
    #[must_use]
    pub fn create_simple_world() -> World {
        World::new(WorldConfig::default())
    }

    /// Create a world with custom gravity
    #[inline]
    #[must_use]
    pub fn create_world_with_gravity(gravity: crate::math::Vec2) -> World {
        World::new(WorldConfig::default().with_gravity(gravity))
    }

    /// Create a world without gravity
    #[inline]
    #[must_use]
    pub fn create_zero_gravity_world() -> World {
        World::new(WorldConfig::default().with_gravity(crate::math::Vec2::zero()))
    }
}

pub use utils::*;
