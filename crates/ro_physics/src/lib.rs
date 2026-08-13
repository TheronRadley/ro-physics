//! # Ro-Physics
//!
//! A production-grade 2D rigid-body physics engine implemented in Rust.
//!
//! ## Features
//!
//! - **Rigid Body Dynamics**: Dynamic, static, and kinematic bodies
//! - **Collision Detection**: Broad-phase (AABB tree) and narrow-phase (SAT, GJK)
//! - **Constraint Solver**: Sequential impulse solver with contact, distance, and revolute constraints
//! - **Sleeping System**: Automatic body sleeping for performance
//! - **Continuous Collision Detection**: Tunneling prevention for fast-moving objects
//! - **Deterministic Simulation**: Consistent results across runs
//!
//! ## Architecture
//!
//! The engine is organized into modular components:
//!
//! - `math`: Vector, rotation, and transform operations
//! - `geometry`: AABB, ray, and spatial primitives
//! - `shapes`: Circle, polygon, and compound shape definitions
//! - `bodies`: Rigid body state and properties
//! - `collision`: Broad-phase and narrow-phase detection
//! - `dynamics`: Force, torque, and integration
//! - `constraints`: Joint and constraint implementations
//! - `solver`: Contact and constraint solving
//! - `world`: Simulation world management
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use ro_physics::prelude::*;
//!
//! // Create a simulation world
//! let mut world = World::with_gravity(Vec2::new(0.0, -9.81));
//!
//! // Create a static ground body
//! let ground = world.create_static_body(Transform::identity());
//! world.add_box(ground, Vec2::new(10.0, 0.5), 0.0);
//!
//! // Create a dynamic circle
//! let ball = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), 0.0));
//! world.add_circle(ball, 0.5, 1.0);
//!
//! // Step the simulation
//! world.step(1.0 / 60.0);
//! ```

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::similar_names)]
#![allow(clippy::module_name_repetitions)]

pub mod math;
pub mod geometry;
pub mod shapes;
pub mod bodies;
pub mod collision;
pub mod dynamics;
pub mod constraints;
pub mod ccd;
pub mod solver;
pub mod world;
pub mod integration;
pub mod events;
pub mod sleeping;
pub mod query;
pub mod debug;
pub mod serialization;

/// Re-exports for convenient access to commonly used types
pub mod prelude {
    pub use crate::math::{Rot2, Transform, Vec2};
    pub use crate::geometry::{Aabb, Ray};
    pub use crate::shapes::{Circle, Polygon, Shape};
    pub use crate::bodies::{Body, BodyHandle, BodyType};
    pub use crate::collision::{
        Collider, ColliderHandle, CollisionEvent, ContactManifold, Filter,
    };
    pub use crate::constraints::{Constraint, ConstraintHandle, DistanceJoint, RevoluteJoint, MotorJoint};
    pub use crate::ccd::{CcdConfig, CcdResult, SweptCollisionDetector, TimeOfImpact, ToiResult};
    pub use crate::world::{World, WorldConfig};
    pub use crate::query::{PointQuery, PointQueryResult, AabbQuery, AabbQueryResult, RayCast, RayCastResult, RayCastOptions, Bvh};
    pub use crate::sleeping::SleepingConfig;
    pub use crate::solver::SolverConfig;
}

/// Numerical constants used throughout the engine
pub mod constants {
    /// PI constant
    pub const PI: f32 = std::f32::consts::PI;
    /// 2 * PI
    pub const TWO_PI: f32 = 2.0 * PI;
    /// PI / 2
    pub const PI_OVER_TWO: f32 = PI / 2.0;
    /// PI / 4
    pub const PI_OVER_FOUR: f32 = PI / 4.0;
    /// Epsilon for floating-point comparisons
    pub const EPSILON: f32 = 1e-6;
    /// Epsilon for collision detection
    pub const COLLISION_EPSILON: f32 = 1e-4;
    /// Small value to prevent division by zero
    pub const MIN_POSITIVE: f32 = 1e-10;
    /// Maximum float value for validation
    pub const MAX_FLOAT: f32 = 1e20;
    /// Minimum float value for validation
    pub const MIN_FLOAT: f32 = -1e20;
}

/// Engine configuration with sensible defaults
pub struct EngineConfig {
    /// Maximum number of velocity iterations in the solver
    pub velocity_iterations: usize,
    /// Maximum number of position iterations in the solver
    pub position_iterations: usize,
    /// Gravity vector
    pub gravity: Vec2,
    /// Sleeping configuration
    pub sleeping_config: SleepingConfig,
    /// Solver configuration
    pub solver_config: SolverConfig,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 3,
            gravity: Vec2::new(0.0, -9.81),
            sleeping_config: SleepingConfig::default(),
            solver_config: SolverConfig::default(),
        }
    }
}

impl EngineConfig {
    /// Create a new engine configuration
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the gravity vector
    #[must_use]
    pub fn with_gravity(mut self, gravity: Vec2) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set the number of velocity iterations
    #[must_use]
    pub fn with_velocity_iterations(mut self, iterations: usize) -> Self {
        self.velocity_iterations = iterations;
        self
    }

    /// Set the number of position iterations
    #[must_use]
    pub fn with_position_iterations(mut self, iterations: usize) -> Self {
        self.position_iterations = iterations;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_engine_config_default() {
        let config = EngineConfig::default();
        assert_eq!(config.velocity_iterations, 8);
        assert_eq!(config.position_iterations, 3);
        assert_eq!(config.gravity, Vec2::new(0.0, -9.81));
    }

    #[test]
    fn test_engine_config_builder() {
        let config = EngineConfig::new()
            .with_gravity(Vec2::new(0.0, -10.0))
            .with_velocity_iterations(10)
            .with_position_iterations(5);

        assert_eq!(config.gravity, Vec2::new(0.0, -10.0));
        assert_eq!(config.velocity_iterations, 10);
        assert_eq!(config.position_iterations, 5);
    }
}
