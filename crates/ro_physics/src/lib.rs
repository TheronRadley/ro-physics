//! # Ro-Physics
//!
//! A 2D rigid-body physics engine implemented in Rust.
//!
//! ## Features
//!
//! - **Rigid body dynamics**: dynamic, static, and kinematic bodies
//! - **Collision detection**: broad-phase (dynamic AABB tree) and narrow-phase
//!   (circle/circle, circle/polygon, polygon/polygon)
//! - **Constraint solver**: sequential impulse solver with warm starting,
//!   friction, restitution and Baumgarte position correction
//! - **Sleeping system**: time-threshold based deactivation of resting bodies
//! - **Continuous collision detection**: conservative advancement to keep fast
//!   moving bodies from tunnelling
//! - **Deterministic simulation**: fixed iteration order, no global state
//!
//! ## Conventions
//!
//! Understanding these conventions is required to use the engine correctly.
//!
//! ### Units and coordinates
//!
//! - Positions, velocities, forces and sizes are in **metres**, **metres per
//!   second**, **newtons** and **metres**. There is no implicit unit scale.
//! - Masses are in **kilograms**, inertia in **kg·m²**, angles in **radians**
//!   and angular velocities in **radians per second**.
//! - The world is right-handed with **+X to the right and +Y up**. The default
//!   gravity is `(0, -9.81)`, i.e. "down" is `-Y`.
//! - Rotations are counter-clockwise positive. A body's transform maps
//!   **local space to world space**; [`math::Transform::transform_point`]
//!   applies rotation first, then translation.
//!
//! ### Shapes and polygons
//!
//! - Shape vertices are stored relative to the shape's local origin.
//! - Polygon vertices are **normalised to counter-clockwise winding** on
//!   construction and their edge normals point **outwards**.
//! - Bodies rotate about their centre of mass; colliders may carry an extra
//!   local transform relative to their body.
//!
//! ### Contacts
//!
//! - A [`collision::ContactManifold`] is generated for **collider A to
//!   collider B** and its normal points **from A to B**.
//! - [`collision::Contact::penetration`] is **positive while overlapping** and
//!   negative (separation) when the shapes are apart.
//! - Impulses are applied as `-normal * impulse` on body A and
//!   `+normal * impulse` on body B, so a resting contact pushes A along
//!   `-normal`, i.e. out of B.
//!
//! ### Tolerance policy
//!
//! - [`constants::EPSILON`] (`1e-6`) is the default tolerance for
//!   floating-point comparisons in the math types.
//! - [`constants::COLLISION_EPSILON`] (`1e-4`) is used for collision
//!   detection; the solver additionally uses a linear slop so that resting
//!   contacts stay slightly overlapped instead of jittering.
//! - Public constructors **reject** non-finite values, negative radii and
//!   degenerate polygons by returning a [`Result`] (see
//!   [`shapes::Polygon::try_new`]). `Circle::new` and `Polygon::new` panic on
//!   invalid input and are meant for already-validated data.
//!
//! ### Ownership
//!
//! - The [`world::World`] owns all bodies, colliders and constraints. User
//!   code refers to them through lightweight, copyable handles
//!   ([`bodies::BodyHandle`], [`collision::ColliderHandle`],
//!   [`constraints::ConstraintHandle`]).
//! - Handles carry a generation counter: using a handle after the object it
//!   refers to has been destroyed always fails (it never resolves to a
//!   different object).
//! - Default-constructed handles are "null" handles and never refer to a live
//!   object.
//!
//! ## Architecture
//!
//! - `math`: vector, rotation, and transform operations
//! - `geometry`: AABB and ray primitives
//! - `shapes`: circle and polygon shapes with mass properties
//! - `bodies`: rigid body state and properties
//! - `collision`: broad-phase, narrow-phase, manifolds and filters
//! - `constraints`: joint implementations
//! - `solver`: contact solving
//! - `world`: simulation world management
//! - `integration`: velocity/position integration
//! - `sleeping`: body deactivation
//! - `query`: spatial queries
//! - `debug`: debug drawing helpers
//!
//! ## Quick Start
//!
//! ```
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
//! let ball = world.create_dynamic_body(Transform::from_translation(Vec2::new(0.0, 5.0)));
//! world.add_circle(ball, 0.5, 1.0);
//!
//! // Step the simulation
//! world.step(1.0 / 60.0);
//!
//! // The ball started falling
//! assert!(world.get_body(ball).unwrap().position().y < 5.0);
//! ```

#![warn(missing_docs)]
#![warn(clippy::all)]
// `clippy::pedantic` is deliberately not enabled as a group: under
// `-D warnings` every style nit would become a build failure, and the group is
// only worth adopting once the crate can be triaged lint by lint. The
// individual lints that already proved useful are enabled explicitly below.
#![warn(clippy::default_trait_access)]
#![warn(clippy::explicit_into_iter_loop)]
#![warn(clippy::redundant_closure_for_method_calls)]
#![warn(clippy::ref_option_ref)]
#![warn(clippy::unnecessary_wraps)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::similar_names)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::too_many_arguments)]
#![allow(clippy::many_single_char_names)]

pub mod math;
pub mod geometry;
pub mod shapes;
pub mod bodies;
pub mod collision;
pub mod constraints;
pub mod ccd;
pub mod solver;
pub mod world;
pub mod integration;
pub mod sleeping;
pub mod query;
pub mod debug;

#[cfg(feature = "serialization")]
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
    pub use crate::constraints::{
        Constraint, ConstraintHandle, DistanceJoint, MotorJoint, RevoluteJoint,
    };
    pub use crate::ccd::{CcdConfig, CcdResult, SweptCollisionDetector, TimeOfImpact, ToiResult};
    pub use crate::world::{World, WorldConfig};
    pub use crate::query::{
        AabbQuery, AabbQueryResult, Bvh, PointQuery, PointQueryResult, RayCast, RayCastOptions,
        RayCastResult,
    };
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
    /// Largest timestep the simulation accepts (seconds)
    pub const MAX_TIMESTEP: f32 = 1.0 / 15.0;
    /// Smallest timestep the simulation accepts (seconds)
    pub const MIN_TIMESTEP: f32 = 1e-5;
    /// Contact slop: overlaps smaller than this are not corrected
    pub const LINEAR_SLOP: f32 = 0.005;
}
