//! Constraint solver for the physics engine.
//!
//! This module provides the constraint solver that resolves
//! collisions and constraints between bodies.

mod config;
mod sequential_impulse;
mod contact_solver;

pub use config::SolverConfig;
pub use sequential_impulse::SequentialImpulseSolver;
pub use contact_solver::ContactSolver;

/// Solver utilities
pub mod utils {
    use crate::math::Vec2;

    /// Calculate the impulse to apply to two bodies
    #[inline]
    #[must_use]
    pub fn calculate_impulse(
        normal: Vec2,
        relative_velocity: f32,
        inv_mass_a: f32,
        inv_mass_b: f32,
        inv_inertia_a: f32,
        inv_inertia_b: f32,
        contact_point_a: Vec2,
        contact_point_b: Vec2,
        restitution: f32,
    ) -> Vec2 {
        // Calculate the effective mass
        let r_a = contact_point_a;
        let r_b = contact_point_b;
        
        let effective_mass = inv_mass_a + inv_mass_b
            + r_a.cross(normal).powi(2) * inv_inertia_a
            + r_b.cross(normal).powi(2) * inv_inertia_b;

        if effective_mass <= 0.0 {
            return Vec2::zero();
        }

        // Calculate the impulse magnitude
        let impulse_magnitude = -(1.0 + restitution) * relative_velocity / effective_mass;
        
        // Return the impulse vector
        normal * impulse_magnitude
    }

    /// Calculate the friction impulse
    #[inline]
    #[must_use]
    pub fn calculate_friction_impulse(
        tangent: Vec2,
        relative_velocity: f32,
        inv_mass_a: f32,
        inv_mass_b: f32,
        inv_inertia_a: f32,
        inv_inertia_b: f32,
        contact_point_a: Vec2,
        contact_point_b: Vec2,
        friction: f32,
        normal_impulse: f32,
    ) -> Vec2 {
        // Calculate the effective mass for friction
        let r_a = contact_point_a;
        let r_b = contact_point_b;
        
        let effective_mass = inv_mass_a + inv_mass_b
            + r_a.cross(tangent).powi(2) * inv_inertia_a
            + r_b.cross(tangent).powi(2) * inv_inertia_b;

        if effective_mass <= 0.0 {
            return Vec2::zero();
        }

        // Calculate the maximum friction impulse (Coulomb friction)
        let max_friction = friction * normal_impulse.abs();
        
        // Calculate the friction impulse magnitude
        let impulse_magnitude = -relative_velocity / effective_mass;
        
        // Clamp to maximum friction
        let impulse_magnitude = impulse_magnitude.clamp(-max_friction, max_friction);
        
        // Return the friction impulse vector
        tangent * impulse_magnitude
    }
}

pub use utils::*;
