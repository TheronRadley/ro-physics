//! Contact solver for resolving individual contact constraints.
//!
//! This module provides utilities for solving contact constraints
//! between pairs of bodies.

use crate::bodies::Body;
use crate::collision::{ContactManifold, ColliderHandle};
use crate::math::Vec2;
use super::{SequentialImpulseSolver, SolverConfig};

/// Contact solver that resolves collisions between bodies.
///
/// This is a higher-level wrapper around the sequential impulse solver
/// that manages contact constraints.
#[derive(Debug, Clone)]
pub struct ContactSolver {
    /// The underlying sequential impulse solver
    solver: SequentialImpulseSolver,
    /// Configuration
    config: SolverConfig,
}

impl ContactSolver {
    /// Create a new contact solver
    #[inline]
    #[must_use]
    pub fn new(config: SolverConfig) -> Self {
        Self {
            solver: SequentialImpulseSolver::new(config.clone()),
            config,
        }
    }

    /// Get the solver configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &SolverConfig {
        &self.config
    }

    /// Solve a set of contact manifolds
    #[inline]
    pub fn solve(
        &self,
        bodies: &mut [Option<Body>],
        manifolds: &mut [ContactManifold],
        dt: f32,
    ) {
        self.solver.solve_contacts(bodies, manifolds, dt);
    }

    /// Solve a single contact manifold
    #[inline]
    pub fn solve_manifold(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        manifold: &mut ContactManifold,
        dt: f32,
    ) {
        // For now, we need to convert to the solver's expected format
        // This is a temporary implementation until we refactor
        
        let mut bodies = vec![Some(body_a.clone()), Some(body_b.clone())];
        let mut manifolds = vec![manifold.clone()];
        
        self.solver.solve_contacts(&mut bodies, &mut manifolds, dt);
        
        // Copy back the results
        if let Some(b) = bodies[0].as_ref() {
            *body_a = b.clone();
        }
        if let Some(b) = bodies[1].as_ref() {
            *body_b = b.clone();
        }
        *manifold = manifolds[0].clone();
    }
}

impl Default for ContactSolver {
    fn default() -> Self {
        Self::new(SolverConfig::default())
    }
}

/// Utilities for solving contacts
pub mod utils {
    use super::*;

    /// Calculate the impulse needed to resolve a contact
    #[inline]
    #[must_use]
    pub fn calculate_contact_impulse(
        body_a: &Body,
        body_b: &Body,
        contact_point: Vec2,
        normal: Vec2,
        normal_velocity: f32,
        restitution: f32,
    ) -> f32 {
        let ra = contact_point - body_a.position();
        let rb = contact_point - body_b.position();
        
        let effective_mass = calculate_effective_mass(
            body_a, body_b,
            normal,
            ra,
            rb,
        );

        if effective_mass <= EPSILON {
            return 0.0;
        }

        // Only apply restitution if velocity exceeds threshold
        let e = if normal_velocity < -0.1 {
            restitution
        } else {
            0.0
        };

        // Normal impulse = -(1 + e) * v_n / m_eff
        -(1.0 + e) * normal_velocity / effective_mass
    }

    /// Calculate effective mass for a contact
    #[inline]
    #[must_use]
    pub fn calculate_effective_mass(
        body_a: &Body,
        body_b: &Body,
        normal: Vec2,
        ra: Vec2,
        rb: Vec2,
    ) -> f32 {
        let inv_mass_a = body_a.inv_mass();
        let inv_mass_b = body_b.inv_mass();
        let inv_inertia_a = body_a.inv_inertia();
        let inv_inertia_b = body_b.inv_inertia();

        let ra_cross_normal = ra.cross(normal);
        let rb_cross_normal = rb.cross(normal);

        let denominator = inv_mass_a + inv_mass_b
            + ra_cross_normal * ra_cross_normal * inv_inertia_a
            + rb_cross_normal * rb_cross_normal * inv_inertia_b;

        if denominator <= crate::constants::EPSILON {
            0.0
        } else {
            1.0 / denominator
        }
    }

    /// Calculate relative velocity at a contact point
    #[inline]
    #[must_use]
    pub fn calculate_relative_velocity(
        body_a: &Body,
        body_b: &Body,
        contact_point: Vec2,
    ) -> Vec2 {
        body_a.velocity_at_point(contact_point) - body_b.velocity_at_point(contact_point)
    }

    /// Calculate normal velocity
    #[inline]
    #[must_use]
    pub fn calculate_normal_velocity(
        relative_velocity: Vec2,
        normal: Vec2,
    ) -> f32 {
        relative_velocity.dot(normal)
    }

    /// Apply impulse to two bodies
    #[inline]
    pub fn apply_impulse(
        body_a: &mut Body,
        body_b: &mut Body,
        normal: Vec2,
        impulse: f32,
        contact_point: Vec2,
    ) {
        if !body_a.is_static() {
            body_a.add_impulse_at_point(-normal * impulse, contact_point);
        }
        if !body_b.is_static() {
            body_b.add_impulse_at_point(normal * impulse, contact_point);
        }
    }
}

pub use utils::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_construction() {
        let solver = ContactSolver::new(SolverConfig::default());
        assert_eq!(solver.config().velocity_iterations, 8);
    }

    #[test]
    fn test_effective_mass() {
        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_mass_properties(10.0, 5.0);

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_mass_properties(10.0, 5.0);

        let normal = Vec2::new(1.0, 0.0);
        let ra = Vec2::zero();
        let rb = Vec2::zero();

        let effective_mass = calculate_effective_mass(&body_a, &body_b, normal, ra, rb);
        assert!(approx_eq(effective_mass, 5.0, 1e-5));
    }

    #[test]
    fn test_relative_velocity() {
        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_linear_velocity(Vec2::new(1.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_linear_velocity(Vec2::new(0.0, 0.0));

        let contact_point = Vec2::new(0.0, 0.0);
        let relative_velocity = calculate_relative_velocity(&body_a, &body_b, contact_point);
        
        assert_eq!(relative_velocity, Vec2::new(1.0, 0.0));
    }

    #[test]
    fn test_normal_velocity() {
        let relative_velocity = Vec2::new(3.0, 4.0);
        let normal = Vec2::new(0.0, 1.0);
        
        let normal_velocity = calculate_normal_velocity(relative_velocity, normal);
        assert!(approx_eq(normal_velocity, 4.0));
    }

    #[test]
    fn test_contact_impulse() {
        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_mass_properties(1.0, 0.0);
        body_a.set_linear_velocity(Vec2::new(0.0, -1.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_static(handle_b);

        let contact_point = Vec2::new(0.0, 0.0);
        let normal = Vec2::new(0.0, 1.0);
        let normal_velocity = -1.0; // Body A moving down at 1 m/s
        let restitution = 0.6;

        let impulse = calculate_contact_impulse(
            &body_a, &body_b, contact_point, normal, normal_velocity, restitution
        );

        // effective_mass = 1 / (1 + 0) = 1
        // impulse = -(1 + 0.6) * (-1) / 1 = 1.6
        assert!(approx_eq(impulse, 1.6, 1e-5));
    }
}
