//! Contact solver for resolving individual contact constraints.
//!
//! This module provides utilities for solving contact constraints
//! between pairs of bodies.

use std::collections::HashMap;

use crate::bodies::{Body, BodyHandle};
use crate::collision::ContactManifold;
use crate::math::Vec2;
use super::{SequentialImpulseSolver, SolverConfig};

/// Contact solver that resolves collisions between bodies.
///
/// This is a thin, allocation-free wrapper around the
/// [`SequentialImpulseSolver`] that world implementations can call directly.
///
/// Bodies are addressed by [`BodyHandle`]; `body_indices` maps a handle to its
/// index in the body storage. Manifolds are solved in place and keep their
/// accumulated impulses, which the world can reuse for warm starting on the
/// next step.
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
            solver: SequentialImpulseSolver::new(config),
            config,
        }
    }

    /// Get the solver configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &SolverConfig {
        &self.config
    }

    /// Get the underlying sequential impulse solver
    #[inline]
    #[must_use]
    pub const fn sequential_solver(&self) -> &SequentialImpulseSolver {
        &self.solver
    }

    /// Solve a set of contact manifolds
    ///
    /// # Panics
    ///
    /// Never panics: invalid timesteps, missing bodies and sensor manifolds are
    /// all ignored.
    #[inline]
    pub fn solve(
        &self,
        bodies: &mut [Option<Body>],
        manifolds: &mut [ContactManifold],
        dt: f32,
        body_indices: &HashMap<BodyHandle, usize>,
    ) {
        self.solver.solve_contacts(bodies, manifolds, dt, body_indices);
    }
}

impl Default for ContactSolver {
    fn default() -> Self {
        Self::new(SolverConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;

    #[test]
    fn test_construction() {
        let solver = ContactSolver::new(SolverConfig::default());
        assert_eq!(solver.config().velocity_iterations, 8);
    }







    #[test]
    fn test_solve_updates_bodies_in_place() {
        use crate::collision::{ColliderHandle, ContactId};

        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_mass_properties(1.0, 1.0);
        body_a.set_position(Vec2::new(0.0, 1.0));
        body_a.set_linear_velocity(Vec2::new(0.0, -3.0));

        let mut body_b = Body::new_static(handle_b);
        body_b.set_position(Vec2::zero());

        let mut manifold = ContactManifold::between(
            handle_a,
            handle_b,
            ColliderHandle::null(),
            ColliderHandle::null(),
        );
        manifold.add_contact(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.0,
            0.0,
            0.0,
            ContactId::new(0, 0),
        );
        manifold.set_friction(0.5);
        manifold.set_restitution(0.0);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        ContactSolver::default().solve(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        // The dynamic body must have been stopped by the contact
        let a = bodies[0].as_ref().expect("body a");
        assert!(a.linear_velocity().y >= -1e-3);
        // ... and the accumulated impulse is stored on the manifold
        assert!(manifold.contact(0).normal_impulse() > 0.0);
    }
}
