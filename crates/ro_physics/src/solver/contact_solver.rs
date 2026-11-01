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

/// Utilities for solving contacts
pub mod utils {
    use super::*;
    use crate::constants::EPSILON;

    /// Calculate the impulse needed to resolve a contact
    ///
    /// The returned impulse is the one that must be applied along `normal` to
    /// body B (and in the opposite direction to body A) to bring the relative
    /// normal velocity to zero, including restitution.
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

        let effective_mass = calculate_effective_mass(body_a, body_b, normal, ra, rb);

        if effective_mass <= EPSILON {
            return 0.0;
        }

        // Only apply restitution if the approach velocity exceeds the threshold
        let e = if normal_velocity < -0.1 {
            restitution.max(0.0)
        } else {
            0.0
        };

        // Normal impulse = -(1 + e) * v_n / (1 / m_eff)
        // `normal_velocity` is negative while the bodies approach, so the
        // impulse comes out positive.
        -(1.0 + e) * normal_velocity * effective_mass
    }

    /// Calculate effective mass for a contact
    ///
    /// This is `1 / (sum of inverse masses and inverse inertias along the
    /// constraint direction)`; it is zero when neither body can move.
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

        let denominator = inv_mass_a
            + inv_mass_b
            + ra_cross_normal * ra_cross_normal * inv_inertia_a
            + rb_cross_normal * rb_cross_normal * inv_inertia_b;

        if denominator <= EPSILON {
            0.0
        } else {
            1.0 / denominator
        }
    }

    /// Calculate the velocity of body B relative to body A at a point
    #[inline]
    #[must_use]
    pub fn calculate_relative_velocity(body_a: &Body, body_b: &Body, contact_point: Vec2) -> Vec2 {
        body_b.velocity_at_point(contact_point) - body_a.velocity_at_point(contact_point)
    }

    /// Calculate normal velocity
    #[inline]
    #[must_use]
    pub fn calculate_normal_velocity(relative_velocity: Vec2, normal: Vec2) -> f32 {
        relative_velocity.dot(normal)
    }

    /// Apply an impulse to two bodies at a contact point
    #[inline]
    pub fn apply_impulse(
        body_a: &mut Body,
        body_b: &mut Body,
        normal: Vec2,
        impulse: f32,
        contact_point: Vec2,
    ) {
        if body_a.is_dynamic() {
            body_a.add_impulse_at_point(-normal * impulse, contact_point);
        }
        if body_b.is_dynamic() {
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

    fn approx_eq_eps(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() <= epsilon
    }

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
        // 1 / (0.1 + 0.1) = 5
        assert!(approx_eq_eps(effective_mass, 5.0, 1e-5));
    }

    #[test]
    fn test_effective_mass_of_static_body_is_zero() {
        let mut body_a = Body::new_static(BodyHandle::new(1, 0));
        let mut body_b = Body::new_static(BodyHandle::new(2, 0));
        body_a.set_position(Vec2::zero());
        body_b.set_position(Vec2::zero());

        assert_eq!(
            calculate_effective_mass(&body_a, &body_b, Vec2::unit_x(), Vec2::zero(), Vec2::zero()),
            0.0
        );
    }

    #[test]
    fn test_relative_velocity() {
        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_linear_velocity(Vec2::new(1.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_linear_velocity(Vec2::new(3.0, 0.0));

        let contact_point = Vec2::new(0.0, 0.0);
        let relative_velocity = calculate_relative_velocity(&body_a, &body_b, contact_point);

        // B relative to A
        assert_eq!(relative_velocity, Vec2::new(2.0, 0.0));
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
        let body_b = Body::new_static(handle_b);

        let contact_point = Vec2::new(0.0, 0.0);
        let normal = Vec2::new(0.0, 1.0);
        let normal_velocity = -1.0; // Bodies approaching at 1 m/s
        let restitution = 0.6;

        let impulse = calculate_contact_impulse(
            &body_a,
            &body_b,
            contact_point,
            normal,
            normal_velocity,
            restitution,
        );

        // effective_mass = 1 / (1 + 0) = 1
        // impulse = -(1 + 0.6) * (-1) * 1 = 1.6
        assert!(approx_eq_eps(impulse, 1.6, 1e-5));
    }

    #[test]
    fn test_contact_impulse_is_zero_for_immovable_bodies() {
        let body_a = Body::new_static(BodyHandle::new(1, 0));
        let body_b = Body::new_static(BodyHandle::new(2, 0));

        let impulse = calculate_contact_impulse(
            &body_a,
            &body_b,
            Vec2::zero(),
            Vec2::unit_y(),
            -1.0,
            0.5,
        );

        assert_eq!(impulse, 0.0);
    }

    #[test]
    fn test_solve_updates_bodies_in_place() {
        use crate::collision::{ColliderHandle, Contact, ContactId};

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
