//! Sequential impulse solver for the physics engine.
//!
//! This module implements a sequential impulse (SI) solver: contacts are
//! turned into velocity constraints and solved iteratively, one contact at a
//! time, until the relative normal velocity at every contact point is
//! non-negative and friction stays inside its Coulomb cone.
//!
//! The implementation follows the approach popularised by Box2D:
//!
//! * **Warm starting** — accumulated impulses from the previous step are
//!   applied before iterating, which makes resting stacks converge quickly.
//! * **Baumgarte stabilisation** — penetration is removed with a velocity bias
//!   (plus an explicit positional relaxation pass) instead of a hard
//!   constraint, which keeps the system solvable.
//! * **Accumulated impulse clamping** — normal impulses stay non-negative
//!   (contacts can push but never pull) and friction impulses are clamped to
//!   `friction * normal_impulse`.
//!
//! Bodies are addressed by [`BodyHandle`], never by array position, so a
//! manifold produced by the narrow-phase can be solved directly.

use std::collections::HashMap;

use crate::bodies::{Body, BodyHandle};
use crate::collision::ContactManifold;
use crate::constants::{EPSILON, LINEAR_SLOP};
use crate::math::Vec2;
use super::SolverConfig;

/// A single contact point prepared for solving.
#[derive(Debug, Clone, Copy)]
struct ContactConstraint {
    /// Index of the contact in its manifold
    index: usize,
    /// Contact position in world space
    position: Vec2,
    /// Contact normal (points from A to B)
    normal: Vec2,
    /// Contact tangent (`normal.perp()`)
    tangent: Vec2,
    /// Lever arm from body A's centre of mass
    ra: Vec2,
    /// Lever arm from body B's centre of mass
    rb: Vec2,
    /// Effective mass along the normal
    normal_mass: f32,
    /// Effective mass along the tangent
    tangent_mass: f32,
    /// Baumgarte velocity bias used to push the bodies apart
    position_bias: f32,
    /// Velocity bias introduced by restitution
    restitution_bias: f32,
    /// Penetration depth (positive when overlapping)
    penetration: f32,
    /// Accumulated normal impulse
    normal_impulse: f32,
    /// Accumulated tangent (friction) impulse
    tangent_impulse: f32,
}

/// Sequential impulse solver.
#[derive(Debug, Clone)]
pub struct SequentialImpulseSolver {
    /// Solver configuration
    config: SolverConfig,
}

impl SequentialImpulseSolver {
    /// Create a new sequential impulse solver
    #[inline]
    #[must_use]
    pub fn new(config: SolverConfig) -> Self {
        Self { config }
    }

    /// Get the solver configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &SolverConfig {
        &self.config
    }

    /// Solve a set of contact manifolds
    ///
    /// `bodies` is the world's body storage and `body_indices` maps a body
    /// handle to its index in that storage. Manifolds whose bodies are missing,
    /// identical, or both non-dynamic are skipped, as are sensor manifolds
    /// (they report overlaps but never generate impulses).
    #[inline]
    pub fn solve_contacts(
        &self,
        bodies: &mut [Option<Body>],
        manifolds: &mut [ContactManifold],
        dt: f32,
        body_indices: &HashMap<BodyHandle, usize>,
    ) {
        if manifolds.is_empty() || !dt.is_finite() || dt <= 0.0 {
            return;
        }

        let inv_dt = 1.0 / dt;

        for manifold in manifolds.iter_mut() {
            if manifold.is_sensor() || manifold.contact_count() == 0 {
                continue;
            }

            let Some((index_a, index_b)) = resolve_pair(manifold.body_a(), manifold.body_b(), body_indices)
            else {
                continue;
            };

            let Some((body_a, body_b)) = two_bodies_mut(bodies, index_a, index_b) else {
                continue;
            };

            // Two non-dynamic bodies cannot exchange impulses
            if !body_a.is_dynamic() && !body_b.is_dynamic() {
                continue;
            }

            self.solve_manifold(body_a, body_b, manifold, inv_dt);
        }
    }

    /// Solve a single manifold (velocity + position constraints)
    fn solve_manifold(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        manifold: &mut ContactManifold,
        inv_dt: f32,
    ) {
        let friction = manifold.friction().max(0.0);
        let restitution = if self.config.enable_restitution {
            manifold.restitution().max(0.0)
        } else {
            0.0
        };

        let mut constraints = self.prepare_constraints(body_a, body_b, manifold, inv_dt, restitution);
        if constraints.is_empty() {
            return;
        }

        if self.config.warm_starting {
            self.warm_start(body_a, body_b, &constraints);
        }

        for _ in 0..self.config.velocity_iterations {
            self.solve_normal_constraints(body_a, body_b, &mut constraints);

            if self.config.enable_friction && friction > 0.0 {
                self.solve_friction_constraints(body_a, body_b, &mut constraints, friction);
            }
        }

        // Write the accumulated impulses back so the next step can warm start
        for constraint in &constraints {
            if let Some(contact) = manifold.contacts_mut().get_mut(constraint.index) {
                contact.set_normal_impulse(constraint.normal_impulse);
                contact.set_tangent_impulse(constraint.tangent_impulse);
            }
        }

        if self.config.position_iterations > 0 {
            self.solve_position_constraints(body_a, body_b, &mut constraints);
        }
    }

    /// Build the per-contact constraints for a manifold
    fn prepare_constraints(
        &self,
        body_a: &Body,
        body_b: &Body,
        manifold: &ContactManifold,
        inv_dt: f32,
        restitution: f32,
    ) -> Vec<ContactConstraint> {
        let mut constraints = Vec::with_capacity(manifold.contact_count());

        for (index, contact) in manifold.contacts().iter().enumerate() {
            if !contact.is_valid() {
                continue;
            }

            let normal = contact.normal();
            if normal.length_squared() <= EPSILON * EPSILON {
                continue;
            }
            let normal = normal.normalize();
            let tangent = normal.perp();
            let ra = contact.position() - body_a.position();
            let rb = contact.position() - body_b.position();

            let normal_mass = effective_mass(body_a, body_b, normal, ra, rb);
            let tangent_mass = effective_mass(body_a, body_b, tangent, ra, rb);

            // Penetration is positive while overlapping; separation is the
            // signed distance along the normal (negative while overlapping).
            let penetration = contact.penetration();
            let separation = -penetration;

            let position_bias = if separation > -LINEAR_SLOP {
                0.0
            } else {
                let push = (separation + LINEAR_SLOP).min(0.0);
                (-self.config.baumgarte_factor * inv_dt * push).min(self.config.max_position_correction * inv_dt)
            };

            // Restitution only kicks in above the configured threshold and is
            // based on the approach velocity measured before solving.
            let approach = relative_velocity(body_a, body_b, ra, rb).dot(normal);
            let restitution_bias = if restitution > 0.0 && approach < -self.config.restitution_threshold {
                -restitution * approach
            } else {
                0.0
            };

            constraints.push(ContactConstraint {
                index,
                position: contact.position(),
                normal,
                tangent,
                ra,
                rb,
                normal_mass,
                tangent_mass,
                position_bias,
                restitution_bias,
                penetration,
                normal_impulse: contact.normal_impulse(),
                tangent_impulse: contact.tangent_impulse(),
            });
        }

        constraints
    }

    /// Apply the impulses accumulated during the previous step
    fn warm_start(&self, body_a: &mut Body, body_b: &mut Body, constraints: &[ContactConstraint]) {
        for constraint in constraints {
            let impulse = constraint.normal * constraint.normal_impulse
                + constraint.tangent * constraint.tangent_impulse;
            apply_impulse(body_a, body_b, constraint.ra, constraint.rb, impulse);
        }
    }

    /// One pass over the normal constraints
    fn solve_normal_constraints(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        constraints: &mut [ContactConstraint],
    ) {
        for constraint in constraints.iter_mut() {
            if constraint.normal_mass <= 0.0 {
                continue;
            }

            let velocity = relative_velocity(body_a, body_b, constraint.ra, constraint.rb);
            let normal_velocity = velocity.dot(constraint.normal);

            let mut lambda = -constraint.normal_mass
                * (normal_velocity - constraint.position_bias - constraint.restitution_bias);

            // Contacts can push, never pull
            let new_impulse = (constraint.normal_impulse + lambda).max(0.0);
            lambda = new_impulse - constraint.normal_impulse;
            constraint.normal_impulse = new_impulse;

            if lambda != 0.0 {
                let impulse = constraint.normal * lambda;
                apply_impulse(body_a, body_b, constraint.ra, constraint.rb, impulse);
            }
        }
    }

    /// One pass over the friction constraints
    fn solve_friction_constraints(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        constraints: &mut [ContactConstraint],
        friction: f32,
    ) {
        for constraint in constraints.iter_mut() {
            if constraint.tangent_mass <= 0.0 {
                continue;
            }

            let velocity = relative_velocity(body_a, body_b, constraint.ra, constraint.rb);
            let tangent_velocity = velocity.dot(constraint.tangent);

            let mut lambda = -constraint.tangent_mass * tangent_velocity;

            // Coulomb friction cone: |P_t| <= friction * P_n
            let max_friction = friction * constraint.normal_impulse;
            let new_impulse =
                (constraint.tangent_impulse + lambda).clamp(-max_friction, max_friction);
            lambda = new_impulse - constraint.tangent_impulse;
            constraint.tangent_impulse = new_impulse;

            if lambda != 0.0 {
                let impulse = constraint.tangent * lambda;
                apply_impulse(body_a, body_b, constraint.ra, constraint.rb, impulse);
            }
        }
    }

    /// Relax remaining penetration by moving the bodies apart directly
    ///
    /// Velocities are left untouched: this removes the "sinking" that a pure
    /// Baumgarte bias cannot fix within the iteration budget, without adding
    /// energy to the simulation.
    fn solve_position_constraints(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        constraints: &mut [ContactConstraint],
    ) {
        let inv_mass_a = body_a.inv_mass();
        let inv_mass_b = body_b.inv_mass();
        let total_inv_mass = inv_mass_a + inv_mass_b;

        if total_inv_mass <= EPSILON {
            return;
        }

        for _ in 0..self.config.position_iterations {
            for constraint in constraints.iter_mut() {
                let excess = constraint.penetration - LINEAR_SLOP;
                if excess <= 0.0 {
                    continue;
                }

                let correction = (excess * self.config.baumgarte_factor)
                    .min(self.config.max_position_correction);
                if correction <= 0.0 {
                    continue;
                }

                let shift_a = -constraint.normal * (correction * inv_mass_a / total_inv_mass);
                let shift_b = constraint.normal * (correction * inv_mass_b / total_inv_mass);

                if body_a.is_dynamic() {
                    body_a.set_position(body_a.position() + shift_a);
                }
                if body_b.is_dynamic() {
                    body_b.set_position(body_b.position() + shift_b);
                }

                constraint.penetration -= correction;
            }
        }
    }
}

impl Default for SequentialImpulseSolver {
    fn default() -> Self {
        Self::new(SolverConfig::default())
    }
}

/// Resolve the storage indices of a manifold's bodies
#[inline]
fn resolve_pair(
    handle_a: BodyHandle,
    handle_b: BodyHandle,
    body_indices: &HashMap<BodyHandle, usize>,
) -> Option<(usize, usize)> {
    if handle_a.is_null() || handle_b.is_null() || handle_a == handle_b {
        return None;
    }

    let index_a = *body_indices.get(&handle_a)?;
    let index_b = *body_indices.get(&handle_b)?;

    Some((index_a, index_b))
}

/// Borrow two distinct bodies mutably at the same time
///
/// Uses `split_at_mut` (or `get_disjoint_mut` when available) so the borrow
/// checker accepts two simultaneous mutable references into the same slice.
#[inline]
fn two_bodies_mut(
    bodies: &mut [Option<Body>],
    index_a: usize,
    index_b: usize,
) -> Option<(&mut Body, &mut Body)> {
    if index_a == index_b || index_a >= bodies.len() || index_b >= bodies.len() {
        return None;
    }

    let (first, second) = if index_a < index_b {
        let (left, right) = bodies.split_at_mut(index_b);
        (left.get_mut(index_a)?, right.first_mut()?)
    } else {
        let (left, right) = bodies.split_at_mut(index_a);
        (right.first_mut()?, left.get_mut(index_b)?)
    };

    Some((first.as_mut()?, second.as_mut()?))
}

/// Effective mass of a contact along `direction`
#[inline]
fn effective_mass(body_a: &Body, body_b: &Body, direction: Vec2, ra: Vec2, rb: Vec2) -> f32 {
    let ra_cross = ra.cross(direction);
    let rb_cross = rb.cross(direction);

    let denominator = body_a.inv_mass()
        + body_b.inv_mass()
        + ra_cross * ra_cross * body_a.inv_inertia()
        + rb_cross * rb_cross * body_b.inv_inertia();

    if denominator <= EPSILON {
        0.0
    } else {
        1.0 / denominator
    }
}

/// Velocity of body B relative to body A at a contact point
///
/// The sign convention matches the manifold normal (which points from A to
/// B): a negative value along the normal means the bodies are approaching.
#[inline]
fn relative_velocity(body_a: &Body, body_b: &Body, ra: Vec2, rb: Vec2) -> Vec2 {
    let velocity_a = body_a.linear_velocity() + perpendicular(ra) * body_a.angular_velocity();
    let velocity_b = body_b.linear_velocity() + perpendicular(rb) * body_b.angular_velocity();

    velocity_b - velocity_a
}

/// Rotate a vector 90 degrees (omega x r in 2D)
#[inline]
fn perpendicular(r: Vec2) -> Vec2 {
    Vec2::new(-r.y, r.x)
}

/// Apply an impulse at a contact: `-impulse` on A, `+impulse` on B
///
/// Non-dynamic bodies are skipped (their inverse mass and inertia are zero,
/// so this is a no-op for them anyway).
#[inline]
fn apply_impulse(body_a: &mut Body, body_b: &mut Body, ra: Vec2, rb: Vec2, impulse: Vec2) {
    if body_a.is_dynamic() {
        body_a.set_linear_velocity(body_a.linear_velocity() - impulse * body_a.inv_mass());
        body_a.set_angular_velocity(
            body_a.angular_velocity() - ra.cross(impulse) * body_a.inv_inertia(),
        );
    }

    if body_b.is_dynamic() {
        body_b.set_linear_velocity(body_b.linear_velocity() + impulse * body_b.inv_mass());
        body_b.set_angular_velocity(
            body_b.angular_velocity() + rb.cross(impulse) * body_b.inv_inertia(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::{ColliderHandle, Contact, ContactId};
    use crate::math::Vec2;

    fn dynamic(handle: BodyHandle, position: Vec2) -> Body {
        let mut body = Body::new_dynamic(handle);
        body.set_mass_properties(1.0, 1.0);
        body.set_position(position);
        body
    }

    fn manifold_between(handle_a: BodyHandle, handle_b: BodyHandle, contacts: Vec<Contact>) -> ContactManifold {
        let mut manifold = ContactManifold::between(handle_a, handle_b, ColliderHandle::null(), ColliderHandle::null());
        for contact in contacts {
            manifold.add_contact(
                contact.position(),
                contact.normal(),
                contact.penetration(),
                0.0,
                0.0,
                contact.contact_id(),
            );
        }
        manifold.set_friction(0.5);
        manifold.set_restitution(0.0);
        manifold
    }

    #[test]
    fn test_construction() {
        let solver = SequentialImpulseSolver::new(SolverConfig::default());
        assert_eq!(solver.config().velocity_iterations, 8);
    }

    #[test]
    fn test_effective_mass_of_two_equal_bodies() {
        let a = dynamic(BodyHandle::new(1, 0), Vec2::zero());
        let b = dynamic(BodyHandle::new(2, 0), Vec2::zero());

        let mass = effective_mass(&a, &b, Vec2::unit_x(), Vec2::zero(), Vec2::zero());

        // 1 / (1 + 1) = 0.5
        assert!((mass - 0.5).abs() < 1e-5);
    }

    #[test]
    fn test_effective_mass_ignores_static_bodies() {
        let a = dynamic(BodyHandle::new(1, 0), Vec2::zero());
        let mut b = Body::new_static(BodyHandle::new(2, 0));
        b.set_position(Vec2::zero());

        let mass = effective_mass(&a, &b, Vec2::unit_x(), Vec2::zero(), Vec2::zero());

        // 1 / (1 + 0) = 1
        assert!((mass - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_relative_velocity_includes_rotation() {
        let mut a = dynamic(BodyHandle::new(1, 0), Vec2::zero());
        a.set_angular_velocity(2.0);
        let b = dynamic(BodyHandle::new(2, 0), Vec2::zero());

        // The result is the velocity of B relative to A, so the rotation of A
        // shows up with a negative sign: r = (1, 0) gives omega x r = (0, 2)
        let velocity = relative_velocity(&a, &b, Vec2::new(1.0, 0.0), Vec2::zero());

        assert!((velocity.x).abs() < 1e-5);
        assert!((velocity.y + 2.0).abs() < 1e-5);

        // Swapping the bodies flips the sign
        let velocity = relative_velocity(&b, &a, Vec2::zero(), Vec2::new(1.0, 0.0));
        assert!((velocity.y - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_two_bodies_mut_returns_distinct_bodies() {
        let mut bodies = vec![
            Some(dynamic(BodyHandle::new(1, 0), Vec2::zero())),
            Some(dynamic(BodyHandle::new(2, 0), Vec2::new(1.0, 0.0))),
        ];

        let (a, b) = two_bodies_mut(&mut bodies, 0, 1).expect("distinct bodies");
        assert_eq!(a.handle(), BodyHandle::new(1, 0));
        assert_eq!(b.handle(), BodyHandle::new(2, 0));

        assert!(two_bodies_mut(&mut bodies, 1, 1).is_none());
        assert!(two_bodies_mut(&mut bodies, 0, 7).is_none());
    }

    #[test]
    fn test_solver_stops_approaching_bodies() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        let mut body_a = dynamic(handle_a, Vec2::new(0.0, 1.0));
        body_a.set_linear_velocity(Vec2::new(0.0, -5.0));
        let body_b = Body::new_static(handle_b);

        let contact = Contact::without_impulses(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.0,
            ContactId::new(0, 0),
        );

        let mut manifold = manifold_between(handle_a, handle_b, vec![contact]);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        SequentialImpulseSolver::default().solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        let a = bodies[0].as_ref().expect("body a");
        // The body must no longer be moving into the ground
        assert!(a.linear_velocity().y >= -1e-3, "velocity {}", a.linear_velocity().y);
    }

    #[test]
    fn test_solver_never_pulls_bodies_together() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        // Two bodies touching but separating: no impulse must be applied
        let mut body_a = dynamic(handle_a, Vec2::new(0.0, 1.0));
        body_a.set_linear_velocity(Vec2::new(0.0, 5.0));
        let mut body_b = dynamic(handle_b, Vec2::new(0.0, 0.0));
        body_b.set_linear_velocity(Vec2::new(0.0, -5.0));

        let contact = Contact::without_impulses(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.0,
            ContactId::new(0, 0),
        );
        let mut manifold = manifold_between(handle_a, handle_b, vec![contact]);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        SequentialImpulseSolver::default().solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        assert!((bodies[0].as_ref().expect("a").linear_velocity().y - 5.0).abs() < 1e-3);
        assert!((bodies[1].as_ref().expect("b").linear_velocity().y + 5.0).abs() < 1e-3);
    }

    #[test]
    fn test_friction_slows_sliding_body() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        let mut body_a = dynamic(handle_a, Vec2::new(0.0, 1.0));
        // A small downward component is needed, otherwise the normal impulse
        // (and therefore the friction cone) is empty.
        body_a.set_linear_velocity(Vec2::new(4.0, -1.0));
        let body_b = Body::new_static(handle_b);

        let contact = Contact::without_impulses(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.0,
            ContactId::new(0, 0),
        );
        let mut manifold = manifold_between(handle_a, handle_b, vec![contact]);
        manifold.set_friction(1.0);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        SequentialImpulseSolver::default().solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        let a = bodies[0].as_ref().expect("body a");
        assert!(a.linear_velocity().x < 4.0, "friction should slow the body");
        // Vertical velocity is untouched by the tangent impulse
        assert!(a.linear_velocity().y.abs() < 1e-3);
    }

    #[test]
    fn test_restitution_makes_bodies_bounce() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        let mut body_a = dynamic(handle_a, Vec2::new(0.0, 1.0));
        body_a.set_linear_velocity(Vec2::new(0.0, -5.0));
        let body_b = Body::new_static(handle_b);

        let contact = Contact::without_impulses(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.0,
            ContactId::new(0, 0),
        );
        let mut manifold = manifold_between(handle_a, handle_b, vec![contact]);
        manifold.set_restitution(0.8);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        let config = SolverConfig::default().with_restitution_threshold(0.1);
        SequentialImpulseSolver::new(config).solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        let a = bodies[0].as_ref().expect("body a");
        // Restitution 0.8 on a 5 m/s impact leaves about 4 m/s of separation
        assert!(a.linear_velocity().y > 3.0, "velocity {}", a.linear_velocity().y);
    }

    #[test]
    fn test_sensors_generate_no_impulses() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        let mut body_a = dynamic(handle_a, Vec2::new(0.0, 1.0));
        body_a.set_linear_velocity(Vec2::new(0.0, -5.0));
        let body_b = Body::new_static(handle_b);

        let contact = Contact::without_impulses(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.1,
            ContactId::new(0, 0),
        );
        let mut manifold = manifold_between(handle_a, handle_b, vec![contact]);
        manifold.set_sensor(true);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        SequentialImpulseSolver::default().solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        let a = bodies[0].as_ref().expect("body a");
        assert!((a.linear_velocity().y + 5.0).abs() < 1e-3);
    }

    #[test]
    fn test_unknown_and_equal_handles_are_skipped() {
        let mut bodies: Vec<Option<Body>> = vec![Some(dynamic(BodyHandle::new(1, 0), Vec2::zero()))];
        let mut indices = HashMap::new();
        indices.insert(BodyHandle::new(1, 0), 0);

        let contact = Contact::without_impulses(
            Vec2::zero(),
            Vec2::new(0.0, -1.0),
            0.0,
            ContactId::new(0, 0),
        );

        // Unknown body handle
        let mut manifold =
            manifold_between(BodyHandle::new(1, 0), BodyHandle::new(9, 0), vec![contact]);
        SequentialImpulseSolver::default().solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        // Same body on both sides
        let mut manifold =
            manifold_between(BodyHandle::new(1, 0), BodyHandle::new(1, 0), vec![contact]);
        SequentialImpulseSolver::default().solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        let body = bodies[0].as_ref().expect("body");
        assert_eq!(body.linear_velocity(), Vec2::zero());
    }

    #[test]
    fn test_penetration_is_relaxed() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        let body_a = dynamic(handle_a, Vec2::new(0.0, 0.9));
        let body_b = Body::new_static(handle_b);

        let contact = Contact::without_impulses(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.2,
            ContactId::new(0, 0),
        );
        let mut manifold = manifold_between(handle_a, handle_b, vec![contact]);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        SequentialImpulseSolver::default().solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            1.0 / 60.0,
            &indices,
        );

        let a = bodies[0].as_ref().expect("body a");
        assert!(a.position().y > 0.9, "body should be pushed out, y = {}", a.position().y);
    }

    #[test]
    fn test_zero_timestep_is_ignored() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);

        let mut body_a = dynamic(handle_a, Vec2::new(0.0, 1.0));
        body_a.set_linear_velocity(Vec2::new(0.0, -5.0));
        let body_b = Body::new_static(handle_b);

        let contact = Contact::without_impulses(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, -1.0),
            0.0,
            ContactId::new(0, 0),
        );
        let mut manifold = manifold_between(handle_a, handle_b, vec![contact]);

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut indices = HashMap::new();
        indices.insert(handle_a, 0);
        indices.insert(handle_b, 1);

        let solver = SequentialImpulseSolver::default();
        solver.solve_contacts(&mut bodies, std::slice::from_mut(&mut manifold), 0.0, &indices);
        solver.solve_contacts(
            &mut bodies,
            std::slice::from_mut(&mut manifold),
            f32::NAN,
            &indices,
        );

        let a = bodies[0].as_ref().expect("body a");
        assert!((a.linear_velocity().y + 5.0).abs() < 1e-3);
    }
}
