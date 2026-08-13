//! Sequential impulse solver for the physics engine.
//!
//! This module implements a sequential impulse solver that resolves
//! collisions and constraints between rigid bodies.

use crate::bodies::Body;
use crate::collision::ContactManifold;
use crate::math::Vec2;
use super::SolverConfig;

/// Sequential impulse solver.
///
/// This solver resolves collisions by iteratively applying impulses
/// to satisfy contact constraints. It handles:
/// - Normal impulses (preventing interpenetration)
/// - Friction impulses (Coulomb friction model)
/// - Restitution (bounciness)
/// - Warm starting (using previous frame's impulses)
/// - Position correction (Baumgarte stabilization)
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
    /// This applies impulses to resolve collisions between bodies.
    #[inline]
    pub fn solve_contacts(
        &self,
        bodies: &mut [Option<Body>],
        manifolds: &mut [ContactManifold],
        dt: f32,
    ) {
        if manifolds.is_empty() {
            return;
        }

        // Warm starting: apply accumulated impulses from previous frame
        if self.config.warm_starting {
            self.apply_warm_starting(bodies, manifolds);
        }

        // Velocity iterations
        for _ in 0..self.config.velocity_iterations {
            for manifold in manifolds.iter_mut() {
                self.solve_velocity_constraints(bodies, manifold, dt);
            }
        }

        // Position corrections
        if self.config.position_iterations > 0 {
            for _ in 0..self.config.position_iterations {
                for manifold in manifolds.iter_mut() {
                    self.solve_position_constraints(bodies, manifold);
                }
            }
        }
    }

    /// Apply warm starting impulses
    fn apply_warm_starting(&self, bodies: &mut [Option<Body>], manifolds: &mut [ContactManifold]) {
        for manifold in manifolds {
            for contact in manifold.contacts() {
                let body_a = self.get_body_mut(bodies, contact.contact_id().feature_a as usize);
                let body_b = self.get_body_mut(bodies, contact.contact_id().feature_b as usize);

                if let (Some(body_a), Some(body_b)) = (body_a, body_b) {
                    // Apply accumulated normal impulse
                    let impulse = contact.normal_impulse();
                    if impulse > 0.0 {
                        let normal = contact.normal();
                        self.apply_impulse(
                            body_a, body_b,
                            normal,
                            impulse,
                            contact.position(),
                        );
                    }

                    // Apply accumulated tangent impulse
                    let tangent = contact.normal().perp();
                    let tangent_impulse = contact.tangent_impulse();
                    if tangent_impulse.abs() > 0.0 {
                        self.apply_impulse(
                            body_a, body_b,
                            tangent,
                            tangent_impulse,
                            contact.position(),
                        );
                    }
                }
            }
        }
    }

    /// Solve velocity constraints
    fn solve_velocity_constraints(
        &self,
        bodies: &mut [Option<Body>],
        manifold: &mut ContactManifold,
        dt: f32,
    ) {
        for contact in manifold.contacts_mut() {
            let body_a = self.get_body_mut(bodies, contact.contact_id().feature_a as usize);
            let body_b = self.get_body_mut(bodies, contact.contact_id().feature_b as usize);

            if let (Some(body_a), Some(body_b)) = (body_a, body_b) {
                // Skip if either body is static
                if body_a.is_static() && body_b.is_static() {
                    continue;
                }

                // Calculate relative velocity at contact point
                let relative_velocity = self.calculate_relative_velocity(
                    body_a, body_b, contact.position()
                );

                // Calculate normal velocity
                let normal_velocity = relative_velocity.dot(contact.normal());

                // Only resolve if bodies are moving toward each other
                if normal_velocity > 0.0 {
                    continue;
                }

                // Calculate normal impulse
                let normal_impulse = self.calculate_normal_impulse(
                    body_a, body_b,
                    contact,
                    normal_velocity,
                    dt,
                );

                // Apply normal impulse
                self.apply_normal_impulse(
                    body_a, body_b,
                    contact,
                    normal_impulse,
                    dt,
                );

                // Calculate tangent impulse (friction)
                if self.config.enable_friction {
                    let tangent_impulse = self.calculate_tangent_impulse(
                        body_a, body_b,
                        contact,
                        relative_velocity,
                        normal_velocity,
                        normal_impulse,
                        dt,
                    );

                    // Apply tangent impulse
                    self.apply_tangent_impulse(
                        body_a, body_b,
                        contact,
                        tangent_impulse,
                        dt,
                    );
                }
            }
        }
    }

    /// Calculate relative velocity at a contact point
    #[inline]
    fn calculate_relative_velocity(&self, body_a: &Body, body_b: &Body, contact_point: Vec2) -> Vec2 {
        // Velocity at contact point for body A
        let va = body_a.velocity_at_point(contact_point);
        
        // Velocity at contact point for body B
        let vb = body_b.velocity_at_point(contact_point);
        
        // Relative velocity
        va - vb
    }

    /// Calculate normal impulse
    #[inline]
    fn calculate_normal_impulse(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        contact: &mut super::ContactManifold::Contact,
        normal_velocity: f32,
        dt: f32,
    ) -> f32 {
        let normal = contact.normal();
        let contact_point = contact.position();

        // Calculate effective mass
        let effective_mass = self.calculate_effective_mass(
            body_a, body_b,
            normal,
            contact_point - body_a.position(),
            contact_point - body_b.position(),
        );

        if effective_mass <= EPSILON {
            return 0.0;
        }

        // Calculate restitution
        let restitution = self.calculate_restitution(body_a, body_b, normal_velocity);

        // Normal impulse = -(1 + e) * v_n / m_eff
        let impulse = -(1.0 + restitution) * normal_velocity / effective_mass;

        // Clamp to prevent negative impulses (bodies moving apart)
        let old_impulse = contact.normal_impulse();
        let new_impulse = (old_impulse + impulse).max(0.0);
        
        // Store the accumulated impulse for warm starting
        // Note: We don't modify contact here, that's done in apply_normal_impulse
        
        new_impulse - old_impulse
    }

    /// Apply normal impulse
    #[inline]
    fn apply_normal_impulse(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        contact: &mut super::ContactManifold::Contact,
        impulse: f32,
        _dt: f32,
    ) {
        if impulse <= EPSILON {
            return;
        }

        let normal = contact.normal();
        let contact_point = contact.position();

        // Apply to body A
        if !body_a.is_static() {
            let ra = contact_point - body_a.position();
            body_a.add_impulse_at_point(-normal * impulse, contact_point);
            
            // Update accumulated impulse for warm starting
            // We'll do this after applying to both bodies
        }

        // Apply to body B
        if !body_b.is_static() {
            let rb = contact_point - body_b.position();
            body_b.add_impulse_at_point(normal * impulse, contact_point);
        }

        // Update accumulated impulse in contact
        contact.normal_impulse += impulse;
    }

    /// Calculate effective mass for a contact
    #[inline]
    fn calculate_effective_mass(
        &self,
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

        // Effective mass = 1 / (inv_mass_a + inv_mass_b + 
        //                  ra.cross(normal)^2 * inv_inertia_a +
        //                  rb.cross(normal)^2 * inv_inertia_b)
        let ra_cross_normal = ra.cross(normal);
        let rb_cross_normal = rb.cross(normal);

        let denominator = inv_mass_a + inv_mass_b
            + ra_cross_normal * ra_cross_normal * inv_inertia_a
            + rb_cross_normal * rb_cross_normal * inv_inertia_b;

        if denominator <= EPSILON {
            0.0
        } else {
            1.0 / denominator
        }
    }

    /// Calculate restitution
    #[inline]
    fn calculate_restitution(&self, body_a: &Body, body_b: &Body, normal_velocity: f32) -> f32 {
        // If relative velocity is very small, use 0 restitution
        if normal_velocity > -self.config.restitution_threshold {
            return 0.0;
        }

        // Combined restitution (average of both bodies' colliders)
        // For now, use a default value since we don't have access to colliders here
        // This will be improved when we integrate with the world
        0.6
    }

    /// Calculate tangent impulse (friction)
    #[inline]
    fn calculate_tangent_impulse(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        contact: &mut super::ContactManifold::Contact,
        relative_velocity: Vec2,
        normal_velocity: f32,
        normal_impulse: f32,
        _dt: f32,
    ) -> f32 {
        let normal = contact.normal();
        let contact_point = contact.position();

        // Calculate tangent vector (perpendicular to normal)
        let tangent = normal.perp();

        // Calculate tangent velocity
        let tangent_velocity = relative_velocity.dot(tangent);

        // Calculate effective mass for friction
        let effective_mass = self.calculate_effective_mass(
            body_a, body_b,
            tangent,
            contact_point - body_a.position(),
            contact_point - body_b.position(),
        );

        if effective_mass <= EPSILON {
            return 0.0;
        }

        // Calculate friction coefficient (combined from both bodies)
        // For now, use a default value
        let friction = 0.6;

        // Maximum friction impulse (Coulomb friction: F_friction <= u * F_normal)
        let max_friction = friction * normal_impulse.abs();

        // Friction impulse = -v_t / m_eff
        let mut friction_impulse = -tangent_velocity / effective_mass;

        // Clamp to maximum friction
        friction_impulse = friction_impulse.clamp(-max_friction, max_friction);

        // Apply sign based on direction
        if tangent_velocity < -EPSILON {
            friction_impulse = -friction_impulse;
        }

        friction_impulse
    }

    /// Apply tangent impulse
    #[inline]
    fn apply_tangent_impulse(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        contact: &mut super::ContactManifold::Contact,
        impulse: f32,
        _dt: f32,
    ) {
        if impulse.abs() <= EPSILON {
            return;
        }

        let tangent = contact.normal().perp();
        let contact_point = contact.position();

        // Apply to body A
        if !body_a.is_static() {
            body_a.add_impulse_at_point(-tangent * impulse, contact_point);
        }

        // Apply to body B
        if !body_b.is_static() {
            body_b.add_impulse_at_point(tangent * impulse, contact_point);
        }

        // Update accumulated tangent impulse
        contact.tangent_impulse += impulse;
    }

    /// Solve position constraints (Baumgarte stabilization)
    fn solve_position_constraints(
        &self,
        bodies: &mut [Option<Body>],
        manifold: &mut ContactManifold,
    ) {
        for contact in manifold.contacts_mut() {
            let body_a = self.get_body_mut(bodies, contact.contact_id().feature_a as usize);
            let body_b = self.get_body_mut(bodies, contact.contact_id().feature_b as usize);

            if let (Some(body_a), Some(body_b)) = (body_a, body_b) {
                // Skip if either body is static
                if body_a.is_static() && body_b.is_static() {
                    continue;
                }

                // Calculate position error
                let contact_point = contact.position();
                let separation = contact.penetration();

                // Only correct if penetration is significant
                if separation >= -EPSILON {
                    continue;
                }

                // Baumgarte stabilization: apply fraction of penetration correction
                let correction = self.config.baumgarte_factor * separation.max(-self.config.max_position_correction);

                // Calculate effective mass for position correction
                let normal = contact.normal();
                let ra = contact_point - body_a.position();
                let rb = contact_point - body_b.position();

                let effective_mass = self.calculate_effective_mass(
                    body_a, body_b,
                    normal,
                    ra,
                    rb,
                );

                if effective_mass <= EPSILON {
                    continue;
                }

                // Calculate position correction impulse
                let correction_impulse = -correction / effective_mass;

                // Apply position correction
                if !body_a.is_static() {
                    let impulse = normal * (correction_impulse * body_a.inv_mass());
                    body_a.set_position(body_a.position() + impulse);
                    
                    // Apply angular correction
                    let angular_impulse = ra.cross(normal) * correction_impulse * body_a.inv_inertia();
                    body_a.set_rotation(body_a.rotation() + Rot2::new(angular_impulse));
                }

                if !body_b.is_static() {
                    let impulse = -normal * (correction_impulse * body_b.inv_mass());
                    body_b.set_position(body_b.position() + impulse);
                    
                    // Apply angular correction
                    let angular_impulse = rb.cross(-normal) * correction_impulse * body_b.inv_inertia();
                    body_b.set_rotation(body_b.rotation() + Rot2::new(angular_impulse));
                }
            }
        }
    }

    /// Apply an impulse to two bodies
    #[inline]
    fn apply_impulse(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        direction: Vec2,
        impulse: f32,
        contact_point: Vec2,
    ) {
        if !body_a.is_static() {
            body_a.add_impulse_at_point(-direction * impulse, contact_point);
        }
        if !body_b.is_static() {
            body_b.add_impulse_at_point(direction * impulse, contact_point);
        }
    }

    /// Get a mutable reference to a body by index
    #[inline]
    fn get_body_mut(
        &self,
        bodies: &mut [Option<Body>],
        index: usize,
    ) -> Option<&mut Body> {
        bodies.get_mut(index).and_then(|b| b.as_mut())
    }
}

impl Default for SequentialImpulseSolver {
    fn default() -> Self {
        Self::new(SolverConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::math::utils::approx_eq;
    use crate::constants::EPSILON;

    #[test]
    fn test_construction() {
        let solver = SequentialImpulseSolver::new(SolverConfig::default());
        assert_eq!(solver.config().velocity_iterations, 8);
    }

    #[test]
    fn test_effective_mass() {
        let solver = SequentialImpulseSolver::default();

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_mass_properties(10.0, 5.0);

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_mass_properties(10.0, 5.0);

        let normal = Vec2::new(1.0, 0.0);
        let ra = Vec2::new(0.0, 0.0);
        let rb = Vec2::new(0.0, 0.0);

        let effective_mass = solver.calculate_effective_mass(&body_a, &body_b, normal, ra, rb);
        
        // For two bodies with mass 10, inv_mass = 0.1 each
        // effective_mass = 1 / (0.1 + 0.1) = 5.0
        assert!(approx_eq(effective_mass, 5.0, 1e-5));
    }

    #[test]
    fn test_effective_mass_with_inertia() {
        let solver = SequentialImpulseSolver::default();

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_mass_properties(10.0, 5.0);

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_mass_properties(10.0, 5.0);

        let normal = Vec2::new(0.0, 1.0);
        let ra = Vec2::new(1.0, 0.0); // 1 unit from center
        let rb = Vec2::new(0.0, 0.0);

        let effective_mass = solver.calculate_effective_mass(&body_a, &body_b, normal, ra, rb);
        
        // ra.cross(normal) = (1,0).cross(0,1) = 1*1 - 0*0 = 1
        // ra_term = 1^2 * (1/5) = 0.2
        // effective_mass = 1 / (0.1 + 0.1 + 0.2 + 0) = 1 / 0.4 = 2.5
        assert!(approx_eq(effective_mass, 2.5, 1e-5));
    }

    #[test]
    fn test_relative_velocity() {
        let solver = SequentialImpulseSolver::default();

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_linear_velocity(Vec2::new(1.0, 0.0));
        body_a.set_angular_velocity(0.0);

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_linear_velocity(Vec2::new(0.0, 0.0));
        body_b.set_angular_velocity(0.0);

        let contact_point = Vec2::new(0.0, 0.0);
        let relative_velocity = solver.calculate_relative_velocity(&body_a, &body_b, contact_point);
        
        assert_eq!(relative_velocity, Vec2::new(1.0, 0.0));
    }

    #[test]
    fn test_solve_empty() {
        let solver = SequentialImpulseSolver::default();
        let mut bodies: Vec<Option<Body>> = Vec::new();
        let mut manifolds: Vec<ContactManifold> = Vec::new();

        solver.solve_contacts(&mut bodies, &mut manifolds, 1.0 / 60.0);
        // Should not panic
    }

    #[test]
    fn test_solve_single_contact() {
        let solver = SequentialImpulseSolver::new(
            SolverConfig::new()
                .with_velocity_iterations(1)
                .with_position_iterations(0)
                .with_warm_starting(false)
        );

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::new(0.0, 0.0));
        body_a.set_mass_properties(1.0, 0.5);
        body_a.set_linear_velocity(Vec2::new(0.0, -1.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_static(handle_b);
        body_b.set_position(Vec2::new(0.0, -1.0));

        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(0.0, -0.5),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            super::super::ContactId::new(0, 0),
        );

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut manifolds = vec![manifold];

        solver.solve_contacts(&mut bodies, &mut manifolds, 1.0 / 60.0);

        // Body A should have bounced (velocity should be positive now)
        let body_a = bodies[0].as_ref().unwrap();
        assert!(body_a.linear_velocity().y > 0.0, 
                "Body should bounce upward after collision");
    }

    #[test]
    fn test_position_correction() {
        let solver = SequentialImpulseSolver::new(
            SolverConfig::new()
                .with_velocity_iterations(0)
                .with_position_iterations(1)
                .with_baumgarte_factor(0.2)
                .with_max_position_correction(0.1)
        );

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::new(0.0, 0.0));
        body_a.set_mass_properties(1.0, 0.5);

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(0.0, -0.5));
        body_b.set_mass_properties(1.0, 0.5);

        let mut manifold = ContactManifold::new();
        // Penetration of 0.5 (bodies are overlapping by 0.5)
        manifold.add_contact(
            Vec2::new(0.0, -0.25),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            super::super::ContactId::new(0, 0),
        );

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut manifolds = vec![manifold];

        let original_pos_a = bodies[0].as_ref().unwrap().position();
        let original_pos_b = bodies[1].as_ref().unwrap().position();

        solver.solve_contacts(&mut bodies, &mut manifolds, 1.0 / 60.0);

        // Bodies should have moved apart slightly
        let new_pos_a = bodies[0].as_ref().unwrap().position();
        let new_pos_b = bodies[1].as_ref().unwrap().position();

        let separation = (new_pos_a.y - new_pos_b.y) - (original_pos_a.y - original_pos_b.y);
        assert!(separation > 0.0, "Position correction should increase separation");
    }

    #[test]
    fn test_static_body_no_movement() {
        let solver = SequentialImpulseSolver::default();

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_static(handle_a);
        body_a.set_position(Vec2::new(0.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(0.0, 1.0));
        body_b.set_mass_properties(1.0, 0.5);
        body_b.set_linear_velocity(Vec2::new(0.0, -1.0));

        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(0.0, 0.5),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            super::super::ContactId::new(0, 0),
        );

        let original_pos_a = body_a.position();
        let original_vel_b = body_b.linear_velocity();

        let mut bodies = vec![Some(body_a), Some(body_b)];
        let mut manifolds = vec![manifold];

        solver.solve_contacts(&mut bodies, &mut manifolds, 1.0 / 60.0);

        // Static body should not move
        let new_pos_a = bodies[0].as_ref().unwrap().position();
        assert_eq!(new_pos_a, original_pos_a);

        // Dynamic body should bounce
        let new_vel_b = bodies[1].as_ref().unwrap().linear_velocity();
        assert!(new_vel_b.y > original_vel_b.y, "Dynamic body should bounce");
    }
}
