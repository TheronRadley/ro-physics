//! Integration methods for the physics engine.
//!
//! This module provides various integration methods for rigid body simulation.

use crate::bodies::Body;
use crate::math::Vec2;

/// Integration method trait
pub trait Integrator: Send + Sync {
    /// Integrate a body forward in time
    fn integrate(&self, body: &mut Body, gravity: Vec2, dt: f32);
}

/// Semi-implicit Euler integrator.
///
/// This is the most commonly used integrator for game physics.
/// It is stable for reasonable timesteps and provides good energy conservation.
#[derive(Debug, Clone, Copy)]
pub struct SemiImplicitEuler;

impl Integrator for SemiImplicitEuler {
    fn integrate(&self, body: &mut Body, gravity: Vec2, dt: f32) {
        if !body.is_dynamic() || !body.is_enabled() || body.is_sleeping() {
            return;
        }

        let inv_mass = body.inv_mass();
        let inv_inertia = body.inv_inertia();

        // Apply gravity
        let gravity_force = gravity * body.mass() * body.gravity_scale();

        // Update velocity
        let total_force = body.force() + gravity_force;
        let linear_acceleration = total_force * inv_mass;
        let angular_acceleration = body.torque() * inv_inertia;

        let new_linear_velocity = body.linear_velocity() + linear_acceleration * dt;
        let new_angular_velocity = body.angular_velocity() + angular_acceleration * dt;

        // Time-correct damping: v /= (1 + damping * dt). Unlike a plain
        // `v * (1 - damping)` this is stable for any damping and timestep.
        let linear_velocity = new_linear_velocity / (1.0 + body.linear_damping().max(0.0) * dt);
        let angular_velocity =
            new_angular_velocity / (1.0 + body.angular_damping().max(0.0) * dt);

        // Update position
        let new_position = body.position() + linear_velocity * dt;
        let new_rotation = body.rotation() + crate::math::Rot2::new(angular_velocity * dt);

        // Update body state
        body.set_position(new_position);
        body.set_rotation(new_rotation);
        body.set_linear_velocity(linear_velocity);
        body.set_angular_velocity(angular_velocity);

        // Clear accumulators
        body.clear_accumulators();
    }
}

/// Symplectic Euler integrator.
///
/// This integrator updates position and velocity in sequence.
/// It is less stable than semi-implicit Euler but can be more accurate for some systems.
#[derive(Debug, Clone, Copy)]
pub struct SymplecticEuler;

impl Integrator for SymplecticEuler {
    fn integrate(&self, body: &mut Body, gravity: Vec2, dt: f32) {
        if !body.is_dynamic() || !body.is_enabled() || body.is_sleeping() {
            return;
        }

        // Update position using current velocity
        let new_position = body.position() + body.linear_velocity() * dt;
        let new_rotation = body.rotation() + crate::math::Rot2::new(body.angular_velocity() * dt);

        // Update velocity using current force
        let inv_mass = body.inv_mass();
        let inv_inertia = body.inv_inertia();

        let gravity_force = gravity * body.mass() * body.gravity_scale();
        let total_force = body.force() + gravity_force;

        let linear_velocity = body.linear_velocity() + total_force * inv_mass * dt;
        let angular_velocity = body.angular_velocity() + body.torque() * inv_inertia * dt;

        // Time-correct damping: v /= (1 + damping * dt)
        let linear_velocity = linear_velocity / (1.0 + body.linear_damping().max(0.0) * dt);
        let angular_velocity = angular_velocity / (1.0 + body.angular_damping().max(0.0) * dt);

        // Update body state
        body.set_position(new_position);
        body.set_rotation(new_rotation);
        body.set_linear_velocity(linear_velocity);
        body.set_angular_velocity(angular_velocity);

        // Clear accumulators
        body.clear_accumulators();
    }
}

/// Integration utilities
pub mod utils {
    use super::*;

    /// Integrate all bodies using a specific integrator
    #[inline]
    pub fn integrate_bodies(
        bodies: &mut [Option<Body>],
        integrator: &dyn Integrator,
        gravity: Vec2,
        dt: f32,
    ) {
        for body in bodies {
            if let Some(body) = body {
                integrator.integrate(body, gravity, dt);
            }
        }
    }
}

pub use utils::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::math::utils::approx_eq_eps;

    #[test]
    fn test_semi_implicit_euler() {
        let integrator = SemiImplicitEuler;
        
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);
        body.set_mass_properties(10.0, 5.0);
        body.set_linear_velocity(Vec2::new(0.0, 0.0));
        body.set_angular_velocity(0.0);
        body.add_force(Vec2::new(10.0, 0.0));

        let gravity = Vec2::new(0.0, -9.81);
        let dt = 0.1;

        integrator.integrate(&mut body, gravity, dt);

        // F = ma => a = F/m = 10/10 = 1
        // v = v0 + a*dt = 0 + 1*0.1 = 0.1
        assert!(approx_eq_eps(body.linear_velocity().x, 0.1, 1e-5));
        
        // x = x0 + v*dt = 0 + 0.1*0.1 = 0.01
        assert!(approx_eq_eps(body.position().x, 0.01, 1e-5));
    }

    #[test]
    fn test_symplectic_euler() {
        let integrator = SymplecticEuler;
        
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);
        body.set_mass_properties(10.0, 5.0);
        body.set_linear_velocity(Vec2::new(1.0, 0.0));
        body.set_angular_velocity(0.0);
        body.add_force(Vec2::new(10.0, 0.0));

        let gravity = Vec2::new(0.0, -9.81);
        let dt = 0.1;

        integrator.integrate(&mut body, gravity, dt);

        // x = x0 + v*dt = 0 + 1*0.1 = 0.1
        assert!(approx_eq_eps(body.position().x, 0.1, 1e-5));
        
        // F = ma => a = F/m = 10/10 = 1
        // v = v0 + a*dt = 1 + 1*0.1 = 1.1
        assert!(approx_eq_eps(body.linear_velocity().x, 1.1, 1e-5));
    }

    #[test]
    fn test_integrate_bodies() {
        let integrator = SemiImplicitEuler;
        
        let handle1 = BodyHandle::new(1, 0);
        let mut body1 = Body::new_dynamic(handle1);
        body1.set_mass_properties(10.0, 5.0);
        body1.set_linear_velocity(Vec2::new(0.0, 0.0));
        body1.add_force(Vec2::new(10.0, 0.0));

        let handle2 = BodyHandle::new(2, 0);
        let mut body2 = Body::new_dynamic(handle2);
        body2.set_mass_properties(5.0, 2.5);
        body2.set_linear_velocity(Vec2::new(0.0, 0.0));
        body2.add_force(Vec2::new(0.0, 5.0));

        let mut bodies = vec![Some(body1), Some(body2)];
        let gravity = Vec2::new(0.0, -9.81);
        let dt = 0.1;

        integrate_bodies(&mut bodies, &integrator, gravity, dt);

        assert_eq!(bodies.len(), 2);
        assert!(bodies[0].as_ref().unwrap().linear_velocity().x > 0.0);

        // Body 2 only pushes with 5 N on a mass of 5 kg, i.e. 1 m/s^2, which
        // gravity (9.81 m/s^2) easily wins, so it still ends up moving down.
        assert!(approx_eq_eps(
            bodies[1].as_ref().unwrap().linear_velocity().y,
            (1.0 - 9.81) * 0.1,
            1e-5
        ));
    }

    #[test]
    fn test_damping() {
        let integrator = SemiImplicitEuler;
        
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);
        body.set_mass_properties(10.0, 5.0);
        body.set_linear_velocity(Vec2::new(10.0, 0.0));
        body.set_linear_damping(0.5);

        let gravity = Vec2::zero();
        let dt = 0.1;

        integrator.integrate(&mut body, gravity, dt);

        // v = v0 / (1 + damping * dt) = 10 / 1.05
        assert!(approx_eq_eps(body.linear_velocity().x, 10.0 / 1.05, 1e-5));
    }

    #[test]
    fn test_static_body() {
        let integrator = SemiImplicitEuler;
        
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_static(handle);
        body.set_linear_velocity(Vec2::new(10.0, 0.0));
        body.add_force(Vec2::new(100.0, 0.0));

        let gravity = Vec2::new(0.0, -9.81);
        let dt = 0.1;

        let original_position = body.position();
        let original_velocity = body.linear_velocity();

        integrator.integrate(&mut body, gravity, dt);

        // Static bodies should not move
        assert_eq!(body.position(), original_position);
        assert_eq!(body.linear_velocity(), original_velocity);
    }
}
