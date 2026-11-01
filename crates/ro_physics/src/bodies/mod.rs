//! Rigid body implementations for the physics engine.
//!
//! This module provides the core rigid body types and operations
//! for physics simulation.

mod body;
mod body_handle;
mod body_type;

pub use body::Body;
pub use body_handle::{BodyHandle, BodyHandleGenerator};
pub use body_type::BodyType;

/// Body utilities
pub mod utils {
    use super::Body;
    use crate::math::Vec2;

    /// Calculate the center of mass for a system of bodies
    #[inline]
    #[must_use]
    pub fn system_center_of_mass(bodies: &[Body]) -> Vec2 {
        let mut total_mass = 0.0;
        let mut com = Vec2::zero();

        for body in bodies {
            if body.is_dynamic() {
                let mass = body.mass();
                total_mass += mass;
                com += body.position() * mass;
            }
        }

        if total_mass > 0.0 {
            com / total_mass
        } else {
            Vec2::zero()
        }
    }

    /// Calculate the total mass of a system of bodies
    #[inline]
    #[must_use]
    pub fn system_total_mass(bodies: &[Body]) -> f32 {
        bodies
            .iter()
            .filter(|b| b.is_dynamic())
            .map(|b| b.mass())
            .sum()
    }

    /// Calculate the total linear momentum of a system of bodies
    #[inline]
    #[must_use]
    pub fn system_linear_momentum(bodies: &[Body]) -> Vec2 {
        bodies
            .iter()
            .filter(|b| b.is_dynamic())
            .map(|b| b.linear_velocity() * b.mass())
            .fold(Vec2::zero(), |acc, x| acc + x)
    }

    /// Calculate the total angular momentum of a system of bodies about a point
    #[inline]
    #[must_use]
    pub fn system_angular_momentum(bodies: &[Body], point: Vec2) -> f32 {
        bodies
            .iter()
            .filter(|b| b.is_dynamic())
            .map(|b| {
                let r = b.position() - point;
                let linear = b.linear_velocity() * b.mass();
                let angular = b.angular_velocity() * b.inertia();
                r.cross(linear) + angular
            })
            .sum()
    }

    /// Calculate the total kinetic energy of a system of bodies
    #[inline]
    #[must_use]
    pub fn system_kinetic_energy(bodies: &[Body]) -> f32 {
        bodies
            .iter()
            .filter(|b| b.is_dynamic())
            .map(|b| {
                let linear_ke = 0.5 * b.mass() * b.linear_velocity().length_squared();
                let angular_ke = 0.5 * b.inertia() * b.angular_velocity() * b.angular_velocity();
                linear_ke + angular_ke
            })
            .sum()
    }
}

pub use utils::*;
