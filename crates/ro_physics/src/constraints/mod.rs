//! Joint constraints for the physics engine.
//!
//! This module provides various joint constraint types that can be used to
//! connect bodies together with specific motion constraints.

mod constraint;
mod constraint_handle;
mod distance_joint;
mod revolute_joint;
mod motor_joint;

pub use constraint::{BaseConstraint, Constraint, ConstraintType};
pub use constraint_handle::{ConstraintHandle, ConstraintHandleGenerator};
pub use distance_joint::DistanceJoint;
pub use revolute_joint::RevoluteJoint;
pub use motor_joint::MotorJoint;

/// Constraint utilities
pub mod utils {
    use crate::bodies::Body;
    use crate::math::Vec2;

    /// Calculate the effective mass for a constraint along a given axis
    #[inline]
    #[must_use]
    pub fn calculate_constraint_effective_mass(
        body_a: &Body,
        body_b: &Body,
        axis: Vec2,
        ra: Vec2,
        rb: Vec2,
    ) -> f32 {
        let inv_mass_a = body_a.inv_mass();
        let inv_mass_b = body_b.inv_mass();
        let inv_inertia_a = body_a.inv_inertia();
        let inv_inertia_b = body_b.inv_inertia();

        let ra_cross_axis = ra.cross(axis);
        let rb_cross_axis = rb.cross(axis);

        let denominator = inv_mass_a + inv_mass_b
            + ra_cross_axis * ra_cross_axis * inv_inertia_a
            + rb_cross_axis * rb_cross_axis * inv_inertia_b;

        if denominator <= crate::constants::EPSILON {
            0.0
        } else {
            1.0 / denominator
        }
    }

    /// Calculate the bias for Baumgarte stabilization
    #[inline]
    #[must_use]
    pub fn calculate_bias(error: f32, baumgarte_factor: f32, dt: f32) -> f32 {
        baumgarte_factor * error / dt
    }
}

pub use utils::*;
