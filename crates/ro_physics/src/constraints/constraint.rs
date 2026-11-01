//! Base constraint trait and implementation.
//!
//! This module provides the Constraint trait which is the foundation
//! for all joint constraint types.

use std::fmt;

use crate::bodies::{Body, BodyHandle};
use crate::math::Vec2;
use super::ConstraintHandle;

/// A constraint that connects two bodies together.
///
/// Constraints restrict the relative motion of bodies in specific ways.
/// All constraint types implement this trait.
pub trait Constraint: fmt::Debug + Send + Sync {
    /// Get the constraint handle
    fn handle(&self) -> ConstraintHandle;

    /// Get the first body handle
    fn body_a(&self) -> BodyHandle;

    /// Get the second body handle
    fn body_b(&self) -> BodyHandle;

    /// Check if the constraint is enabled
    fn is_enabled(&self) -> bool;

    /// Set if the constraint is enabled
    fn set_enabled(&mut self, enabled: bool);

    /// Pre-solve the constraint (calculate mass matrix, bias, etc.)
    fn pre_solve(
        &mut self,
        body_a: &Body,
        body_b: &Body,
        dt: f32,
    );

    /// Solve the constraint (apply impulses to satisfy the constraint)
    fn solve(&mut self, body_a: &mut Body, body_b: &mut Body);

    /// Post-solve the constraint (update accumulated impulses, etc.)
    fn post_solve(&mut self);

    /// Get the reaction force at body A
    fn reaction_force(&self) -> Vec2 {
        Vec2::zero()
    }

    /// Get the reaction torque at body A
    fn reaction_torque(&self) -> f32 {
        0.0
    }
}

/// Base struct for all constraints
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct BaseConstraint {
    /// Handle to this constraint
    handle: ConstraintHandle,
    /// First body handle
    body_a: BodyHandle,
    /// Second body handle
    pub body_b: BodyHandle,
    /// Whether the constraint is enabled
    enabled: bool,
    /// Whether to collide connected bodies
    collide_connected: bool,
}

impl BaseConstraint {
    /// Create a new base constraint
    #[inline]
    #[must_use]
    pub const fn new(
        handle: ConstraintHandle,
        body_a: BodyHandle,
        body_b: BodyHandle,
    ) -> Self {
        Self {
            handle,
            body_a,
            body_b,
            enabled: true,
            collide_connected: false,
        }
    }

    /// Get the constraint handle
    #[inline]
    #[must_use]
    pub const fn handle(&self) -> ConstraintHandle {
        self.handle
    }

    /// Get the first body handle
    #[inline]
    #[must_use]
    pub const fn body_a(&self) -> BodyHandle {
        self.body_a
    }

    /// Get the second body handle
    #[inline]
    #[must_use]
    pub const fn body_b(&self) -> BodyHandle {
        self.body_b
    }

    /// Check if the constraint is enabled
    #[inline]
    #[must_use]
    pub const fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Set if the constraint is enabled
    #[inline]
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if connected bodies should collide
    #[inline]
    #[must_use]
    pub const fn collide_connected(&self) -> bool {
        self.collide_connected
    }

    /// Set if connected bodies should collide
    #[inline]
    pub fn set_collide_connected(&mut self, collide: bool) {
        self.collide_connected = collide;
    }
}

/// Enum of all constraint types
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub enum ConstraintType {
    /// Distance joint (point-to-point constraint)
    DistanceJoint,
    /// Revolute joint (hinge constraint)
    RevoluteJoint,
    /// Motor joint (controlled rotation)
    MotorJoint,
}

impl fmt::Display for ConstraintType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConstraintType::DistanceJoint => write!(f, "DistanceJoint"),
            ConstraintType::RevoluteJoint => write!(f, "RevoluteJoint"),
            ConstraintType::MotorJoint => write!(f, "MotorJoint"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraints::ConstraintHandle;
    use crate::bodies::BodyHandle;

    #[test]
    fn test_base_constraint() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let constraint = BaseConstraint::new(handle, body_a, body_b);

        assert_eq!(constraint.handle(), handle);
        assert_eq!(constraint.body_a(), body_a);
        assert_eq!(constraint.body_b(), body_b);
        assert!(constraint.is_enabled());
        assert!(!constraint.collide_connected());
    }

    #[test]
    fn test_constraint_type_display() {
        let distance = ConstraintType::DistanceJoint;
        let revolute = ConstraintType::RevoluteJoint;
        let motor = ConstraintType::MotorJoint;

        assert_eq!(format!("{}", distance), "DistanceJoint");
        assert_eq!(format!("{}", revolute), "RevoluteJoint");
        assert_eq!(format!("{}", motor), "MotorJoint");
    }
}
