//! Motor joint constraint implementation.
//!
//! A motor joint provides controlled relative motion between two bodies.
//! It can control linear and/or angular motion with configurable limits.

use std::fmt;

use crate::bodies::Body;
use crate::constants::EPSILON;
use crate::math::Vec2;
use crate::bodies::BodyHandle;
use super::{BaseConstraint, Constraint, ConstraintHandle};

/// A motor joint constraint.
///
/// This constraint provides controlled relative motion between two bodies.
/// It can be used to create various effects:
/// - Linear motor: controlled linear motion along an axis
/// - Angular motor: controlled angular motion
/// - Both combined
///
/// The motor can have limits and can apply maximum forces/torques.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorJoint {
    /// Base constraint data
    base: BaseConstraint,
    /// Linear offset from body A's center to the motor axis
    linear_offset: Vec2,
    /// Angular offset from body A's angle to the motor axis
    angular_offset: f32,
    /// Linear axis direction (in world space when created)
    axis: Vec2,
    /// Maximum linear force
    max_force: f32,
    /// Maximum linear correction force (for position correction)
    max_correction_force: f32,
    /// Maximum angular force (torque)
    max_torque: f32,
    /// Maximum angular correction torque
    max_correction_torque: f32,
    /// Linear stiffness (0-1)
    linear_stiffness: f32,
    /// Angular stiffness (0-1)
    angular_stiffness: f32,
    /// Linear damping (0-1)
    linear_damping: f32,
    /// Angular damping (0-1)
    angular_damping: f32,
    /// Target linear velocity
    target_linear_velocity: Vec2,
    /// Target angular velocity
    target_angular_velocity: f32,
    /// Whether to enable linear correction
    enable_linear_correction: bool,
    /// Whether to enable angular correction
    enable_angular_correction: bool,
    /// Accumulated linear impulse
    linear_impulse: Vec2,
    /// Accumulated angular impulse
    angular_impulse: f32,
    /// Effective mass for linear constraint
    linear_effective_mass: f32,
    /// Effective mass for angular constraint
    angular_effective_mass: f32,
}

impl MotorJoint {
    /// Create a new motor joint
    #[inline]
    #[must_use]
    pub fn new(
        handle: ConstraintHandle,
        body_a: BodyHandle,
        body_b: BodyHandle,
    ) -> Self {
        Self {
            base: BaseConstraint::new(handle, body_a, body_b),
            linear_offset: Vec2::zero(),
            angular_offset: 0.0,
            axis: Vec2::unit_x(),
            max_force: 0.0,
            max_correction_force: 1.0,
            max_torque: 0.0,
            max_correction_torque: 1.0,
            linear_stiffness: 1.0,
            angular_stiffness: 1.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            target_linear_velocity: Vec2::zero(),
            target_angular_velocity: 0.0,
            enable_linear_correction: true,
            enable_angular_correction: true,
            linear_impulse: Vec2::zero(),
            angular_impulse: 0.0,
            linear_effective_mass: 0.0,
            angular_effective_mass: 0.0,
        }
    }

    /// Create a new motor joint with a specific axis
    #[inline]
    #[must_use]
    pub fn with_axis(
        handle: ConstraintHandle,
        body_a: BodyHandle,
        body_b: BodyHandle,
        axis: Vec2,
    ) -> Self {
        let mut joint = Self::new(handle, body_a, body_b);
        joint.axis = axis.normalize();
        joint
    }

    /// Get the linear offset
    #[inline]
    #[must_use]
    pub const fn linear_offset(&self) -> Vec2 {
        self.linear_offset
    }

    /// Set the linear offset
    #[inline]
    pub fn set_linear_offset(&mut self, offset: Vec2) {
        self.linear_offset = offset;
    }

    /// Get the angular offset
    #[inline]
    #[must_use]
    pub const fn angular_offset(&self) -> f32 {
        self.angular_offset
    }

    /// Set the angular offset
    #[inline]
    pub fn set_angular_offset(&mut self, offset: f32) {
        self.angular_offset = offset;
    }

    /// Get the axis
    #[inline]
    #[must_use]
    pub const fn axis(&self) -> Vec2 {
        self.axis
    }

    /// Set the axis
    #[inline]
    pub fn set_axis(&mut self, axis: Vec2) {
        self.axis = axis.normalize();
    }

    /// Get the maximum force
    #[inline]
    #[must_use]
    pub const fn max_force(&self) -> f32 {
        self.max_force
    }

    /// Set the maximum force
    #[inline]
    pub fn set_max_force(&mut self, max_force: f32) {
        self.max_force = max_force.max(0.0);
    }

    /// Get the maximum correction force
    #[inline]
    #[must_use]
    pub const fn max_correction_force(&self) -> f32 {
        self.max_correction_force
    }

    /// Set the maximum correction force
    #[inline]
    pub fn set_max_correction_force(&mut self, max_force: f32) {
        self.max_correction_force = max_force.max(0.0);
    }

    /// Get the maximum torque
    #[inline]
    #[must_use]
    pub const fn max_torque(&self) -> f32 {
        self.max_torque
    }

    /// Set the maximum torque
    #[inline]
    pub fn set_max_torque(&mut self, max_torque: f32) {
        self.max_torque = max_torque.max(0.0);
    }

    /// Get the maximum correction torque
    #[inline]
    #[must_use]
    pub const fn max_correction_torque(&self) -> f32 {
        self.max_correction_torque
    }

    /// Set the maximum correction torque
    #[inline]
    pub fn set_max_correction_torque(&mut self, max_torque: f32) {
        self.max_correction_torque = max_torque.max(0.0);
    }

    /// Get the linear stiffness
    #[inline]
    #[must_use]
    pub const fn linear_stiffness(&self) -> f32 {
        self.linear_stiffness
    }

    /// Set the linear stiffness
    #[inline]
    pub fn set_linear_stiffness(&mut self, stiffness: f32) {
        self.linear_stiffness = stiffness.clamp(0.0, 1.0);
    }

    /// Get the angular stiffness
    #[inline]
    #[must_use]
    pub const fn angular_stiffness(&self) -> f32 {
        self.angular_stiffness
    }

    /// Set the angular stiffness
    #[inline]
    pub fn set_angular_stiffness(&mut self, stiffness: f32) {
        self.angular_stiffness = stiffness.clamp(0.0, 1.0);
    }

    /// Get the linear damping
    #[inline]
    #[must_use]
    pub const fn linear_damping(&self) -> f32 {
        self.linear_damping
    }

    /// Set the linear damping
    #[inline]
    pub fn set_linear_damping(&mut self, damping: f32) {
        self.linear_damping = damping.clamp(0.0, 1.0);
    }

    /// Get the angular damping
    #[inline]
    #[must_use]
    pub const fn angular_damping(&self) -> f32 {
        self.angular_damping
    }

    /// Set the angular damping
    #[inline]
    pub fn set_angular_damping(&mut self, damping: f32) {
        self.angular_damping = damping.clamp(0.0, 1.0);
    }

    /// Get the target linear velocity
    #[inline]
    #[must_use]
    pub const fn target_linear_velocity(&self) -> Vec2 {
        self.target_linear_velocity
    }

    /// Set the target linear velocity
    #[inline]
    pub fn set_target_linear_velocity(&mut self, velocity: Vec2) {
        self.target_linear_velocity = velocity;
    }

    /// Get the target angular velocity
    #[inline]
    #[must_use]
    pub const fn target_angular_velocity(&self) -> f32 {
        self.target_angular_velocity
    }

    /// Set the target angular velocity
    #[inline]
    pub fn set_target_angular_velocity(&mut self, velocity: f32) {
        self.target_angular_velocity = velocity;
    }

    /// Check if linear correction is enabled
    #[inline]
    #[must_use]
    pub const fn enable_linear_correction(&self) -> bool {
        self.enable_linear_correction
    }

    /// Enable or disable linear correction
    #[inline]
    pub fn set_enable_linear_correction(&mut self, enable: bool) {
        self.enable_linear_correction = enable;
    }

    /// Check if angular correction is enabled
    #[inline]
    #[must_use]
    pub const fn enable_angular_correction(&self) -> bool {
        self.enable_angular_correction
    }

    /// Enable or disable angular correction
    #[inline]
    pub fn set_enable_angular_correction(&mut self, enable: bool) {
        self.enable_angular_correction = enable;
    }

    /// Set all motor parameters at once
    #[inline]
    pub fn set_motor_parameters(
        &mut self,
        target_linear_velocity: Vec2,
        target_angular_velocity: f32,
        max_force: f32,
        max_torque: f32,
    ) {
        self.target_linear_velocity = target_linear_velocity;
        self.target_angular_velocity = target_angular_velocity;
        self.max_force = max_force;
        self.max_torque = max_torque;
    }
}

impl Constraint for MotorJoint {
    fn handle(&self) -> ConstraintHandle {
        self.base.handle()
    }

    fn body_a(&self) -> BodyHandle {
        self.base.body_a()
    }

    fn body_b(&self) -> BodyHandle {
        self.base.body_b()
    }

    fn is_enabled(&self) -> bool {
        self.base.is_enabled()
    }

    fn set_enabled(&mut self, enabled: bool) {
        self.base.set_enabled(enabled);
    }

    fn pre_solve(&mut self, body_a: &Body, body_b: &Body, dt: f32) {
        if !self.base.is_enabled() {
            self.linear_impulse = Vec2::zero();
            self.angular_impulse = 0.0;
            self.linear_effective_mass = 0.0;
            self.angular_effective_mass = 0.0;
            return;
        }

        // Calculate effective masses
        self.linear_effective_mass = self.calculate_linear_effective_mass(body_a, body_b);
        self.angular_effective_mass = self.calculate_angular_effective_mass(body_a, body_b);

        // Calculate current relative velocity
        let relative_linear_velocity = self.calculate_relative_linear_velocity(body_a, body_b);
        let relative_angular_velocity = self.calculate_relative_angular_velocity(body_a, body_b);

        // Calculate velocity errors
        let linear_velocity_error = self.target_linear_velocity - relative_linear_velocity;
        let angular_velocity_error = self.target_angular_velocity - relative_angular_velocity;

        // Apply stiffness
        if self.linear_stiffness < 1.0 - EPSILON {
            // Soft constraint
        }

        if self.angular_stiffness < 1.0 - EPSILON {
            // Soft constraint
        }

        // Calculate position errors for correction
        if self.enable_linear_correction {
            // Position error calculation
        }

        if self.enable_angular_correction {
            // Angle error calculation
        }
    }

    fn solve(&mut self, body_a: &mut Body, body_b: &mut Body) {
        if !self.base.is_enabled() {
            return;
        }

        // Solve linear constraint
        if self.linear_effective_mass > EPSILON {
            let relative_linear_velocity = self.calculate_relative_linear_velocity(body_a, body_b);
            let linear_velocity_error = self.target_linear_velocity - relative_linear_velocity;

            // Calculate linear impulse
            let linear_impulse = self.calculate_linear_impulse(
                body_a, body_b, linear_velocity_error
            );

            // Apply damping
            let linear_impulse = if self.linear_damping > EPSILON {
                linear_impulse * (1.0 - self.linear_damping)
            } else {
                linear_impulse
            };

            // Apply stiffness
            let linear_impulse = if self.linear_stiffness < 1.0 - EPSILON {
                linear_impulse * self.linear_stiffness
            } else {
                linear_impulse
            };

            // Apply the impulse
            self.apply_linear_impulse(body_a, body_b, linear_impulse);
            self.linear_impulse += linear_impulse;
        }

        // Solve angular constraint
        if self.angular_effective_mass > EPSILON {
            let relative_angular_velocity = self.calculate_relative_angular_velocity(body_a, body_b);
            let angular_velocity_error = self.target_angular_velocity - relative_angular_velocity;

            // Calculate angular impulse
            let angular_impulse = self.calculate_angular_impulse(
                body_a, body_b, angular_velocity_error
            );

            // Apply damping
            let angular_impulse = if self.angular_damping > EPSILON {
                angular_impulse * (1.0 - self.angular_damping)
            } else {
                angular_impulse
            };

            // Apply stiffness
            let angular_impulse = if self.angular_stiffness < 1.0 - EPSILON {
                angular_impulse * self.angular_stiffness
            } else {
                angular_impulse
            };

            // Apply the impulse
            self.apply_angular_impulse(body_a, body_b, angular_impulse);
            self.angular_impulse += angular_impulse;
        }
    }

    fn post_solve(&mut self) {
        // Clear accumulated impulses
        self.linear_impulse = Vec2::zero();
        self.angular_impulse = 0.0;
    }

    fn reaction_force(&self) -> Vec2 {
        self.linear_impulse
    }

    fn reaction_torque(&self) -> f32 {
        self.angular_impulse
    }
}

impl MotorJoint {
    /// Calculate the effective mass for linear constraint
    #[inline]
    fn calculate_linear_effective_mass(&self, body_a: &Body, body_b: &Body) -> f32 {
        let inv_mass_a = body_a.inv_mass();
        let inv_mass_b = body_b.inv_mass();

        if inv_mass_a + inv_mass_b <= EPSILON {
            0.0
        } else {
            1.0 / (inv_mass_a + inv_mass_b)
        }
    }

    /// Calculate the effective mass for angular constraint
    #[inline]
    fn calculate_angular_effective_mass(&self, body_a: &Body, body_b: &Body) -> f32 {
        let inv_inertia_a = body_a.inv_inertia();
        let inv_inertia_b = body_b.inv_inertia();

        if inv_inertia_a + inv_inertia_b <= EPSILON {
            0.0
        } else {
            1.0 / (inv_inertia_a + inv_inertia_b)
        }
    }

    /// Calculate relative linear velocity
    #[inline]
    fn calculate_relative_linear_velocity(&self, body_a: &Body, body_b: &Body) -> Vec2 {
        // Get the axis in world space
        let world_axis = body_a.transform().rotation * self.axis;
        
        // Calculate relative velocity along the axis
        let relative_velocity = body_b.linear_velocity() - body_a.linear_velocity();
        
        // Project onto axis
        relative_velocity.dot(world_axis) * world_axis
    }

    /// Calculate relative angular velocity
    #[inline]
    fn calculate_relative_angular_velocity(&self, body_a: &Body, body_b: &Body) -> f32 {
        body_b.angular_velocity() - body_a.angular_velocity()
    }

    /// Calculate linear impulse
    #[inline]
    fn calculate_linear_impulse(
        &self,
        body_a: &Body,
        body_b: &Body,
        velocity_error: Vec2,
    ) -> Vec2 {
        if self.linear_effective_mass <= EPSILON {
            return Vec2::zero();
        }

        // Get the axis in world space
        let world_axis = body_a.transform().rotation * self.axis;
        
        // Project velocity error onto axis
        let error_along_axis = velocity_error.dot(world_axis);
        
        // Calculate impulse magnitude
        let impulse_magnitude = -error_along_axis * self.linear_effective_mass;
        
        // Clamp to maximum force
        let max_force = self.max_force * crate::constants::MIN_POSITIVE;
        let impulse_magnitude = impulse_magnitude.clamp(-max_force, max_force);
        
        // Return impulse vector
        world_axis * impulse_magnitude
    }

    /// Calculate angular impulse
    #[inline]
    fn calculate_angular_impulse(
        &self,
        body_a: &Body,
        body_b: &Body,
        velocity_error: f32,
    ) -> f32 {
        if self.angular_effective_mass <= EPSILON {
            return 0.0;
        }

        // Calculate impulse magnitude
        let impulse = -velocity_error * self.angular_effective_mass;
        
        // Clamp to maximum torque
        let max_torque = self.max_torque * crate::constants::MIN_POSITIVE;
        impulse.clamp(-max_torque, max_torque)
    }

    /// Apply linear impulse
    #[inline]
    fn apply_linear_impulse(&self, body_a: &mut Body, body_b: &mut Body, impulse: Vec2) {
        if !body_a.is_dynamic() && !body_b.is_dynamic() {
            return;
        }

        if body_a.is_dynamic() {
            body_a.add_impulse(-impulse);
        }
        if body_b.is_dynamic() {
            body_b.add_impulse(impulse);
        }
    }

    /// Apply angular impulse
    #[inline]
    fn apply_angular_impulse(&self, body_a: &mut Body, body_b: &mut Body, impulse: f32) {
        if !body_a.is_dynamic() && !body_b.is_dynamic() {
            return;
        }

        if body_a.is_dynamic() {
            body_a.add_angular_impulse(-impulse);
        }
        if body_b.is_dynamic() {
            body_b.add_angular_impulse(impulse);
        }
    }
}

impl fmt::Display for MotorJoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "MotorJoint(axis: {}, max_force: {:.2}, max_torque: {:.2})",
            self.axis, self.max_force, self.max_torque
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::PI;
    use crate::bodies::BodyHandle;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_construction() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let joint = MotorJoint::new(handle, body_a, body_b);

        assert_eq!(joint.handle(), handle);
        assert_eq!(joint.body_a(), body_a);
        assert_eq!(joint.body_b(), body_b);
        assert!(joint.is_enabled());
        assert_eq!(joint.axis(), Vec2::unit_x());
        assert!(approx_eq(joint.max_force(), 0.0));
        assert!(approx_eq(joint.max_torque(), 0.0));
    }

    #[test]
    fn test_with_axis() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let joint = MotorJoint::with_axis(handle, body_a, body_b, Vec2::new(0.0, 1.0));
        assert_eq!(joint.axis(), Vec2::unit_y());
    }

    #[test]
    fn test_setters() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = MotorJoint::new(handle, body_a, body_b);

        joint.set_linear_offset(Vec2::new(1.0, 2.0));
        assert_eq!(joint.linear_offset(), Vec2::new(1.0, 2.0));

        joint.set_angular_offset(PI / 4.0);
        assert!(approx_eq(joint.angular_offset(), PI / 4.0));

        joint.set_axis(Vec2::new(1.0, 1.0));
        assert!(approx_eq(joint.axis().length(), 1.0));

        joint.set_max_force(100.0);
        assert!(approx_eq(joint.max_force(), 100.0));

        joint.set_max_correction_force(50.0);
        assert!(approx_eq(joint.max_correction_force(), 50.0));

        joint.set_max_torque(200.0);
        assert!(approx_eq(joint.max_torque(), 200.0));

        joint.set_max_correction_torque(100.0);
        assert!(approx_eq(joint.max_correction_torque(), 100.0));

        joint.set_linear_stiffness(0.8);
        assert!(approx_eq(joint.linear_stiffness(), 0.8));

        joint.set_angular_stiffness(0.6);
        assert!(approx_eq(joint.angular_stiffness(), 0.6));

        joint.set_linear_damping(0.2);
        assert!(approx_eq(joint.linear_damping(), 0.2));

        joint.set_angular_damping(0.4);
        assert!(approx_eq(joint.angular_damping(), 0.4));

        joint.set_target_linear_velocity(Vec2::new(1.0, 2.0));
        assert_eq!(joint.target_linear_velocity(), Vec2::new(1.0, 2.0));

        joint.set_target_angular_velocity(5.0);
        assert!(approx_eq(joint.target_angular_velocity(), 5.0));

        joint.set_enable_linear_correction(false);
        assert!(!joint.enable_linear_correction());

        joint.set_enable_angular_correction(false);
        assert!(!joint.enable_angular_correction());
    }

    #[test]
    fn test_set_motor_parameters() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = MotorJoint::new(handle, body_a, body_b);

        joint.set_motor_parameters(
            Vec2::new(1.0, 2.0),
            5.0,
            100.0,
            200.0,
        );

        assert_eq!(joint.target_linear_velocity(), Vec2::new(1.0, 2.0));
        assert!(approx_eq(joint.target_angular_velocity(), 5.0));
        assert!(approx_eq(joint.max_force(), 100.0));
        assert!(approx_eq(joint.max_torque(), 200.0));
    }

    #[test]
    fn test_constraint_trait() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = MotorJoint::new(handle, body_a, body_b);

        // Test Constraint trait methods
        assert_eq!(Constraint::handle(&joint), handle);
        assert_eq!(Constraint::body_a(&joint), body_a);
        assert_eq!(Constraint::body_b(&joint), body_b);
        assert!(Constraint::is_enabled(&joint));

        Constraint::set_enabled(&mut joint, false);
        assert!(!Constraint::is_enabled(&joint));
    }

    #[test]
    fn test_display() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let joint = MotorJoint::new(handle, body_a, body_b);

        let joint_str = format!("{}", joint);
        assert!(joint_str.contains("MotorJoint"));
    }

    #[test]
    fn test_axis_normalization() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = MotorJoint::new(handle, body_a, body_b);
        joint.set_axis(Vec2::new(3.0, 4.0));
        
        // Axis should be normalized
        assert!(approx_eq(joint.axis().length(), 1.0));
        assert!(approx_eq(joint.axis().x, 0.6));
        assert!(approx_eq(joint.axis().y, 0.8));
    }
}
