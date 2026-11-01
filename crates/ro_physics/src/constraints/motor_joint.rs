//! Motor joint constraint implementation.
//!
//! A motor joint provides controlled relative motion between two bodies.
//! It can control linear and/or angular motion with configurable limits.

use std::fmt;

use super::{BaseConstraint, Constraint, ConstraintHandle, ConstraintType};
use crate::bodies::Body;
use crate::bodies::BodyHandle;
use crate::constants::{EPSILON, MAX_CORRECTION_SPEED};
use crate::math::Vec2;

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
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
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
    /// Correction velocity along the motor axis (metres per second)
    #[cfg_attr(feature = "serialization", serde(default))]
    linear_bias: f32,
    /// Correction angular velocity (radians per second)
    #[cfg_attr(feature = "serialization", serde(default))]
    angular_bias: f32,
    /// Timestep of the current solve, used to convert forces into impulses
    #[cfg_attr(feature = "serialization", serde(default))]
    current_dt: f32,
}

impl MotorJoint {
    /// Create a new motor joint
    #[inline]
    #[must_use]
    pub fn new(handle: ConstraintHandle, body_a: BodyHandle, body_b: BodyHandle) -> Self {
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
            linear_bias: 0.0,
            angular_bias: 0.0,
            current_dt: 0.0,
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
    fn kind(&self) -> ConstraintType {
        ConstraintType::Motor
    }

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

        self.current_dt = dt.max(0.0);

        // Calculate effective masses
        self.linear_effective_mass = self.calculate_linear_effective_mass(body_a, body_b);
        self.angular_effective_mass = self.calculate_angular_effective_mass(body_a, body_b);

        // Baumgarte correction: turn the remaining position error into a
        // target velocity so it is solved together with the motor targets.
        self.linear_bias = 0.0;
        self.angular_bias = 0.0;

        if dt > EPSILON && self.enable_linear_correction {
            let world_axis = body_a.transform().rotation * self.axis;
            let target_offset = body_a.transform().rotation * self.linear_offset;
            let error = (body_b.position() - body_a.position() - target_offset).dot(world_axis);
            self.linear_bias = self.correction_velocity(
                error,
                dt,
                self.linear_effective_mass,
                self.max_correction_force,
            );
        }

        if dt > EPSILON && self.enable_angular_correction {
            let error =
                (body_b.rotation().angle() - body_a.rotation().angle()) - self.angular_offset;
            self.angular_bias = self.correction_velocity(
                error,
                dt,
                self.angular_effective_mass,
                self.max_correction_torque,
            );
        }
    }

    fn solve(&mut self, body_a: &mut Body, body_b: &mut Body) {
        if !self.base.is_enabled() {
            return;
        }

        // Solve linear constraint
        if self.linear_effective_mass > EPSILON {
            let world_axis = body_a.transform().rotation * self.axis;
            let relative_velocity = self
                .calculate_relative_linear_velocity(body_a, body_b)
                .dot(world_axis);
            let target_velocity = self.target_linear_velocity.dot(world_axis) + self.linear_bias;

            let mut impulse = (target_velocity - relative_velocity) * self.linear_effective_mass;

            // Damping resists the current relative motion
            if self.linear_damping > EPSILON {
                impulse -= self.linear_damping * self.linear_effective_mass * relative_velocity;
            }

            // Soft constraint
            if self.linear_stiffness < 1.0 - EPSILON {
                impulse *= self.linear_stiffness;
            }

            if self.max_force > EPSILON {
                let max_impulse = self.max_force * self.current_dt;
                impulse = impulse.clamp(-max_impulse, max_impulse);
            }

            // Apply the impulse
            self.apply_linear_impulse(body_a, body_b, world_axis * impulse);
            self.linear_impulse += world_axis * impulse;
        }

        // Solve angular constraint
        if self.angular_effective_mass > EPSILON {
            let relative_velocity = self.calculate_relative_angular_velocity(body_a, body_b);
            let target_velocity = self.target_angular_velocity + self.angular_bias;

            let mut impulse = (target_velocity - relative_velocity) * self.angular_effective_mass;

            // Damping resists the current relative motion
            if self.angular_damping > EPSILON {
                impulse -= self.angular_damping * self.angular_effective_mass * relative_velocity;
            }

            // Soft constraint
            if self.angular_stiffness < 1.0 - EPSILON {
                impulse *= self.angular_stiffness;
            }

            if self.max_torque > EPSILON {
                let max_impulse = self.max_torque * self.current_dt;
                impulse = impulse.clamp(-max_impulse, max_impulse);
            }

            // Apply the impulse
            self.apply_angular_impulse(body_a, body_b, impulse);
            self.angular_impulse += impulse;
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

    /// Convert a position error into a Baumgarte correction velocity
    ///
    /// The result is clamped so the impulse needed to apply it stays below
    /// `max_force` newtons (or newton-metres for the angular case).
    #[inline]
    fn correction_velocity(&self, error: f32, dt: f32, effective_mass: f32, max_force: f32) -> f32 {
        // Baumgarte factor
        const BAUMGARTE_FACTOR: f32 = 0.2;

        let velocity = (-BAUMGARTE_FACTOR * error / dt)
            .clamp(-MAX_CORRECTION_SPEED, MAX_CORRECTION_SPEED);

        if max_force > EPSILON && effective_mass > EPSILON {
            let max_velocity = max_force * dt / effective_mass;
            return velocity.clamp(-max_velocity, max_velocity);
        }

        velocity
    }

    /// Apply linear impulse
    #[inline]
    fn apply_linear_impulse(&self, body_a: &mut Body, body_b: &mut Body, impulse: Vec2) {
        if !body_a.is_dynamic() && !body_b.is_dynamic() {
            return;
        }

        if body_a.is_dynamic() {
            body_a.add_impulse_no_wake(-impulse);
        }
        if body_b.is_dynamic() {
            body_b.add_impulse_no_wake(impulse);
        }
    }

    /// Apply angular impulse
    #[inline]
    fn apply_angular_impulse(&self, body_a: &mut Body, body_b: &mut Body, impulse: f32) {
        if !body_a.is_dynamic() && !body_b.is_dynamic() {
            return;
        }

        if body_a.is_dynamic() {
            body_a.add_angular_impulse_no_wake(-impulse);
        }
        if body_b.is_dynamic() {
            body_b.add_angular_impulse_no_wake(impulse);
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
    use crate::bodies::BodyHandle;
    use crate::constants::PI;
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

        joint.set_motor_parameters(Vec2::new(1.0, 2.0), 5.0, 100.0, 200.0);

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
