//! Revolute joint constraint implementation.
//!
//! A revolute joint (or hinge joint) connects two bodies at a common point,
//! allowing only relative rotation about that point.

use std::fmt;

use crate::bodies::Body;
use crate::constants::{EPSILON, PI};
use crate::math::{Rot2, Vec2};
use crate::bodies::BodyHandle;
use super::{BaseConstraint, Constraint, ConstraintHandle};

/// A revolute joint constraint.
///
/// This constraint connects two bodies at a common anchor point, allowing
/// only relative rotation about that point. The bodies cannot translate
/// relative to each other, but can rotate freely (or within limits).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct RevoluteJoint {
    /// Base constraint data
    base: BaseConstraint,
    /// Local anchor point on body A
    local_anchor_a: Vec2,
    /// Local anchor point on body B
    local_anchor_b: Vec2,
    /// Lower angle limit (in radians)
    lower_angle: f32,
    /// Upper angle limit (in radians)
    upper_angle: f32,
    /// Whether angle limits are enabled
    limit_enabled: bool,
    /// Maximum motor torque
    max_motor_torque: f32,
    /// Motor speed (in radians per second)
    motor_speed: f32,
    /// Whether the motor is enabled
    motor_enabled: bool,
    /// Accumulated impulse for position correction
    position_impulse: Vec2,
    /// Accumulated impulse for motor
    motor_impulse: f32,
    /// Effective mass for translation
    translation_effective_mass: f32,
    /// Effective mass for rotation
    rotation_effective_mass: f32,
    /// Bias for position correction
    position_bias: Vec2,
    /// Bias for angle correction
    angle_bias: f32,
}

impl RevoluteJoint {
    /// Create a new revolute joint
    #[inline]
    #[must_use]
    pub fn new(
        handle: ConstraintHandle,
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor: Vec2,
    ) -> Self {
        Self {
            base: BaseConstraint::new(handle, body_a, body_b),
            local_anchor_a: anchor,
            local_anchor_b: anchor,
            lower_angle: -PI,
            upper_angle: PI,
            limit_enabled: false,
            max_motor_torque: 0.0,
            motor_speed: 0.0,
            motor_enabled: false,
            position_impulse: Vec2::zero(),
            motor_impulse: 0.0,
            translation_effective_mass: 0.0,
            rotation_effective_mass: 0.0,
            position_bias: Vec2::zero(),
            angle_bias: 0.0,
        }
    }

    /// Create a new revolute joint with anchor points in world space
    #[inline]
    #[must_use]
    pub fn new_at_points(
        handle: ConstraintHandle,
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vec2,
        anchor_b: Vec2,
    ) -> Self {
        // For now, just use the same anchor for both
        // The caller should convert to local space
        Self {
            base: BaseConstraint::new(handle, body_a, body_b),
            local_anchor_a: anchor_a,
            local_anchor_b: anchor_b,
            lower_angle: -PI,
            upper_angle: PI,
            limit_enabled: false,
            max_motor_torque: 0.0,
            motor_speed: 0.0,
            motor_enabled: false,
            position_impulse: Vec2::zero(),
            motor_impulse: 0.0,
            translation_effective_mass: 0.0,
            rotation_effective_mass: 0.0,
            position_bias: Vec2::zero(),
            angle_bias: 0.0,
        }
    }

    /// Get the local anchor point on body A
    #[inline]
    #[must_use]
    pub const fn local_anchor_a(&self) -> Vec2 {
        self.local_anchor_a
    }

    /// Set the local anchor point on body A
    #[inline]
    pub fn set_local_anchor_a(&mut self, anchor: Vec2) {
        self.local_anchor_a = anchor;
    }

    /// Get the local anchor point on body B
    #[inline]
    #[must_use]
    pub const fn local_anchor_b(&self) -> Vec2 {
        self.local_anchor_b
    }

    /// Set the local anchor point on body B
    #[inline]
    pub fn set_local_anchor_b(&mut self, anchor: Vec2) {
        self.local_anchor_b = anchor;
    }

    /// Get the lower angle limit
    #[inline]
    #[must_use]
    pub const fn lower_angle(&self) -> f32 {
        self.lower_angle
    }

    /// Set the lower angle limit
    #[inline]
    pub fn set_lower_angle(&mut self, angle: f32) {
        self.lower_angle = angle;
    }

    /// Get the upper angle limit
    #[inline]
    #[must_use]
    pub const fn upper_angle(&self) -> f32 {
        self.upper_angle
    }

    /// Set the upper angle limit
    #[inline]
    pub fn set_upper_angle(&mut self, angle: f32) {
        self.upper_angle = angle;
    }

    /// Check if angle limits are enabled
    #[inline]
    #[must_use]
    pub const fn limit_enabled(&self) -> bool {
        self.limit_enabled
    }

    /// Enable or disable angle limits
    #[inline]
    pub fn set_limit_enabled(&mut self, enabled: bool) {
        self.limit_enabled = enabled;
    }

    /// Enable angle limits with the given range
    #[inline]
    pub fn enable_limits(&mut self, lower: f32, upper: f32) {
        self.lower_angle = lower;
        self.upper_angle = upper;
        self.limit_enabled = true;
    }

    /// Disable angle limits
    #[inline]
    pub fn disable_limits(&mut self) {
        self.limit_enabled = false;
    }

    /// Get the maximum motor torque
    #[inline]
    #[must_use]
    pub const fn max_motor_torque(&self) -> f32 {
        self.max_motor_torque
    }

    /// Set the maximum motor torque
    #[inline]
    pub fn set_max_motor_torque(&mut self, torque: f32) {
        self.max_motor_torque = torque.max(0.0);
    }

    /// Get the motor speed
    #[inline]
    #[must_use]
    pub const fn motor_speed(&self) -> f32 {
        self.motor_speed
    }

    /// Set the motor speed
    #[inline]
    pub fn set_motor_speed(&mut self, speed: f32) {
        self.motor_speed = speed;
    }

    /// Check if the motor is enabled
    #[inline]
    #[must_use]
    pub const fn motor_enabled(&self) -> bool {
        self.motor_enabled
    }

    /// Enable or disable the motor
    #[inline]
    pub fn set_motor_enabled(&mut self, enabled: bool) {
        self.motor_enabled = enabled;
    }

    /// Enable the motor with the given speed and torque
    #[inline]
    pub fn enable_motor(&mut self, speed: f32, max_torque: f32) {
        self.motor_speed = speed;
        self.max_motor_torque = max_torque;
        self.motor_enabled = true;
    }

    /// Disable the motor
    #[inline]
    pub fn disable_motor(&mut self) {
        self.motor_enabled = false;
        self.motor_speed = 0.0;
        self.motor_impulse = 0.0;
    }

    /// Get the current angle between bodies
    #[inline]
    #[must_use]
    pub fn current_angle(&self, body_a: &Body, body_b: &Body) -> f32 {
        let angle_a = body_a.rotation().angle();
        let angle_b = body_b.rotation().angle();
        angle_b - angle_a
    }

    /// Get the world anchor point (should be the same for both bodies)
    #[inline]
    #[must_use]
    pub fn world_anchor(&self, body_a: &Body, body_b: &Body) -> Vec2 {
        // The world anchor should be the same for both bodies if the constraint is satisfied
        // For now, return the anchor from body A
        body_a.transform().transform_point(self.local_anchor_a)
    }

    /// Check if the joint is at its lower limit
    #[inline]
    #[must_use]
    pub fn is_at_lower_limit(&self, body_a: &Body, body_b: &Body) -> bool {
        if !self.limit_enabled {
            return false;
        }
        let angle = self.current_angle(body_a, body_b);
        angle <= self.lower_angle + EPSILON
    }

    /// Check if the joint is at its upper limit
    #[inline]
    #[must_use]
    pub fn is_at_upper_limit(&self, body_a: &Body, body_b: &Body) -> bool {
        if !self.limit_enabled {
            return false;
        }
        let angle = self.current_angle(body_a, body_b);
        angle >= self.upper_angle - EPSILON
    }

    /// Check if the joint is at any limit
    #[inline]
    #[must_use]
    pub fn is_at_limit(&self, body_a: &Body, body_b: &Body) -> bool {
        self.is_at_lower_limit(body_a, body_b) || self.is_at_upper_limit(body_a, body_b)
    }
}

impl Constraint for RevoluteJoint {
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
            self.position_impulse = Vec2::zero();
            self.motor_impulse = 0.0;
            self.translation_effective_mass = 0.0;
            self.rotation_effective_mass = 0.0;
            self.position_bias = Vec2::zero();
            self.angle_bias = 0.0;
            return;
        }

        // Get world anchors
        let anchor_a = body_a.transform().transform_point(self.local_anchor_a);
        let anchor_b = body_b.transform().transform_point(self.local_anchor_b);

        // Calculate relative positions
        let ra = anchor_a - body_a.position();
        let rb = anchor_b - body_b.position();

        // Calculate effective mass for translation constraint
        self.translation_effective_mass = self.calculate_translation_effective_mass(
            body_a, body_b, ra, rb
        );

        // Calculate effective mass for rotation constraint
        self.rotation_effective_mass = self.calculate_rotation_effective_mass(
            body_a, body_b
        );

        // Calculate position bias
        let position_error = anchor_b - anchor_a;
        self.position_bias = self.calculate_position_bias(position_error, dt);

        // Calculate angle bias
        if self.limit_enabled {
            let angle = self.current_angle(body_a, body_b);
            let angle_error = self.clamp_angle(angle) - angle;
            self.angle_bias = self.calculate_angle_bias(angle_error, dt);
        } else {
            self.angle_bias = 0.0;
        }
    }

    fn solve(&mut self, body_a: &mut Body, body_b: &mut Body) {
        if !self.base.is_enabled() {
            return;
        }

        // Get world anchors
        let anchor_a = body_a.transform().transform_point(self.local_anchor_a);
        let anchor_b = body_b.transform().transform_point(self.local_anchor_b);

        // Solve position constraint (prevent translation)
        if self.translation_effective_mass > EPSILON {
            let position_error = anchor_b - anchor_a;
            let position_impulse = self.solve_position_constraint(
                body_a, body_b, anchor_a, anchor_b, position_error
            );
            self.position_impulse += position_impulse;
        }

        // Solve rotation constraint (angle limits)
        if self.limit_enabled && self.rotation_effective_mass > EPSILON {
            let angle = self.current_angle(body_a, body_b);
            let angle_error = self.clamp_angle(angle) - angle;
            let angle_impulse = self.solve_angle_constraint(body_a, body_b, angle_error);
            self.motor_impulse += angle_impulse;
        }

        // Solve motor constraint
        if self.motor_enabled && self.max_motor_torque > EPSILON {
            let motor_impulse = self.solve_motor_constraint(body_a, body_b);
            self.motor_impulse += motor_impulse;
        }
    }

    fn post_solve(&mut self) {
        // Clear accumulated impulses or keep for warm starting
        self.position_impulse = Vec2::zero();
        self.motor_impulse = 0.0;
    }

    fn reaction_force(&self) -> Vec2 {
        self.position_impulse
    }

    fn reaction_torque(&self) -> f32 {
        self.motor_impulse
    }
}

impl RevoluteJoint {
    /// Calculate the effective mass for translation constraint
    #[inline]
    fn calculate_translation_effective_mass(
        &self,
        body_a: &Body,
        body_b: &Body,
        ra: Vec2,
        rb: Vec2,
    ) -> f32 {
        let inv_mass_a = body_a.inv_mass();
        let inv_mass_b = body_b.inv_mass();
        let inv_inertia_a = body_a.inv_inertia();
        let inv_inertia_b = body_b.inv_inertia();

        // For revolute joint, we need to constrain both x and y translation
        // We use a combined effective mass
        let ra_cross_x = ra.cross(Vec2::unit_x());
        let ra_cross_y = ra.cross(Vec2::unit_y());
        let rb_cross_x = rb.cross(Vec2::unit_x());
        let rb_cross_y = rb.cross(Vec2::unit_y());

        // Effective mass for x-axis
        let eff_mass_x = 1.0 / (inv_mass_a + inv_mass_b
            + ra_cross_x * ra_cross_x * inv_inertia_a
            + rb_cross_x * rb_cross_x * inv_inertia_b);

        // Effective mass for y-axis
        let eff_mass_y = 1.0 / (inv_mass_a + inv_mass_b
            + ra_cross_y * ra_cross_y * inv_inertia_a
            + rb_cross_y * rb_cross_y * inv_inertia_b);

        // Use the minimum for stability
        eff_mass_x.min(eff_mass_y)
    }

    /// Calculate the effective mass for rotation constraint
    #[inline]
    fn calculate_rotation_effective_mass(&self, body_a: &Body, body_b: &Body) -> f32 {
        let inv_inertia_a = body_a.inv_inertia();
        let inv_inertia_b = body_b.inv_inertia();

        if inv_inertia_a + inv_inertia_b <= EPSILON {
            0.0
        } else {
            1.0 / (inv_inertia_a + inv_inertia_b)
        }
    }

    /// Calculate position bias for Baumgarte stabilization
    #[inline]
    fn calculate_position_bias(&self, error: Vec2, dt: f32) -> Vec2 {
        // Baumgarte factor
        const BAUMGARTE_FACTOR: f32 = 0.2;
        error * (BAUMGARTE_FACTOR / dt)
    }

    /// Calculate angle bias for Baumgarte stabilization
    #[inline]
    fn calculate_angle_bias(&self, error: f32, dt: f32) -> f32 {
        // Baumgarte factor
        const BAUMGARTE_FACTOR: f32 = 0.2;
        error * (BAUMGARTE_FACTOR / dt)
    }

    /// Clamp angle to be within limits
    #[inline]
    fn clamp_angle(&self, angle: f32) -> f32 {
        angle.clamp(self.lower_angle, self.upper_angle)
    }

    /// Solve position constraint
    #[inline]
    fn solve_position_constraint(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        anchor_a: Vec2,
        anchor_b: Vec2,
        position_error: Vec2,
    ) -> Vec2 {
        if self.translation_effective_mass <= EPSILON {
            return Vec2::zero();
        }

        // Calculate impulse
        let impulse = -position_error * self.translation_effective_mass;

        // Apply impulse
        if body_a.is_dynamic() {
            body_a.add_impulse_at_point(-impulse, anchor_a);
        }
        if body_b.is_dynamic() {
            body_b.add_impulse_at_point(impulse, anchor_b);
        }

        impulse
    }

    /// Solve angle constraint
    #[inline]
    fn solve_angle_constraint(&self, body_a: &mut Body, body_b: &mut Body, angle_error: f32) -> f32 {
        if self.rotation_effective_mass <= EPSILON {
            return 0.0;
        }

        // Calculate impulse
        let impulse = -angle_error * self.rotation_effective_mass;

        // Apply impulse
        if body_a.is_dynamic() {
            body_a.add_angular_impulse(-impulse);
        }
        if body_b.is_dynamic() {
            body_b.add_angular_impulse(impulse);
        }

        impulse
    }

    /// Solve motor constraint
    #[inline]
    fn solve_motor_constraint(&self, body_a: &mut Body, body_b: &mut Body) -> f32 {
        if self.rotation_effective_mass <= EPSILON {
            return 0.0;
        }

        // Calculate current relative angular velocity
        let relative_velocity = body_b.angular_velocity() - body_a.angular_velocity();

        // Calculate desired velocity (motor speed)
        let desired_velocity = self.motor_speed;

        // Calculate velocity error
        let velocity_error = desired_velocity - relative_velocity;

        // Calculate impulse
        let impulse = velocity_error * self.rotation_effective_mass;

        // Clamp to maximum torque
        let max_impulse = self.max_motor_torque * crate::constants::MIN_POSITIVE;
        let impulse = impulse.clamp(-max_impulse, max_impulse);

        // Apply impulse
        if body_a.is_dynamic() {
            body_a.add_angular_impulse(-impulse);
        }
        if body_b.is_dynamic() {
            body_b.add_angular_impulse(impulse);
        }

        impulse
    }
}

impl fmt::Display for RevoluteJoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "RevoluteJoint(anchor_a: {}, anchor_b: {}, limits: [{:.2}, {:.2}], motor: {})",
            self.local_anchor_a,
            self.local_anchor_b,
            self.lower_angle,
            self.upper_angle,
            self.motor_enabled
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_construction() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);
        let anchor = Vec2::new(1.0, 2.0);

        let joint = RevoluteJoint::new(handle, body_a, body_b, anchor);

        assert_eq!(joint.handle(), handle);
        assert_eq!(joint.body_a(), body_a);
        assert_eq!(joint.body_b(), body_b);
        assert_eq!(joint.local_anchor_a(), anchor);
        assert_eq!(joint.local_anchor_b(), anchor);
        assert!(joint.is_enabled());
        assert!(!joint.limit_enabled());
        assert!(!joint.motor_enabled());
    }

    #[test]
    fn test_with_anchors() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let joint = RevoluteJoint::new_at_points(
            handle, body_a, body_b,
            Vec2::new(1.0, 0.0),
            Vec2::new(-1.0, 0.0),
        );

        assert_eq!(joint.local_anchor_a(), Vec2::new(1.0, 0.0));
        assert_eq!(joint.local_anchor_b(), Vec2::new(-1.0, 0.0));
    }

    #[test]
    fn test_angle_limits() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = RevoluteJoint::new(handle, body_a, body_b, Vec2::zero());

        assert!(!joint.limit_enabled());

        joint.set_lower_angle(-PI / 2.0);
        joint.set_upper_angle(PI / 2.0);
        assert!(approx_eq(joint.lower_angle(), -PI / 2.0));
        assert!(approx_eq(joint.upper_angle(), PI / 2.0));

        joint.set_limit_enabled(true);
        assert!(joint.limit_enabled());

        joint.enable_limits(-PI / 4.0, PI / 4.0);
        assert!(approx_eq(joint.lower_angle(), -PI / 4.0));
        assert!(approx_eq(joint.upper_angle(), PI / 4.0));
        assert!(joint.limit_enabled());

        joint.disable_limits();
        assert!(!joint.limit_enabled());
    }

    #[test]
    fn test_motor() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = RevoluteJoint::new(handle, body_a, body_b, Vec2::zero());

        assert!(!joint.motor_enabled());
        assert!(approx_eq(joint.motor_speed(), 0.0));
        assert!(approx_eq(joint.max_motor_torque(), 0.0));

        joint.set_max_motor_torque(100.0);
        assert!(approx_eq(joint.max_motor_torque(), 100.0));

        joint.set_motor_speed(2.0);
        assert!(approx_eq(joint.motor_speed(), 2.0));

        joint.set_motor_enabled(true);
        assert!(joint.motor_enabled());

        joint.enable_motor(5.0, 200.0);
        assert!(joint.motor_enabled());
        assert!(approx_eq(joint.motor_speed(), 5.0));
        assert!(approx_eq(joint.max_motor_torque(), 200.0));

        joint.disable_motor();
        assert!(!joint.motor_enabled());
        assert!(approx_eq(joint.motor_speed(), 0.0));
    }

    #[test]
    fn test_current_angle() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut body_a_obj = Body::new_dynamic(body_a);
        body_a_obj.set_rotation(Rot2::new(PI / 4.0));

        let mut body_b_obj = Body::new_dynamic(body_b);
        body_b_obj.set_rotation(Rot2::new(3.0 * PI / 4.0));

        let joint = RevoluteJoint::new(handle, body_a, body_b, Vec2::zero());

        let angle = joint.current_angle(&body_a_obj, &body_b_obj);
        assert!(approx_eq(angle, PI / 2.0));
    }

    #[test]
    fn test_world_anchor() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut body_a_obj = Body::new_dynamic(body_a);
        body_a_obj.set_position(Vec2::new(10.0, 20.0));

        let mut body_b_obj = Body::new_dynamic(body_b);
        body_b_obj.set_position(Vec2::new(30.0, 40.0));

        let joint = RevoluteJoint::new(handle, body_a, body_b, Vec2::new(1.0, 2.0));

        let anchor = joint.world_anchor(&body_a_obj, &body_b_obj);
        assert_eq!(anchor, Vec2::new(11.0, 22.0));
    }

    #[test]
    fn test_is_at_limits() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut body_a_obj = Body::new_dynamic(body_a);
        let mut body_b_obj = Body::new_dynamic(body_b);

        let mut joint = RevoluteJoint::new(handle, body_a, body_b, Vec2::zero());

        // Without limits enabled, should never be at limit
        assert!(!joint.is_at_lower_limit(&body_a_obj, &body_b_obj));
        assert!(!joint.is_at_upper_limit(&body_a_obj, &body_b_obj));
        assert!(!joint.is_at_limit(&body_a_obj, &body_b_obj));

        // Enable limits
        joint.enable_limits(-PI / 4.0, PI / 4.0);

        // At zero angle, not at limits
        assert!(!joint.is_at_lower_limit(&body_a_obj, &body_b_obj));
        assert!(!joint.is_at_upper_limit(&body_a_obj, &body_b_obj));

        // At lower limit
        body_b_obj.set_rotation(Rot2::new(-PI / 4.0));
        assert!(joint.is_at_lower_limit(&body_a_obj, &body_b_obj));
        assert!(joint.is_at_limit(&body_a_obj, &body_b_obj));

        // At upper limit
        body_b_obj.set_rotation(Rot2::new(PI / 4.0));
        assert!(joint.is_at_upper_limit(&body_a_obj, &body_b_obj));
        assert!(joint.is_at_limit(&body_a_obj, &body_b_obj));
    }

    #[test]
    fn test_constraint_trait() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = RevoluteJoint::new(handle, body_a, body_b, Vec2::zero());

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

        let joint = RevoluteJoint::new(handle, body_a, body_b, Vec2::new(1.0, 2.0));

        let joint_str = format!("{}", joint);
        assert!(joint_str.contains("RevoluteJoint"));
    }
}
