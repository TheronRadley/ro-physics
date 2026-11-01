//! Distance joint constraint implementation.
//!
//! A distance joint connects two points on two bodies, maintaining a fixed
//! distance between them. This is also known as a point-to-point constraint.

use std::fmt;

use crate::bodies::Body;
use crate::constants::EPSILON;
use crate::math::Vec2;
use crate::bodies::BodyHandle;
use super::{BaseConstraint, Constraint, ConstraintHandle};

/// A distance joint constraint.
///
/// This constraint maintains a fixed distance between two anchor points
/// on two bodies. It can be configured to be rigid (exact distance) or
/// soft (with limits).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct DistanceJoint {
    /// Base constraint data
    base: BaseConstraint,
    /// Local anchor point on body A
    local_anchor_a: Vec2,
    /// Local anchor point on body B
    local_anchor_b: Vec2,
    /// Target distance between anchors
    length: f32,
    /// Minimum distance (for soft constraints)
    min_length: f32,
    /// Maximum distance (for soft constraints)
    max_length: f32,
    /// Stiffness of the constraint (0-1)
    stiffness: f32,
    /// Damping of the constraint (0-1)
    damping: f32,
    /// Accumulated impulse
    impulse: Vec2,
    /// Effective mass
    effective_mass: f32,
    /// Bias for position correction
    bias: f32,
}

impl DistanceJoint {
    /// Create a new distance joint
    #[inline]
    #[must_use]
    pub fn new(
        handle: ConstraintHandle,
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vec2,
        anchor_b: Vec2,
    ) -> Self {
        let delta = anchor_b - anchor_a;
        let length = delta.length();

        Self {
            base: BaseConstraint::new(handle, body_a, body_b),
            local_anchor_a: anchor_a,
            local_anchor_b: anchor_b,
            length,
            min_length: length,
            max_length: length,
            stiffness: 1.0,
            damping: 0.0,
            impulse: Vec2::zero(),
            effective_mass: 0.0,
            bias: 0.0,
        }
    }

    /// Create a new distance joint with a specific length
    #[inline]
    #[must_use]
    pub fn with_length(
        handle: ConstraintHandle,
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vec2,
        anchor_b: Vec2,
        length: f32,
    ) -> Self {
        let mut joint = Self::new(handle, body_a, body_b, anchor_a, anchor_b);
        joint.length = length;
        joint.min_length = length;
        joint.max_length = length;
        joint
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

    /// Get the target length
    #[inline]
    #[must_use]
    pub const fn length(&self) -> f32 {
        self.length
    }

    /// Set the target length
    #[inline]
    pub fn set_length(&mut self, length: f32) {
        self.length = length.max(0.0);
        self.min_length = self.length;
        self.max_length = self.length;
    }

    /// Get the minimum length
    #[inline]
    #[must_use]
    pub const fn min_length(&self) -> f32 {
        self.min_length
    }

    /// Set the minimum length
    #[inline]
    pub fn set_min_length(&mut self, min_length: f32) {
        self.min_length = min_length.max(0.0);
    }

    /// Get the maximum length
    #[inline]
    #[must_use]
    pub const fn max_length(&self) -> f32 {
        self.max_length
    }

    /// Set the maximum length
    #[inline]
    pub fn set_max_length(&mut self, max_length: f32) {
        self.max_length = max_length.max(0.0);
    }

    /// Get the stiffness
    #[inline]
    #[must_use]
    pub const fn stiffness(&self) -> f32 {
        self.stiffness
    }

    /// Set the stiffness
    #[inline]
    pub fn set_stiffness(&mut self, stiffness: f32) {
        self.stiffness = stiffness.clamp(0.0, 1.0);
    }

    /// Get the damping
    #[inline]
    #[must_use]
    pub const fn damping(&self) -> f32 {
        self.damping
    }

    /// Set the damping
    #[inline]
    pub fn set_damping(&mut self, damping: f32) {
        self.damping = damping.clamp(0.0, 1.0);
    }

    /// Get the world anchor point on body A
    #[inline]
    #[must_use]
    pub fn world_anchor_a(&self, body_a: &Body) -> Vec2 {
        body_a.transform().transform_point(self.local_anchor_a)
    }

    /// Get the world anchor point on body B
    #[inline]
    #[must_use]
    pub fn world_anchor_b(&self, body_b: &Body) -> Vec2 {
        body_b.transform().transform_point(self.local_anchor_b)
    }

    /// Get the current distance between anchors
    #[inline]
    #[must_use]
    pub fn current_distance(&self, body_a: &Body, body_b: &Body) -> f32 {
        let anchor_a = self.world_anchor_a(body_a);
        let anchor_b = self.world_anchor_b(body_b);
        (anchor_b - anchor_a).length()
    }

    /// Check if the constraint is at its limits
    #[inline]
    #[must_use]
    pub fn is_at_limits(&self, body_a: &Body, body_b: &Body) -> bool {
        let distance = self.current_distance(body_a, body_b);
        distance <= self.min_length + EPSILON || distance >= self.max_length - EPSILON
    }
}

impl Constraint for DistanceJoint {
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
            self.impulse = Vec2::zero();
            self.effective_mass = 0.0;
            self.bias = 0.0;
            return;
        }

        // Get world anchors
        let anchor_a = self.world_anchor_a(body_a);
        let anchor_b = self.world_anchor_b(body_b);

        // Calculate direction from A to B
        let delta = anchor_b - anchor_a;
        let distance = delta.length();

        // Handle zero distance case
        let direction = if distance > EPSILON {
            delta / distance
        } else {
            Vec2::unit_x()
        };

        // Calculate relative positions from body centers
        let ra = anchor_a - body_a.position();
        let rb = anchor_b - body_b.position();

        // Calculate effective mass
        self.effective_mass = self.calculate_effective_mass(body_a, body_b, direction, ra, rb);

        if self.effective_mass > EPSILON {
            // A rigid joint (min == max == length) always drives towards
            // `length`; a joint with a soft range only acts outside of it.
            let soft_range = (self.max_length - self.min_length) > EPSILON;
            let error = if distance < self.min_length {
                distance - self.min_length
            } else if distance > self.max_length {
                distance - self.max_length
            } else if soft_range {
                // Inside the allowed range: the joint is inactive this step.
                self.effective_mass = 0.0;
                self.bias = 0.0;
                return;
            } else {
                distance - self.length
            };

            self.bias = self.calculate_bias(error, dt);
        } else {
            self.bias = 0.0;
        }
    }

    fn solve(&mut self, body_a: &mut Body, body_b: &mut Body) {
        if !self.base.is_enabled() || self.effective_mass <= EPSILON {
            return;
        }

        // Get world anchors
        let anchor_a = self.world_anchor_a(body_a);
        let anchor_b = self.world_anchor_b(body_b);

        // Calculate direction from A to B
        let delta = anchor_b - anchor_a;
        let distance = delta.length();

        let direction = if distance > EPSILON {
            delta / distance
        } else {
            Vec2::unit_x()
        };

        // Separation velocity along the constraint axis (positive means the
        // anchors are moving apart)
        let relative_velocity = self.calculate_relative_velocity(body_a, body_b, direction);

        // Calculate the constraint impulse
        let mut impulse = self.calculate_impulse(relative_velocity);

        // Damping removes energy along the constraint axis
        if self.damping > EPSILON {
            impulse -= self.damping * self.effective_mass * relative_velocity;
        }

        // Apply stiffness (soft constraint)
        if self.stiffness < 1.0 - EPSILON {
            impulse *= self.stiffness;
        }

        // Apply the impulse
        self.apply_impulse(body_a, body_b, direction, impulse, anchor_a, anchor_b);

        // Accumulate impulse for warm starting
        self.impulse += direction * impulse;
    }

    fn post_solve(&mut self) {
        // Clear accumulated impulse or keep it for warm starting
        // For now, we clear it
        self.impulse = Vec2::zero();
    }

    fn reaction_force(&self) -> Vec2 {
        self.impulse
    }

    fn reaction_torque(&self) -> f32 {
        0.0
    }
}

impl DistanceJoint {
    /// Calculate the effective mass for the constraint
    #[inline]
    fn calculate_effective_mass(
        &self,
        body_a: &Body,
        body_b: &Body,
        direction: Vec2,
        ra: Vec2,
        rb: Vec2,
    ) -> f32 {
        let inv_mass_a = body_a.inv_mass();
        let inv_mass_b = body_b.inv_mass();
        let inv_inertia_a = body_a.inv_inertia();
        let inv_inertia_b = body_b.inv_inertia();

        let ra_cross_dir = ra.cross(direction);
        let rb_cross_dir = rb.cross(direction);

        let denominator = inv_mass_a + inv_mass_b
            + ra_cross_dir * ra_cross_dir * inv_inertia_a
            + rb_cross_dir * rb_cross_dir * inv_inertia_b;

        if denominator <= EPSILON {
            0.0
        } else {
            1.0 / denominator
        }
    }

    /// Calculate the bias for Baumgarte stabilization
    #[inline]
    fn calculate_bias(&self, error: f32, dt: f32) -> f32 {
        // Baumgarte factor
        const BAUMGARTE_FACTOR: f32 = 0.2;

        if dt <= EPSILON {
            return 0.0;
        }

        BAUMGARTE_FACTOR * error / dt
    }

    /// Calculate the separation velocity along an axis
    ///
    /// Positive values mean the two anchors are moving apart.
    #[inline]
    fn calculate_relative_velocity(&self, body_a: &Body, body_b: &Body, axis: Vec2) -> f32 {
        let va = body_a.velocity_at_point(self.world_anchor_a(body_a));
        let vb = body_b.velocity_at_point(self.world_anchor_b(body_b));
        (vb - va).dot(axis)
    }

    /// Calculate the impulse to apply
    ///
    /// `relative_velocity` is the separation velocity along the constraint
    /// axis, so the impulse has to cancel both that velocity and the
    /// Baumgarte bias.
    #[inline]
    fn calculate_impulse(&self, relative_velocity: f32) -> f32 {
        if self.effective_mass <= EPSILON {
            return 0.0;
        }

        -(relative_velocity + self.bias) * self.effective_mass
    }

    /// Apply impulse to bodies
    #[inline]
    fn apply_impulse(
        &self,
        body_a: &mut Body,
        body_b: &mut Body,
        direction: Vec2,
        impulse: f32,
        anchor_a: Vec2,
        anchor_b: Vec2,
    ) {
        if !body_a.is_dynamic() && !body_b.is_dynamic() {
            return;
        }

        // Apply to body A
        if body_a.is_dynamic() {
            body_a.add_impulse_at_point_no_wake(-direction * impulse, anchor_a);
        }

        // Apply to body B
        if body_b.is_dynamic() {
            body_b.add_impulse_at_point_no_wake(direction * impulse, anchor_b);
        }
    }
}

impl fmt::Display for DistanceJoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "DistanceJoint(anchor_a: {}, anchor_b: {}, length: {:.2})",
            self.local_anchor_a, self.local_anchor_b, self.length
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
        let anchor_a = Vec2::new(1.0, 0.0);
        let anchor_b = Vec2::new(-1.0, 0.0);

        let joint = DistanceJoint::new(handle, body_a, body_b, anchor_a, anchor_b);

        assert_eq!(joint.handle(), handle);
        assert_eq!(joint.body_a(), body_a);
        assert_eq!(joint.body_b(), body_b);
        assert_eq!(joint.local_anchor_a(), anchor_a);
        assert_eq!(joint.local_anchor_b(), anchor_b);
        assert!(approx_eq(joint.length(), 2.0));
        assert!(joint.is_enabled());
    }

    #[test]
    fn test_with_length() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);
        let anchor_a = Vec2::new(0.0, 0.0);
        let anchor_b = Vec2::new(0.0, 0.0);

        let joint = DistanceJoint::with_length(handle, body_a, body_b, anchor_a, anchor_b, 5.0);
        assert!(approx_eq(joint.length(), 5.0));
    }

    #[test]
    fn test_world_anchors() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut body_a_obj = Body::new_dynamic(body_a);
        body_a_obj.set_position(Vec2::new(10.0, 20.0));

        let mut body_b_obj = Body::new_dynamic(body_b);
        body_b_obj.set_position(Vec2::new(30.0, 40.0));

        let joint = DistanceJoint::new(
            handle, body_a, body_b,
            Vec2::new(1.0, 2.0),
            Vec2::new(3.0, 4.0),
        );

        let anchor_a = joint.world_anchor_a(&body_a_obj);
        let anchor_b = joint.world_anchor_b(&body_b_obj);

        assert_eq!(anchor_a, Vec2::new(11.0, 22.0));
        assert_eq!(anchor_b, Vec2::new(33.0, 44.0));
    }

    #[test]
    fn test_current_distance() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut body_a_obj = Body::new_dynamic(body_a);
        body_a_obj.set_position(Vec2::new(0.0, 0.0));

        let mut body_b_obj = Body::new_dynamic(body_b);
        body_b_obj.set_position(Vec2::new(3.0, 4.0));

        let joint = DistanceJoint::new(
            handle, body_a, body_b,
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 0.0),
        );

        let distance = joint.current_distance(&body_a_obj, &body_b_obj);
        assert!(approx_eq(distance, 5.0)); // sqrt(3^2 + 4^2) = 5
    }

    #[test]
    fn test_setters() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = DistanceJoint::new(
            handle, body_a, body_b,
            Vec2::new(1.0, 0.0),
            Vec2::new(-1.0, 0.0),
        );

        joint.set_local_anchor_a(Vec2::new(2.0, 3.0));
        assert_eq!(joint.local_anchor_a(), Vec2::new(2.0, 3.0));

        joint.set_local_anchor_b(Vec2::new(-2.0, -3.0));
        assert_eq!(joint.local_anchor_b(), Vec2::new(-2.0, -3.0));

        joint.set_length(10.0);
        assert!(approx_eq(joint.length(), 10.0));

        joint.set_min_length(5.0);
        assert!(approx_eq(joint.min_length(), 5.0));

        joint.set_max_length(15.0);
        assert!(approx_eq(joint.max_length(), 15.0));

        joint.set_stiffness(0.5);
        assert!(approx_eq(joint.stiffness(), 0.5));

        joint.set_damping(0.2);
        assert!(approx_eq(joint.damping(), 0.2));
    }

    #[test]
    fn test_constraint_trait() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut joint = DistanceJoint::new(
            handle, body_a, body_b,
            Vec2::new(1.0, 0.0),
            Vec2::new(-1.0, 0.0),
        );

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

        let joint = DistanceJoint::new(
            handle, body_a, body_b,
            Vec2::new(1.0, 0.0),
            Vec2::new(-1.0, 0.0),
        );

        let joint_str = format!("{}", joint);
        assert!(joint_str.contains("DistanceJoint"));
    }

    #[test]
    fn test_is_at_limits() {
        let handle = ConstraintHandle::new(1, 0);
        let body_a = BodyHandle::new(1, 0);
        let body_b = BodyHandle::new(2, 0);

        let mut body_a_obj = Body::new_dynamic(body_a);
        body_a_obj.set_position(Vec2::new(0.0, 0.0));

        let mut body_b_obj = Body::new_dynamic(body_b);
        body_b_obj.set_position(Vec2::new(0.0, 0.0));

        let mut joint = DistanceJoint::new(
            handle, body_a, body_b,
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 0.0),
        );

        joint.set_length(5.0);
        joint.set_min_length(4.0);
        joint.set_max_length(6.0);

        // Distance is 0, which is at min limit
        assert!(joint.is_at_limits(&body_a_obj, &body_b_obj));

        body_b_obj.set_position(Vec2::new(5.0, 0.0));
        // Distance is 5, which is within limits
        assert!(!joint.is_at_limits(&body_a_obj, &body_b_obj));

        body_b_obj.set_position(Vec2::new(6.0, 0.0));
        // Distance is 6, which is at max limit
        assert!(joint.is_at_limits(&body_a_obj, &body_b_obj));
    }
}
