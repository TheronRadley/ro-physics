//! Rigid body implementation.
//!
//! This module provides the Body struct which represents a rigid body
//! in the physics simulation.

use std::fmt;

use super::{BodyHandle, BodyType};
use crate::math::{Rot2, Transform, Vec2};

/// A rigid body in the physics simulation.
///
/// Bodies are the primary entities in the physics world. They have
/// position, orientation, velocity, and can have colliders attached.
///
/// # Body Types
///
/// - **Static**: Fixed in place, infinite mass, does not move
/// - **Dynamic**: Affected by forces and collisions, normal physics behavior
/// - **Kinematic**: Moves according to velocity but not affected by forces
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Body {
    /// Handle to this body
    handle: BodyHandle,
    /// Type of the body (static, dynamic, kinematic)
    body_type: BodyType,
    /// Transform (position and rotation)
    transform: Transform,
    /// Linear velocity
    linear_velocity: Vec2,
    /// Angular velocity (in radians per second)
    angular_velocity: f32,
    /// Force accumulator
    force: Vec2,
    /// Torque accumulator
    torque: f32,
    /// Mass (0 for static bodies)
    mass: f32,
    /// Inverse mass (0 for static bodies)
    inv_mass: f32,
    /// Moment of inertia
    inertia: f32,
    /// Inverse inertia (0 for static bodies)
    inv_inertia: f32,
    /// Linear damping coefficient
    linear_damping: f32,
    /// Angular damping coefficient
    angular_damping: f32,
    /// Gravity scale (1.0 for normal gravity)
    gravity_scale: f32,
    /// Whether the body is enabled
    enabled: bool,
    /// Whether the body is sleeping
    sleeping: bool,
    /// Seconds the body has been below the sleep thresholds
    sleep_timer: f32,
    /// User-defined data
    ///
    /// Not cloned by [`Clone`] and skipped during (de)serialization.
    #[cfg_attr(feature = "serialization", serde(skip))]
    user_data: Option<Box<dyn std::any::Any + Send + Sync>>,
}

impl Body {
    /// Create a new body with the given handle and type
    #[inline]
    #[must_use]
    pub fn new(handle: BodyHandle, body_type: BodyType) -> Self {
        Self {
            handle,
            body_type,
            transform: Transform::identity(),
            linear_velocity: Vec2::zero(),
            angular_velocity: 0.0,
            force: Vec2::zero(),
            torque: 0.0,
            mass: 0.0,
            inv_mass: 0.0,
            inertia: 0.0,
            inv_inertia: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            enabled: true,
            sleeping: false,
            sleep_timer: 0.0,
            user_data: None,
        }
    }

    /// Create a new static body
    #[inline]
    #[must_use]
    pub fn new_static(handle: BodyHandle) -> Self {
        Self::new(handle, BodyType::Static)
    }

    /// Create a new dynamic body
    #[inline]
    #[must_use]
    pub fn new_dynamic(handle: BodyHandle) -> Self {
        Self::new(handle, BodyType::Dynamic)
    }

    /// Create a new kinematic body
    #[inline]
    #[must_use]
    pub fn new_kinematic(handle: BodyHandle) -> Self {
        Self::new(handle, BodyType::Kinematic)
    }

    /// Get the body handle
    #[inline]
    #[must_use]
    pub const fn handle(&self) -> BodyHandle {
        self.handle
    }

    /// Get the body type
    #[inline]
    #[must_use]
    pub const fn body_type(&self) -> BodyType {
        self.body_type
    }

    /// Check if the body is static
    #[inline]
    #[must_use]
    pub fn is_static(&self) -> bool {
        self.body_type.is_static()
    }

    /// Check if the body is dynamic
    #[inline]
    #[must_use]
    pub fn is_dynamic(&self) -> bool {
        self.body_type.is_dynamic()
    }

    /// Check if the body is kinematic
    #[inline]
    #[must_use]
    pub fn is_kinematic(&self) -> bool {
        self.body_type.is_kinematic()
    }

    /// Get the transform
    #[inline]
    #[must_use]
    pub const fn transform(&self) -> Transform {
        self.transform
    }

    /// Get the position
    #[inline]
    #[must_use]
    pub const fn position(&self) -> Vec2 {
        self.transform.position
    }

    /// Get the rotation
    #[inline]
    #[must_use]
    pub const fn rotation(&self) -> Rot2 {
        self.transform.rotation
    }

    /// Set the position
    #[inline]
    pub fn set_position(&mut self, position: Vec2) {
        self.transform.position = position;
        self.wake_up();
    }

    /// Set the rotation
    #[inline]
    pub fn set_rotation(&mut self, rotation: Rot2) {
        self.transform.rotation = rotation;
        self.wake_up();
    }

    /// Set the transform
    #[inline]
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        self.wake_up();
    }

    /// Get the linear velocity
    #[inline]
    #[must_use]
    pub const fn linear_velocity(&self) -> Vec2 {
        self.linear_velocity
    }

    /// Set the linear velocity
    #[inline]
    pub fn set_linear_velocity(&mut self, velocity: Vec2) {
        self.linear_velocity = velocity;
        self.wake_up();
    }

    /// Get the angular velocity
    #[inline]
    #[must_use]
    pub const fn angular_velocity(&self) -> f32 {
        self.angular_velocity
    }

    /// Set the angular velocity
    #[inline]
    pub fn set_angular_velocity(&mut self, velocity: f32) {
        self.angular_velocity = velocity;
        self.wake_up();
    }

    // ---- Internal setters used by the simulation ----
    //
    // The public setters above wake the body up, which is what a user changing
    // the state of a body expects. The simulation itself touches velocities
    // and positions on every step (gravity, solver impulses, integration), and
    // those changes must not reset the sleep timer, or no body could ever
    // fall asleep.

    /// Set the linear velocity without waking the body
    #[inline]
    pub(crate) fn set_linear_velocity_no_wake(&mut self, velocity: Vec2) {
        self.linear_velocity = velocity;
    }

    /// Set the angular velocity without waking the body
    #[inline]
    pub(crate) fn set_angular_velocity_no_wake(&mut self, velocity: f32) {
        self.angular_velocity = velocity;
    }

    /// Set the position without waking the body
    #[inline]
    pub(crate) fn set_position_no_wake(&mut self, position: Vec2) {
        self.transform.position = position;
    }

    /// Set the rotation without waking the body
    #[inline]
    pub(crate) fn set_rotation_no_wake(&mut self, rotation: Rot2) {
        self.transform.rotation = rotation;
    }

    /// Add an impulse at a point without waking the body
    #[inline]
    pub(crate) fn add_impulse_at_point_no_wake(&mut self, impulse: Vec2, point: Vec2) {
        if !self.is_dynamic() {
            return;
        }

        self.linear_velocity += impulse * self.inv_mass;
        let r = point - self.transform.position;
        self.angular_velocity += self.inv_inertia * r.cross(impulse);
    }

    /// Add a linear impulse without waking the body
    #[inline]
    pub(crate) fn add_impulse_no_wake(&mut self, impulse: Vec2) {
        if self.is_dynamic() {
            self.linear_velocity += impulse * self.inv_mass;
        }
    }

    /// Add an angular impulse without waking the body
    #[inline]
    pub(crate) fn add_angular_impulse_no_wake(&mut self, impulse: f32) {
        if self.is_dynamic() {
            self.angular_velocity += impulse * self.inv_inertia;
        }
    }

    /// Get the mass
    #[inline]
    #[must_use]
    pub const fn mass(&self) -> f32 {
        self.mass
    }

    /// Get the inverse mass
    #[inline]
    #[must_use]
    pub const fn inv_mass(&self) -> f32 {
        self.inv_mass
    }

    /// Get the inertia
    #[inline]
    #[must_use]
    pub const fn inertia(&self) -> f32 {
        self.inertia
    }

    /// Get the inverse inertia
    #[inline]
    #[must_use]
    pub const fn inv_inertia(&self) -> f32 {
        self.inv_inertia
    }

    /// Set the mass properties
    #[inline]
    pub fn set_mass_properties(&mut self, mass: f32, inertia: f32) {
        self.mass = mass;
        self.inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        self.inertia = inertia;
        self.inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
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

    /// Get the gravity scale
    #[inline]
    #[must_use]
    pub const fn gravity_scale(&self) -> f32 {
        self.gravity_scale
    }

    /// Set the gravity scale
    #[inline]
    pub fn set_gravity_scale(&mut self, scale: f32) {
        self.gravity_scale = scale;
    }

    /// Check if the body is enabled
    #[inline]
    #[must_use]
    pub const fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Set if the body is enabled
    #[inline]
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if enabled {
            self.wake_up();
        }
    }

    /// Check if the body is sleeping
    #[inline]
    #[must_use]
    pub const fn is_sleeping(&self) -> bool {
        self.sleeping
    }

    /// Set if the body is sleeping
    ///
    /// Putting a body to sleep zeroes its velocity so that a sleeping body can
    /// never accumulate motion.
    #[inline]
    pub fn set_sleeping(&mut self, sleeping: bool) {
        self.sleeping = sleeping;
        if sleeping {
            self.linear_velocity = Vec2::zero();
            self.angular_velocity = 0.0;
            self.force = Vec2::zero();
            self.torque = 0.0;
        } else {
            self.sleep_timer = 0.0;
        }
    }

    /// Wake up the body and reset its sleep timer
    #[inline]
    pub fn wake_up(&mut self) {
        self.sleeping = false;
        self.sleep_timer = 0.0;
    }

    /// Get the force accumulator
    #[inline]
    #[must_use]
    pub const fn force(&self) -> Vec2 {
        self.force
    }

    /// Get the torque accumulator
    #[inline]
    #[must_use]
    pub const fn torque(&self) -> f32 {
        self.torque
    }

    /// Add a force to the body
    #[inline]
    pub fn add_force(&mut self, force: Vec2) {
        self.force += force;
        self.wake_up();
    }

    /// Add a force at a point to the body
    #[inline]
    pub fn add_force_at_point(&mut self, force: Vec2, point: Vec2) {
        self.force += force;
        self.torque += (point - self.transform.position).cross(force);
        self.wake_up();
    }

    /// Add a torque to the body
    #[inline]
    pub fn add_torque(&mut self, torque: f32) {
        self.torque += torque;
        self.wake_up();
    }

    /// Add an impulse to the body
    #[inline]
    pub fn add_impulse(&mut self, impulse: Vec2) {
        if self.is_dynamic() {
            self.linear_velocity += impulse * self.inv_mass;
            self.wake_up();
        }
    }

    /// Add an impulse at a point to the body
    #[inline]
    pub fn add_impulse_at_point(&mut self, impulse: Vec2, point: Vec2) {
        if self.is_dynamic() {
            self.linear_velocity += impulse * self.inv_mass;
            let r = point - self.transform.position;
            self.angular_velocity += r.cross(impulse) * self.inv_inertia;
            self.wake_up();
        }
    }

    /// Add an angular impulse to the body
    #[inline]
    pub fn add_angular_impulse(&mut self, impulse: f32) {
        if self.is_dynamic() {
            self.angular_velocity += impulse * self.inv_inertia;
            self.wake_up();
        }
    }

    /// Clear the force and torque accumulators
    #[inline]
    pub fn clear_accumulators(&mut self) {
        self.force = Vec2::zero();
        self.torque = 0.0;
    }

    /// Seconds the body has continuously been below the sleep thresholds
    #[inline]
    #[must_use]
    pub const fn sleep_timer(&self) -> f32 {
        self.sleep_timer
    }

    /// Reset the sleep timer to zero
    #[inline]
    pub fn reset_sleep_timer(&mut self) {
        self.sleep_timer = 0.0;
    }

    /// Advance the sleep timer by `dt`
    ///
    /// Returns the accumulated time so the caller can compare it against its
    /// configured sleep threshold.
    #[inline]
    pub fn advance_sleep_timer(&mut self, dt: f32) -> f32 {
        self.sleep_timer += dt.max(0.0);
        self.sleep_timer
    }

    /// Get user data
    #[inline]
    #[must_use]
    pub fn user_data<T: 'static + Send + Sync>(&self) -> Option<&T> {
        self.user_data
            .as_ref()
            .and_then(|data| data.downcast_ref::<T>())
    }

    /// Get user data mutably
    #[inline]
    #[must_use]
    pub fn user_data_mut<T: 'static + Send + Sync>(&mut self) -> Option<&mut T> {
        self.user_data
            .as_mut()
            .and_then(|data| data.downcast_mut::<T>())
    }

    /// Set user data
    #[inline]
    pub fn set_user_data<T: 'static + Send + Sync>(&mut self, data: T) {
        self.user_data = Some(Box::new(data));
    }

    /// Remove user data
    #[inline]
    pub fn remove_user_data(&mut self) {
        self.user_data = None;
    }

    /// Check if the body is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        if !self.transform.is_finite() {
            return false;
        }
        if !self.linear_velocity.is_finite() {
            return false;
        }
        if !self.angular_velocity.is_finite() {
            return false;
        }
        if !self.force.is_finite() {
            return false;
        }
        if !self.torque.is_finite() {
            return false;
        }

        match self.body_type {
            BodyType::Static => true,
            BodyType::Dynamic | BodyType::Kinematic => {
                if self.mass < 0.0 || !self.mass.is_finite() {
                    return false;
                }
                if self.inertia < 0.0 || !self.inertia.is_finite() {
                    return false;
                }
                true
            }
        }
    }

    /// Validate the body
    #[inline]
    #[must_use]
    pub fn validate(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        if !self.transform.validate() {
            return false;
        }

        if self.linear_velocity.length() > crate::constants::MAX_FLOAT {
            return false;
        }

        if self.angular_velocity.abs() > crate::constants::MAX_FLOAT {
            return false;
        }

        if self.force.length() > crate::constants::MAX_FLOAT {
            return false;
        }

        if self.torque.abs() > crate::constants::MAX_FLOAT {
            return false;
        }

        true
    }

    /// Check if the body is moving
    #[inline]
    #[must_use]
    pub fn is_moving(&self, linear_threshold: f32, angular_threshold: f32) -> bool {
        self.linear_velocity.length_squared() > linear_threshold * linear_threshold
            || self.angular_velocity.abs() > angular_threshold
    }

    /// Get the velocity at a point on the body
    #[inline]
    #[must_use]
    pub fn velocity_at_point(&self, point: Vec2) -> Vec2 {
        let r = point - self.transform.position;
        self.linear_velocity + Vec2::new(-self.angular_velocity * r.y, self.angular_velocity * r.x)
    }

    /// Get the energy of the body
    #[inline]
    #[must_use]
    pub fn energy(&self) -> f32 {
        if self.is_static() {
            0.0
        } else {
            let linear_ke = 0.5 * self.mass * self.linear_velocity.length_squared();
            let angular_ke = 0.5 * self.inertia * self.angular_velocity * self.angular_velocity;
            linear_ke + angular_ke
        }
    }
}

impl Clone for Body {
    /// Clone the body.
    ///
    /// User data is **not** cloned: [`Box<dyn Any>`][std::any::Any] cannot be
    /// duplicated generically, so the clone starts out without user data.
    fn clone(&self) -> Self {
        Self {
            handle: self.handle,
            body_type: self.body_type,
            transform: self.transform,
            linear_velocity: self.linear_velocity,
            angular_velocity: self.angular_velocity,
            force: self.force,
            torque: self.torque,
            mass: self.mass,
            inv_mass: self.inv_mass,
            inertia: self.inertia,
            inv_inertia: self.inv_inertia,
            linear_damping: self.linear_damping,
            angular_damping: self.angular_damping,
            gravity_scale: self.gravity_scale,
            enabled: self.enabled,
            sleeping: self.sleeping,
            sleep_timer: self.sleep_timer,
            user_data: None,
        }
    }
}

impl fmt::Debug for Body {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Body")
            .field("handle", &self.handle)
            .field("body_type", &self.body_type)
            .field("position", &self.transform.position)
            .field("rotation", &self.transform.rotation.angle())
            .field("linear_velocity", &self.linear_velocity)
            .field("angular_velocity", &self.angular_velocity)
            .field("mass", &self.mass)
            .field("inertia", &self.inertia)
            .field("sleeping", &self.sleeping)
            .field("has_user_data", &self.user_data.is_some())
            .finish()
    }
}

impl Default for Body {
    fn default() -> Self {
        Self::new_static(BodyHandle::null())
    }
}

impl fmt::Display for Body {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Body(handle: {}, type: {}, position: {}, velocity: {:.2})",
            self.handle,
            self.body_type,
            self.transform.position,
            self.linear_velocity.length()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::{approx_eq, approx_eq_eps};

    #[test]
    fn test_construction() {
        let handle = BodyHandle::new(1, 0);

        let static_body = Body::new_static(handle);
        assert!(static_body.is_static());
        assert_eq!(static_body.handle(), handle);

        let dynamic_body = Body::new_dynamic(handle);
        assert!(dynamic_body.is_dynamic());

        let kinematic_body = Body::new_kinematic(handle);
        assert!(kinematic_body.is_kinematic());
    }

    #[test]
    fn test_transform() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert_eq!(body.position(), Vec2::zero());
        assert!(body.rotation().is_identity());

        body.set_position(Vec2::new(10.0, 20.0));
        assert_eq!(body.position(), Vec2::new(10.0, 20.0));

        body.set_rotation(Rot2::new(crate::constants::PI_OVER_TWO));
        assert!(approx_eq(
            body.rotation().angle(),
            crate::constants::PI_OVER_TWO
        ));

        body.set_transform(Transform::new(Vec2::new(5.0, 5.0), Rot2::identity()));
        assert_eq!(body.position(), Vec2::new(5.0, 5.0));
    }

    #[test]
    fn test_velocity() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert_eq!(body.linear_velocity(), Vec2::zero());
        assert_eq!(body.angular_velocity(), 0.0);

        body.set_linear_velocity(Vec2::new(10.0, 5.0));
        assert_eq!(body.linear_velocity(), Vec2::new(10.0, 5.0));

        body.set_angular_velocity(2.0);
        assert_eq!(body.angular_velocity(), 2.0);
    }

    #[test]
    fn test_mass_properties() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert_eq!(body.mass(), 0.0);
        assert_eq!(body.inv_mass(), 0.0);

        body.set_mass_properties(10.0, 5.0);
        assert_eq!(body.mass(), 10.0);
        assert!(approx_eq(body.inv_mass(), 0.1));
        assert_eq!(body.inertia(), 5.0);
        assert!(approx_eq(body.inv_inertia(), 0.2));
    }

    #[test]
    fn test_damping() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        body.set_linear_damping(0.5);
        assert_eq!(body.linear_damping(), 0.5);

        body.set_angular_damping(0.8);
        assert_eq!(body.angular_damping(), 0.8);

        // Test clamping
        body.set_linear_damping(2.0);
        assert_eq!(body.linear_damping(), 1.0);

        body.set_angular_damping(-0.5);
        assert_eq!(body.angular_damping(), 0.0);
    }

    #[test]
    fn test_gravity_scale() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert_eq!(body.gravity_scale(), 1.0);

        body.set_gravity_scale(0.5);
        assert_eq!(body.gravity_scale(), 0.5);
    }

    #[test]
    fn test_enabled() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert!(body.is_enabled());

        body.set_enabled(false);
        assert!(!body.is_enabled());

        body.set_enabled(true);
        assert!(body.is_enabled());
    }

    #[test]
    fn test_sleeping() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert!(!body.is_sleeping());
        assert!(approx_eq(body.sleep_timer(), 0.0));

        body.set_linear_velocity(Vec2::new(1.0, 0.0));
        body.set_sleeping(true);
        assert!(body.is_sleeping());
        // Sleeping bodies never keep momentum
        assert_eq!(body.linear_velocity(), Vec2::zero());
        assert_eq!(body.angular_velocity(), 0.0);

        body.wake_up();
        assert!(!body.is_sleeping());
    }

    #[test]
    fn test_sleep_timer() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        body.advance_sleep_timer(0.1);
        body.advance_sleep_timer(0.1);
        assert!(approx_eq(body.sleep_timer(), 0.2));

        body.wake_up();
        assert!(approx_eq(body.sleep_timer(), 0.0));
    }

    #[test]
    fn test_forces() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert_eq!(body.force(), Vec2::zero());
        assert_eq!(body.torque(), 0.0);

        body.add_force(Vec2::new(10.0, 5.0));
        assert_eq!(body.force(), Vec2::new(10.0, 5.0));

        body.add_torque(2.0);
        assert_eq!(body.torque(), 2.0);

        body.add_force_at_point(Vec2::new(5.0, 0.0), Vec2::new(1.0, 0.0));
        // Force should be added, torque should be increased by r x F
        assert_eq!(body.force(), Vec2::new(15.0, 5.0));
        assert!(approx_eq_eps(body.torque(), 2.0 + 0.0, 1e-5)); // (1,0) x (5,0) = 0

        body.clear_accumulators();
        assert_eq!(body.force(), Vec2::zero());
        assert_eq!(body.torque(), 0.0);
    }

    #[test]
    fn test_impulses() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);
        body.set_mass_properties(10.0, 5.0);

        body.add_impulse(Vec2::new(10.0, 0.0));
        assert_eq!(body.linear_velocity(), Vec2::new(1.0, 0.0));

        body.add_angular_impulse(5.0);
        assert!(approx_eq(body.angular_velocity(), 1.0));

        body.add_impulse_at_point(Vec2::new(10.0, 0.0), Vec2::new(1.0, 0.0));
        // Linear: v += impulse * inv_mass = (10,0) * 0.1 = (1,0)
        // Angular: w += r x impulse * inv_inertia = (0,0) x (10,0) * 0.2 = 0
        // So linear velocity should be (2,0)
        assert_eq!(body.linear_velocity(), Vec2::new(2.0, 0.0));
    }

    #[test]
    fn test_static_body_impulses() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_static(handle);
        body.set_mass_properties(10.0, 5.0);

        // Static bodies should not be affected by impulses
        body.add_impulse(Vec2::new(10.0, 0.0));
        assert_eq!(body.linear_velocity(), Vec2::zero());

        body.add_angular_impulse(5.0);
        assert_eq!(body.angular_velocity(), 0.0);
    }

    #[test]
    fn test_clone_drops_user_data() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);
        body.set_user_data(42_i32);
        body.set_mass_properties(2.0, 3.0);

        let cloned = body.clone();

        assert_eq!(cloned.mass(), 2.0);
        assert_eq!(cloned.inertia(), 3.0);
        assert!(cloned.user_data::<i32>().is_none());
        assert_eq!(*body.user_data::<i32>().unwrap(), 42);
    }

    #[test]
    fn test_debug_output() {
        let handle = BodyHandle::new(1, 0);
        let body = Body::new_dynamic(handle);
        let text = format!("{body:?}");
        assert!(text.contains("Body"));
        assert!(text.contains("Dynamic"));
    }

    #[test]
    fn test_user_data() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert!(body.user_data::<i32>().is_none());

        body.set_user_data(42);
        assert_eq!(*body.user_data::<i32>().unwrap(), 42);

        body.remove_user_data();
        assert!(body.user_data::<i32>().is_none());
    }

    #[test]
    fn test_validation() {
        let handle = BodyHandle::new(1, 0);
        let body = Body::new_dynamic(handle);

        assert!(body.is_valid());
        assert!(body.validate());

        let mut invalid_body = Body::new_dynamic(handle);
        invalid_body.set_position(Vec2::new(f32::NAN, 0.0));
        assert!(!invalid_body.is_valid());
        assert!(!invalid_body.validate());
    }

    #[test]
    fn test_energy() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);
        body.set_mass_properties(10.0, 5.0);
        body.set_linear_velocity(Vec2::new(2.0, 0.0));
        body.set_angular_velocity(2.0);

        let energy = body.energy();
        // KE = 0.5 * m * v^2 + 0.5 * I * w^2 = 0.5 * 10 * 4 + 0.5 * 5 * 4 = 20 + 10 = 30
        assert!(approx_eq(energy, 30.0));
    }

    #[test]
    fn test_velocity_at_point() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);
        body.set_linear_velocity(Vec2::new(1.0, 0.0));
        body.set_angular_velocity(2.0);
        body.set_position(Vec2::new(0.0, 0.0));

        // Velocity at (1,0): v + (-w * y, w * x) = (1,0) + (-2*0, 2*1) = (1,2)
        let vel = body.velocity_at_point(Vec2::new(1.0, 0.0));
        assert_eq!(vel, Vec2::new(1.0, 2.0));
    }

    #[test]
    fn test_is_moving() {
        let handle = BodyHandle::new(1, 0);
        let mut body = Body::new_dynamic(handle);

        assert!(!body.is_moving(0.1, 0.1));

        body.set_linear_velocity(Vec2::new(0.2, 0.0));
        assert!(body.is_moving(0.1, 0.1));

        body.set_linear_velocity(Vec2::zero());
        body.set_angular_velocity(0.2);
        assert!(body.is_moving(0.1, 0.1));
    }

    #[test]
    fn test_default() {
        let body = Body::default();
        assert!(body.is_static());
        assert!(body.handle().is_null());
    }

    #[test]
    fn test_display() {
        let handle = BodyHandle::new(1, 0);
        let body = Body::new_dynamic(handle);
        let body_str = format!("{}", body);
        assert!(body_str.contains("Body"));
        assert!(body_str.contains("Dynamic"));
    }
}
