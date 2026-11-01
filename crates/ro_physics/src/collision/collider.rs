//! Collider implementation for the physics engine.
//!
//! This module provides collider types that can be attached to bodies.

use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::bodies::BodyHandle;
use crate::math::Transform;
use crate::shapes::Shape;
use super::Filter;

/// A handle to a collider
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct ColliderHandle {
    /// Unique identifier for the collider
    pub id: u64,
    /// Generation counter to detect stale handles
    pub generation: u32,
}

impl ColliderHandle {
    /// Create a new collider handle
    #[inline]
    #[must_use]
    pub const fn new(id: u64, generation: u32) -> Self {
        Self { id, generation }
    }

    /// Create a null handle (invalid)
    #[inline]
    #[must_use]
    pub const fn null() -> Self {
        Self {
            id: 0,
            generation: 0,
        }
    }

    /// Check if the handle is null
    #[inline]
    #[must_use]
    pub fn is_null(self) -> bool {
        self.id == 0 && self.generation == 0
    }

    /// Increment the generation (used when a collider is removed and its slot reused)
    #[inline]
    #[must_use]
    pub fn next_generation(self) -> Self {
        Self {
            generation: self.generation.wrapping_add(1),
            ..self
        }
    }
}

impl Default for ColliderHandle {
    fn default() -> Self {
        Self::null()
    }
}

impl fmt::Display for ColliderHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_null() {
            write!(f, "ColliderHandle(null)")
        } else {
            write!(f, "ColliderHandle(id: {}, gen: {})", self.id, self.generation)
        }
    }
}

/// A generator for creating unique collider handles
#[derive(Debug)]
pub struct ColliderHandleGenerator {
    next_id: AtomicU64,
}

impl ColliderHandleGenerator {
    /// Create a new handle generator
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            next_id: AtomicU64::new(1),
        }
    }

    /// Generate a new collider handle
    #[inline]
    #[must_use]
    pub fn generate(&self) -> ColliderHandle {
        let id = self.next_id.fetch_add(1, Ordering::SeqCst);
        ColliderHandle::new(id, 0)
    }

    /// Reset the generator
    #[inline]
    pub fn reset(&self) {
        self.next_id.store(1, Ordering::SeqCst);
    }
}

impl Default for ColliderHandleGenerator {
    fn default() -> Self {
        Self::new()
    }
}

/// A collider attached to a body.
///
/// Colliders define the shape and collision properties of a body.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Collider {
    /// Handle to this collider
    handle: ColliderHandle,
    /// Handle to the parent body
    body_handle: BodyHandle,
    /// Local transform relative to the body
    local_transform: Transform,
    /// Shape of the collider
    shape: Shape,
    /// Collision filter
    filter: Filter,
    /// Friction coefficient
    friction: f32,
    /// Restitution (bounciness)
    restitution: f32,
    /// Whether this is a sensor/trigger
    is_sensor: bool,
    /// Whether the collider is enabled
    enabled: bool,
}

impl Collider {
    /// Create a new collider
    #[inline]
    #[must_use]
    pub fn new(
        handle: ColliderHandle,
        body_handle: BodyHandle,
        shape: Shape,
    ) -> Self {
        Self {
            handle,
            body_handle,
            local_transform: Transform::identity(),
            shape,
            filter: Filter::default(),
            friction: 0.6,
            restitution: 0.0,
            is_sensor: false,
            enabled: true,
        }
    }

    /// Get the collider handle
    #[inline]
    #[must_use]
    pub const fn handle(&self) -> ColliderHandle {
        self.handle
    }

    /// Get the body handle
    #[inline]
    #[must_use]
    pub const fn body_handle(&self) -> BodyHandle {
        self.body_handle
    }

    /// Get the local transform
    #[inline]
    #[must_use]
    pub const fn local_transform(&self) -> Transform {
        self.local_transform
    }

    /// Set the local transform
    #[inline]
    pub fn set_local_transform(&mut self, transform: Transform) {
        self.local_transform = transform;
    }

    /// Get the shape
    #[inline]
    #[must_use]
    pub const fn shape(&self) -> &Shape {
        &self.shape
    }

    /// A characteristic length of the collider, used by continuous collision
    /// detection to decide whether a body can tunnel through it in one step
    ///
    /// This is the half diagonal of the collider's local AABB, i.e. roughly the
    /// radius of a disc that encloses the shape.
    #[inline]
    #[must_use]
    pub fn characteristic_size(&self) -> f32 {
        let aabb = self.shape.local_aabb();
        let half = aabb.half_extents();
        half.length().max(crate::constants::EPSILON)
    }

    /// Get the shape mutably
    #[inline]
    #[must_use]
    pub fn shape_mut(&mut self) -> &mut Shape {
        &mut self.shape
    }

    /// Set the shape
    #[inline]
    pub fn set_shape(&mut self, shape: Shape) {
        self.shape = shape;
    }

    /// Get the filter
    #[inline]
    #[must_use]
    pub const fn filter(&self) -> Filter {
        self.filter
    }

    /// Set the filter
    #[inline]
    pub fn set_filter(&mut self, filter: Filter) {
        self.filter = filter;
    }

    /// Get the friction
    #[inline]
    #[must_use]
    pub const fn friction(&self) -> f32 {
        self.friction
    }

    /// Set the friction
    #[inline]
    pub fn set_friction(&mut self, friction: f32) {
        self.friction = friction.clamp(0.0, 1.0);
    }

    /// Get the restitution
    #[inline]
    #[must_use]
    pub const fn restitution(&self) -> f32 {
        self.restitution
    }

    /// Set the restitution
    #[inline]
    pub fn set_restitution(&mut self, restitution: f32) {
        self.restitution = restitution.clamp(0.0, 1.0);
    }

    /// Check if this is a sensor
    #[inline]
    #[must_use]
    pub const fn is_sensor(&self) -> bool {
        self.is_sensor
    }

    /// Set if this is a sensor
    #[inline]
    pub fn set_is_sensor(&mut self, is_sensor: bool) {
        self.is_sensor = is_sensor;
    }

    /// Check if the collider is enabled
    #[inline]
    #[must_use]
    pub const fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Set if the collider is enabled
    #[inline]
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Get the world transform (body transform * local transform)
    #[inline]
    #[must_use]
    pub fn world_transform(&self, body_transform: Transform) -> Transform {
        body_transform * self.local_transform
    }

    /// Get the AABB in world space
    #[inline]
    #[must_use]
    pub fn aabb(&self, body_transform: Transform) -> crate::geometry::Aabb {
        self.shape.aabb(self.world_transform(body_transform))
    }

    /// Check if the collider is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        if !self.local_transform.is_finite() {
            return false;
        }
        if !self.shape.is_valid() {
            return false;
        }
        if !self.friction.is_finite() || self.friction < 0.0 {
            return false;
        }
        if !self.restitution.is_finite() || self.restitution < 0.0 {
            return false;
        }
        true
    }

    /// Validate the collider
    #[inline]
    #[must_use]
    pub fn validate(&self) -> bool {
        if !self.is_valid() {
            return false;
        }
        if !self.local_transform.validate() {
            return false;
        }
        if !self.shape.validate() {
            return false;
        }
        if self.friction > 1.0 || self.restitution > 1.0 {
            return false;
        }
        true
    }
}

impl Default for Collider {
    fn default() -> Self {
        Self {
            handle: ColliderHandle::null(),
            body_handle: BodyHandle::null(),
            local_transform: Transform::identity(),
            shape: Shape::default(),
            filter: Filter::default(),
            friction: 0.6,
            restitution: 0.0,
            is_sensor: false,
            enabled: true,
        }
    }
}

impl fmt::Display for Collider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Collider(handle: {}, body: {}, shape: {}, sensor: {})",
            self.handle,
            self.body_handle,
            self.shape.shape_type(),
            self.is_sensor
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Rot2, Transform, Vec2};
    use crate::shapes::Circle;

    #[test]
    fn test_collider_handle() {
        let handle1 = ColliderHandle::new(1, 0);
        let handle2 = ColliderHandle::new(2, 0);
        let null = ColliderHandle::null();

        assert!(!handle1.is_null());
        assert!(!handle2.is_null());
        assert!(null.is_null());

        assert_ne!(handle1, handle2);
        assert_ne!(handle1, null);
    }

    #[test]
    fn test_next_generation() {
        let handle = ColliderHandle::new(1, 0);
        let next = handle.next_generation();

        assert_eq!(handle.id, next.id);
        assert_eq!(next.generation, 1);
    }

    #[test]
    fn test_handle_generator() {
        let generator = ColliderHandleGenerator::new();

        let h1 = generator.generate();
        let h2 = generator.generate();
        let h3 = generator.generate();

        assert_eq!(h1.id, 1);
        assert_eq!(h2.id, 2);
        assert_eq!(h3.id, 3);

        assert_ne!(h1, h2);
        assert_ne!(h2, h3);
    }

    #[test]
    fn test_construction() {
        let handle = ColliderHandle::new(1, 0);
        let body_handle = BodyHandle::new(1, 0);
        let shape = Shape::Circle(Circle::new(5.0));

        let collider = Collider::new(handle, body_handle, shape);

        assert_eq!(collider.handle(), handle);
        assert_eq!(collider.body_handle(), body_handle);
        assert!(collider.shape().is_circle());
    }

    #[test]
    fn test_properties() {
        let handle = ColliderHandle::new(1, 0);
        let body_handle = BodyHandle::new(1, 0);
        let shape = Shape::Circle(Circle::new(5.0));
        let mut collider = Collider::new(handle, body_handle, shape);

        // Test defaults
        assert!(collider.local_transform().is_identity());
        assert!(approx_eq(collider.friction(), 0.6));
        assert!(approx_eq(collider.restitution(), 0.0));
        assert!(!collider.is_sensor());
        assert!(collider.is_enabled());

        // Test setters
        collider.set_local_transform(Transform::new(Vec2::new(1.0, 2.0), Rot2::identity()));
        assert_eq!(collider.local_transform().position, Vec2::new(1.0, 2.0));

        collider.set_friction(0.8);
        assert!(approx_eq(collider.friction(), 0.8));

        collider.set_restitution(0.5);
        assert!(approx_eq(collider.restitution(), 0.5));

        collider.set_is_sensor(true);
        assert!(collider.is_sensor());

        collider.set_enabled(false);
        assert!(!collider.is_enabled());
    }

    #[test]
    fn test_clamping() {
        let handle = ColliderHandle::new(1, 0);
        let body_handle = BodyHandle::new(1, 0);
        let shape = Shape::Circle(Circle::new(5.0));
        let mut collider = Collider::new(handle, body_handle, shape);

        collider.set_friction(2.0);
        assert_eq!(collider.friction(), 1.0);

        collider.set_friction(-0.5);
        assert_eq!(collider.friction(), 0.0);

        collider.set_restitution(2.0);
        assert_eq!(collider.restitution(), 1.0);

        collider.set_restitution(-0.5);
        assert_eq!(collider.restitution(), 0.0);
    }

    #[test]
    fn test_world_transform() {
        let handle = ColliderHandle::new(1, 0);
        let body_handle = BodyHandle::new(1, 0);
        let shape = Shape::Circle(Circle::new(5.0));
        let mut collider = Collider::new(handle, body_handle, shape);

        collider.set_local_transform(Transform::new(Vec2::new(1.0, 2.0), Rot2::identity()));

        let body_transform = Transform::new(Vec2::new(10.0, 20.0), Rot2::identity());
        let world_transform = collider.world_transform(body_transform);

        assert_eq!(world_transform.position, Vec2::new(11.0, 22.0));
    }

    #[test]
    fn test_aabb() {
        let handle = ColliderHandle::new(1, 0);
        let body_handle = BodyHandle::new(1, 0);
        let shape = Shape::Circle(Circle::new(5.0));
        let collider = Collider::new(handle, body_handle, shape);

        let body_transform = Transform::new(Vec2::new(10.0, 20.0), Rot2::identity());
        let aabb = collider.aabb(body_transform);

        assert_eq!(aabb.min, Vec2::new(5.0, 15.0));
        assert_eq!(aabb.max, Vec2::new(15.0, 25.0));
    }

    #[test]
    fn test_validation() {
        let handle = ColliderHandle::new(1, 0);
        let body_handle = BodyHandle::new(1, 0);
        let shape = Shape::Circle(Circle::new(5.0));
        let collider = Collider::new(handle, body_handle, shape);

        assert!(collider.is_valid());
        assert!(collider.validate());

        let mut invalid_collider = Collider::new(handle, body_handle, Shape::Circle(Circle::new(5.0)));
        invalid_collider.set_friction(f32::NAN);
        assert!(!invalid_collider.is_valid());
        assert!(!invalid_collider.validate());
    }

    #[test]
    fn test_default() {
        let collider = Collider::default();
        assert!(collider.handle().is_null());
        assert!(collider.body_handle().is_null());
    }

    #[test]
    fn test_display() {
        let handle = ColliderHandle::new(1, 0);
        let body_handle = BodyHandle::new(1, 0);
        let shape = Shape::Circle(Circle::new(5.0));
        let collider = Collider::new(handle, body_handle, shape);

        let collider_str = format!("{}", collider);
        assert!(collider_str.contains("Collider"));
        assert!(collider_str.contains("Circle"));
    }

    // Helper function for tests
    #[allow(dead_code)]
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= 1e-6
    }

}
