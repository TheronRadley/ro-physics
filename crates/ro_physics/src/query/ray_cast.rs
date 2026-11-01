//! Ray cast implementation.
//!
//! This module provides ray casting functionality for the physics world.

use std::fmt;

use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;
use crate::geometry::Ray;
use crate::math::Vec2;

/// Ray cast structure
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct RayCast {
    /// The ray to cast
    pub ray: Ray,
    /// Maximum distance to cast
    pub max_distance: f32,
}

impl RayCast {
    /// Create a new ray cast
    #[inline]
    #[must_use]
    pub const fn new(ray: Ray, max_distance: f32) -> Self {
        Self {
            ray,
            // `f32::max` is not `const` stable on the MSRV, so clamp by hand.
            max_distance: if max_distance > 0.0 { max_distance } else { 0.0 },
        }
    }

    /// Get the ray
    #[inline]
    #[must_use]
    pub const fn ray(&self) -> Ray {
        self.ray
    }

    /// Get the maximum distance
    #[inline]
    #[must_use]
    pub const fn max_distance(&self) -> f32 {
        self.max_distance
    }

    /// Set the ray
    #[inline]
    pub fn set_ray(&mut self, ray: Ray) -> &mut Self {
        self.ray = ray;
        self
    }

    /// Set the maximum distance
    #[inline]
    pub fn set_max_distance(&mut self, max_distance: f32) -> &mut Self {
        self.max_distance = max_distance.max(0.0);
        self
    }
}

impl Default for RayCast {
    fn default() -> Self {
        Self::new(Ray::default(), 100.0)
    }
}

impl fmt::Display for RayCast {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "RayCast(origin: {}, direction: {}, max_distance: {:.2})",
            self.ray.origin, self.ray.direction, self.max_distance
        )
    }
}

/// Ray cast result
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct RayCastResult {
    /// Handle to the body that was hit
    pub body_handle: BodyHandle,
    /// Handle to the collider that was hit
    pub collider_handle: ColliderHandle,
    /// World position of the hit point
    pub hit_point: Vec2,
    /// Surface normal at the hit point
    pub hit_normal: Vec2,
    /// Distance to the hit point along the ray
    pub distance: f32,
    /// Fraction of the ray length to the hit point
    pub fraction: f32,
}

impl RayCastResult {
    /// Create a new ray cast result
    #[inline]
    #[must_use]
    pub const fn new(
        body_handle: BodyHandle,
        collider_handle: ColliderHandle,
        hit_point: Vec2,
        hit_normal: Vec2,
        distance: f32,
        fraction: f32,
    ) -> Self {
        Self {
            body_handle,
            collider_handle,
            hit_point,
            hit_normal,
            distance,
            fraction,
        }
    }

    /// Get the body handle
    #[inline]
    #[must_use]
    pub const fn body_handle(&self) -> BodyHandle {
        self.body_handle
    }

    /// Get the collider handle
    #[inline]
    #[must_use]
    pub const fn collider_handle(&self) -> ColliderHandle {
        self.collider_handle
    }

    /// Get the hit point
    #[inline]
    #[must_use]
    pub const fn hit_point(&self) -> Vec2 {
        self.hit_point
    }

    /// Get the hit normal
    #[inline]
    #[must_use]
    pub const fn hit_normal(&self) -> Vec2 {
        self.hit_normal
    }

    /// Get the distance
    #[inline]
    #[must_use]
    pub const fn distance(&self) -> f32 {
        self.distance
    }

    /// Get the fraction
    #[inline]
    #[must_use]
    pub const fn fraction(&self) -> f32 {
        self.fraction
    }
}

impl Default for RayCastResult {
    fn default() -> Self {
        Self::new(
            BodyHandle::null(),
            ColliderHandle::null(),
            Vec2::zero(),
            Vec2::zero(),
            0.0,
            0.0,
        )
    }
}

impl fmt::Display for RayCastResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "RayCastResult(body: {}, collider: {}, hit: {}, normal: {}, distance: {:.4}, fraction: {:.4})",
            self.body_handle, self.collider_handle, self.hit_point, self.hit_normal, self.distance, self.fraction
        )
    }
}

/// Ray cast options
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct RayCastOptions {
    /// Whether to ignore sensor colliders
    pub ignore_sensors: bool,
    /// Whether to ignore static bodies
    pub ignore_static: bool,
    /// Whether to ignore dynamic bodies
    pub ignore_dynamic: bool,
    /// Whether to ignore kinematic bodies
    pub ignore_kinematic: bool,
    /// Collision filter mask
    pub mask: u32,
}

impl RayCastOptions {
    /// Create new ray cast options with defaults
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ignore_sensors: false,
            ignore_static: false,
            ignore_dynamic: false,
            ignore_kinematic: false,
            mask: 0xFFFF_FFFF,
        }
    }

    /// Set whether to ignore sensor colliders
    #[inline]
    #[must_use]
    pub const fn with_ignore_sensors(mut self, ignore: bool) -> Self {
        self.ignore_sensors = ignore;
        self
    }

    /// Set whether to ignore static bodies
    #[inline]
    #[must_use]
    pub const fn with_ignore_static(mut self, ignore: bool) -> Self {
        self.ignore_static = ignore;
        self
    }

    /// Set whether to ignore dynamic bodies
    #[inline]
    #[must_use]
    pub const fn with_ignore_dynamic(mut self, ignore: bool) -> Self {
        self.ignore_dynamic = ignore;
        self
    }

    /// Set whether to ignore kinematic bodies
    #[inline]
    #[must_use]
    pub const fn with_ignore_kinematic(mut self, ignore: bool) -> Self {
        self.ignore_kinematic = ignore;
        self
    }

    /// Set the collision filter mask
    #[inline]
    #[must_use]
    pub const fn with_mask(mut self, mask: u32) -> Self {
        self.mask = mask;
        self
    }
}

impl Default for RayCastOptions {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::collision::ColliderHandle;
    use crate::geometry::Ray;

    #[test]
    fn test_ray_cast() {
        let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        let cast = RayCast::new(ray, 100.0);
        assert_eq!(cast.ray(), ray);
        assert_eq!(cast.max_distance(), 100.0);
    }

    #[test]
    fn test_ray_cast_default() {
        let cast = RayCast::default();
        assert_eq!(cast.max_distance(), 100.0);
    }

    #[test]
    fn test_ray_cast_setters() {
        let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        let mut cast = RayCast::new(ray, 100.0);
        
        let new_ray = Ray::new(Vec2::new(1.0, 2.0), Vec2::new(0.0, 1.0));
        cast.set_ray(new_ray);
        assert_eq!(cast.ray(), new_ray);
        
        cast.set_max_distance(50.0);
        assert_eq!(cast.max_distance(), 50.0);
    }

    #[test]
    fn test_ray_cast_display() {
        let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
        let cast = RayCast::new(ray, 100.0);
        let cast_str = format!("{}", cast);
        assert!(cast_str.contains("RayCast"));
    }

    #[test]
    fn test_ray_cast_result() {
        let result = RayCastResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            5.0,
            0.5,
        );

        assert_eq!(result.body_handle(), BodyHandle::new(1, 0));
        assert_eq!(result.collider_handle(), ColliderHandle::new(1, 0));
        assert_eq!(result.hit_point(), Vec2::new(1.0, 2.0));
        assert_eq!(result.hit_normal(), Vec2::new(0.0, 1.0));
        assert_eq!(result.distance(), 5.0);
        assert_eq!(result.fraction(), 0.5);
    }

    #[test]
    fn test_ray_cast_result_default() {
        let result = RayCastResult::default();
        assert!(result.body_handle().is_null());
        assert!(result.collider_handle().is_null());
        assert_eq!(result.distance(), 0.0);
        assert_eq!(result.fraction(), 0.0);
    }

    #[test]
    fn test_ray_cast_result_display() {
        let result = RayCastResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            5.0,
            0.5,
        );

        let result_str = format!("{}", result);
        assert!(result_str.contains("RayCastResult"));
    }

    #[test]
    fn test_ray_cast_options() {
        let options = RayCastOptions::new()
            .with_ignore_sensors(true)
            .with_ignore_static(true)
            .with_mask(0xFF);

        assert!(options.ignore_sensors);
        assert!(options.ignore_static);
        assert!(!options.ignore_dynamic);
        assert_eq!(options.mask, 0xFF);
    }

    #[test]
    fn test_ray_cast_options_default() {
        let options = RayCastOptions::default();
        assert!(!options.ignore_sensors);
        assert!(!options.ignore_static);
        assert!(!options.ignore_dynamic);
        assert!(!options.ignore_kinematic);
    }
}
