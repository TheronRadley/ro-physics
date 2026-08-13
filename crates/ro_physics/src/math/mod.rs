//! Mathematical foundations for the physics engine.
//!
//! This module provides the core mathematical types and operations used
//! throughout the physics simulation, including 2D vectors, rotations,
//! and transforms.

mod vec2;
mod rot2;
mod transform;

pub use vec2::Vec2;
pub use rot2::Rot2;
pub use transform::Transform;

/// Mathematical utilities and helper functions
pub mod utils {
    use super::Vec2;
    use crate::constants::EPSILON;

    /// Check if two floating-point values are approximately equal
    #[inline]
    #[must_use]
    pub fn approx_eq(a: f32, b: f32, epsilon: f32 = EPSILON) -> bool {
        (a - b).abs() <= epsilon
    }

    /// Check if a value is approximately zero
    #[inline]
    #[must_use]
    pub fn approx_zero(a: f32, epsilon: f32 = EPSILON) -> bool {
        a.abs() <= epsilon
    }

    /// Clamp a value to a range
    #[inline]
    #[must_use]
    pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
        value.max(min).min(max)
    }

    /// Linear interpolation between two values
    #[inline]
    #[must_use]
    pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }

    /// Linear interpolation between two vectors
    #[inline]
    #[must_use]
    pub fn vec2_lerp(a: Vec2, b: Vec2, t: f32) -> Vec2 {
        Vec2::new(
            lerp(a.x, b.x, t),
            lerp(a.y, b.y, t),
        )
    }

    /// Calculate the sign of a value
    #[inline]
    #[must_use]
    pub fn sign(x: f32) -> f32 {
        if x > 0.0 {
            1.0
        } else if x < 0.0 {
            -1.0
        } else {
            0.0
        }
    }

    /// Safe division that returns 0 if denominator is near zero
    #[inline]
    #[must_use]
    pub fn safe_divide(numerator: f32, denominator: f32) -> f32 {
        if denominator.abs() < EPSILON {
            0.0
        } else {
            numerator / denominator
        }
    }

    /// Check if a value is finite (not NaN or infinity)
    #[inline]
    #[must_use]
    pub fn is_finite(x: f32) -> bool {
        x.is_finite()
    }

    /// Check if a value is NaN
    #[inline]
    #[must_use]
    pub fn is_nan(x: f32) -> bool {
        x.is_nan()
    }

    /// Check if a value is infinity
    #[inline]
    #[must_use]
    pub fn is_infinite(x: f32) -> bool {
        x.is_infinite()
    }

    /// Validate that a value is finite and within reasonable bounds
    #[inline]
    #[must_use]
    pub fn validate_finite(x: f32) -> bool {
        if !x.is_finite() {
            return false;
        }
        x.abs() < crate::constants::MAX_FLOAT
    }

    /// Validate that a vector has finite components
    #[inline]
    #[must_use]
    pub fn validate_vec2(v: Vec2) -> bool {
        validate_finite(v.x) && validate_finite(v.y)
    }
}

/// Re-export utils for convenience
pub use utils::*;
