//! 2D Vector implementation for physics calculations.
//!
//! This module provides a robust 2D vector type with all operations
//! needed for rigid-body physics simulation.

use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::constants::EPSILON;
use crate::math::utils::approx_eq;

/// A 2D vector for physics calculations.
///
/// This is the fundamental building block for all geometric and physical
/// calculations in the engine. It provides comprehensive vector operations
/// optimized for physics simulation.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec2 {
    /// X component
    pub x: f32,
    /// Y component
    pub y: f32,
}

impl Vec2 {
    /// Create a new vector with the given components
    #[inline]
    #[must_use]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Create a zero vector
    #[inline]
    #[must_use]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Create a vector with both components set to 1
    #[inline]
    #[must_use]
    pub const fn one() -> Self {
        Self { x: 1.0, y: 1.0 }
    }

    /// Create a unit vector pointing in the X direction
    #[inline]
    #[must_use]
    pub const fn unit_x() -> Self {
        Self { x: 1.0, y: 0.0 }
    }

    /// Create a unit vector pointing in the Y direction
    #[inline]
    #[must_use]
    pub const fn unit_y() -> Self {
        Self { x: 0.0, y: 1.0 }
    }

    /// Create a vector from a single scalar (both components set to the same value)
    #[inline]
    #[must_use]
    pub const fn splat(scalar: f32) -> Self {
        Self {
            x: scalar,
            y: scalar,
        }
    }

    /// Get the X component
    #[inline]
    #[must_use]
    pub const fn x(self) -> f32 {
        self.x
    }

    /// Get the Y component
    #[inline]
    #[must_use]
    pub const fn y(self) -> f32 {
        self.y
    }

    /// Set both components
    #[inline]
    pub fn set(&mut self, x: f32, y: f32) {
        self.x = x;
        self.y = y;
    }

    /// Calculate the squared length (magnitude) of the vector
    ///
    /// This avoids the square root operation and is useful for comparisons.
    #[inline]
    #[must_use]
    pub fn length_squared(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Calculate the length (magnitude) of the vector
    #[inline]
    #[must_use]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    /// Check if the vector is approximately zero
    ///
    /// Uses [`EPSILON`][crate::constants::EPSILON] as the tolerance.
    #[inline]
    #[must_use]
    pub fn is_zero(self) -> bool {
        self.length_squared() <= EPSILON * EPSILON
    }

    /// Check if the vector is approximately zero with an explicit tolerance
    #[inline]
    #[must_use]
    pub fn is_zero_eps(self, epsilon: f32) -> bool {
        self.length_squared() <= epsilon * epsilon
    }

    /// Largest component of the vector
    #[inline]
    #[must_use]
    pub fn max_element(self) -> f32 {
        self.x.max(self.y)
    }

    /// Smallest component of the vector
    #[inline]
    #[must_use]
    pub fn min_element(self) -> f32 {
        self.x.min(self.y)
    }

    /// Normalize the vector to unit length
    ///
    /// Returns a zero vector if the original vector is zero.
    /// Uses safe division to avoid NaN from division by zero.
    #[inline]
    #[must_use]
    pub fn normalize(self) -> Self {
        let len_sq = self.length_squared();
        if len_sq > EPSILON * EPSILON {
            let inv_len = 1.0 / len_sq.sqrt();
            Self {
                x: self.x * inv_len,
                y: self.y * inv_len,
            }
        } else {
            Self::zero()
        }
    }

    /// Normalize the vector in place
    #[inline]
    pub fn normalize_mut(&mut self) {
        *self = self.normalize();
    }

    /// Calculate the dot product with another vector
    ///
    /// The dot product is commutative: a.dot(b) == b.dot(a)
    #[inline]
    #[must_use]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// Calculate the 2D cross product with another vector
    ///
    /// In 2D, the cross product of two vectors is a scalar (the z-component).
    /// The result is positive if `other` is counter-clockwise from `self`.
    ///
    /// This is equivalent to the determinant of the 2x2 matrix formed by the vectors.
    #[inline]
    #[must_use]
    pub fn cross(self, other: Self) -> f32 {
        self.x * other.y - self.y * other.x
    }

    /// Calculate the cross product with a scalar (treated as z-component)
    ///
    /// This returns a vector perpendicular to `self` with magnitude |self| * |scalar|.
    /// Positive scalar gives counter-clockwise rotation, negative gives clockwise.
    #[inline]
    #[must_use]
    pub fn cross_scalar(self, scalar: f32) -> Self {
        Self {
            x: -self.y * scalar,
            y: self.x * scalar,
        }
    }

    /// Calculate the perpendicular vector (90-degree counter-clockwise rotation)
    #[inline]
    #[must_use]
    pub fn perp(self) -> Self {
        Self {
            x: -self.y,
            y: self.x,
        }
    }

    /// Calculate the perpendicular vector (90-degree clockwise rotation)
    #[inline]
    #[must_use]
    pub fn perp_neg(self) -> Self {
        Self {
            x: self.y,
            y: -self.x,
        }
    }

    /// Calculate the distance squared to another vector
    #[inline]
    #[must_use]
    pub fn distance_squared(self, other: Self) -> f32 {
        (self.x - other.x).powi(2) + (self.y - other.y).powi(2)
    }

    /// Calculate the distance to another vector
    #[inline]
    #[must_use]
    pub fn distance(self, other: Self) -> f32 {
        self.distance_squared(other).sqrt()
    }

    /// Linear interpolation to another vector
    #[inline]
    #[must_use]
    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
        }
    }

    /// Calculate the absolute value of each component
    #[inline]
    #[must_use]
    pub fn abs(self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
        }
    }

    /// Calculate the minimum of each component with another vector
    #[inline]
    #[must_use]
    pub fn min(self, other: Self) -> Self {
        Self {
            x: self.x.min(other.x),
            y: self.y.min(other.y),
        }
    }

    /// Calculate the maximum of each component with another vector
    #[inline]
    #[must_use]
    pub fn max(self, other: Self) -> Self {
        Self {
            x: self.x.max(other.x),
            y: self.y.max(other.y),
        }
    }

    /// Clamp each component to a range
    #[inline]
    #[must_use]
    pub fn clamp(self, min: Self, max: Self) -> Self {
        Self {
            x: self.x.clamp(min.x, max.x),
            y: self.y.clamp(min.y, max.y),
        }
    }

    /// Project this vector onto another vector
    ///
    /// Returns the projection scalar: (self * other) / other.length()
    #[inline]
    #[must_use]
    pub fn project_onto_scalar(self, other: Self) -> f32 {
        self.dot(other) / other.length()
    }

    /// Project this vector onto another vector
    ///
    /// Returns the projection vector.
    #[inline]
    #[must_use]
    pub fn project_onto(self, other: Self) -> Self {
        let other_len_sq = other.length_squared();
        if other_len_sq < EPSILON * EPSILON {
            Self::zero()
        } else {
            let scale = self.dot(other) / other_len_sq;
            Self {
                x: other.x * scale,
                y: other.y * scale,
            }
        }
    }

    /// Reject this vector from another vector (orthogonal component)
    ///
    /// Returns self - self.project_onto(other)
    #[inline]
    #[must_use]
    pub fn reject_from(self, other: Self) -> Self {
        self - self.project_onto(other)
    }

    /// Calculate the angle (in radians) from the positive X axis
    #[inline]
    #[must_use]
    pub fn angle(self) -> f32 {
        self.y.atan2(self.x)
    }

    /// Calculate the angle between this vector and another
    #[inline]
    #[must_use]
    pub fn angle_between(self, other: Self) -> f32 {
        let dot = self.dot(other);
        let len_product = (self.length_squared() * other.length_squared()).sqrt();
        if len_product < EPSILON {
            0.0
        } else {
            (dot / len_product).clamp(-1.0, 1.0).acos()
        }
    }

    /// Rotate this vector by an angle (in radians)
    ///
    /// Positive angle rotates counter-clockwise.
    #[inline]
    #[must_use]
    pub fn rotate(self, angle: f32) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }

    /// Check if all components are finite (not NaN or infinity)
    #[inline]
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }

    /// Check if any component is NaN
    #[inline]
    #[must_use]
    pub fn is_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan()
    }

    /// Validate that the vector is finite and within reasonable bounds
    #[inline]
    #[must_use]
    pub fn validate(self) -> bool {
        if !self.is_finite() {
            return false;
        }
        self.x.abs() < crate::constants::MAX_FLOAT
            && self.y.abs() < crate::constants::MAX_FLOAT
    }

    /// Convert to an array
    #[inline]
    #[must_use]
    pub fn to_array(self) -> [f32; 2] {
        [self.x, self.y]
    }

    /// Create from an array
    #[inline]
    #[must_use]
    pub fn from_array(arr: [f32; 2]) -> Self {
        Self {
            x: arr[0],
            y: arr[1],
        }
    }
}

impl Default for Vec2 {
    fn default() -> Self {
        Self::zero()
    }
}

impl fmt::Display for Vec2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vec2({:.4}, {:.4})", self.x, self.y)
    }
}

// Operator implementations

impl Add for Vec2 {
    type Output = Self;
    
    #[inline]
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;
    
    #[inline]
    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    
    #[inline]
    fn mul(self, scalar: f32) -> Self::Output {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl MulAssign<f32> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, scalar: f32) {
        self.x *= scalar;
        self.y *= scalar;
    }
}

impl Div<f32> for Vec2 {
    type Output = Self;
    
    #[inline]
    fn div(self, scalar: f32) -> Self::Output {
        let inv_scalar = if scalar.abs() < EPSILON {
            0.0
        } else {
            1.0 / scalar
        };
        Self {
            x: self.x * inv_scalar,
            y: self.y * inv_scalar,
        }
    }
}

impl DivAssign<f32> for Vec2 {
    #[inline]
    fn div_assign(&mut self, scalar: f32) {
        *self = *self / scalar;
    }
}

impl Mul<Vec2> for f32 {
    type Output = Vec2;
    
    #[inline]
    fn mul(self, vector: Vec2) -> Self::Output {
        vector * self
    }
}

impl Neg for Vec2 {
    type Output = Self;
    
    #[inline]
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl PartialEq for Vec2 {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        approx_eq(self.x, other.x) && approx_eq(self.y, other.y)
    }
}

impl Eq for Vec2 {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::{approx_eq, approx_eq_eps};

    #[test]
    fn test_construction() {
        let v1 = Vec2::new(1.0, 2.0);
        assert_eq!(v1.x, 1.0);
        assert_eq!(v1.y, 2.0);

        let v2 = Vec2::zero();
        assert_eq!(v2.x, 0.0);
        assert_eq!(v2.y, 0.0);

        let v3 = Vec2::unit_x();
        assert_eq!(v3.x, 1.0);
        assert_eq!(v3.y, 0.0);

        let v4 = Vec2::unit_y();
        assert_eq!(v4.x, 0.0);
        assert_eq!(v4.y, 1.0);
    }

    #[test]
    fn test_length() {
        let v1 = Vec2::new(3.0, 4.0);
        assert!(approx_eq(v1.length(), 5.0));
        assert!(approx_eq(v1.length_squared(), 25.0));

        let v2 = Vec2::zero();
        assert!(approx_eq(v2.length(), 0.0));
        assert!(approx_eq(v2.length_squared(), 0.0));
    }

    #[test]
    fn test_normalize() {
        let v1 = Vec2::new(3.0, 4.0);
        let normalized = v1.normalize();
        assert!(approx_eq(normalized.length(), 1.0));
        assert!(approx_eq(normalized.x, 0.6));
        assert!(approx_eq(normalized.y, 0.8));

        let v2 = Vec2::zero();
        let normalized_zero = v2.normalize();
        assert!(normalized_zero.is_zero());
    }

    #[test]
    fn test_dot_product() {
        let v1 = Vec2::new(1.0, 2.0);
        let v2 = Vec2::new(3.0, 4.0);
        assert!(approx_eq(v1.dot(v2), 11.0));

        // Dot product is commutative
        assert!(approx_eq(v1.dot(v2), v2.dot(v1)));

        // Dot product with perpendicular vectors is 0
        let v3 = Vec2::new(1.0, 0.0);
        let v4 = Vec2::new(0.0, 1.0);
        assert!(approx_eq(v3.dot(v4), 0.0));
    }

    #[test]
    fn test_cross_product() {
        let v1 = Vec2::new(1.0, 0.0);
        let v2 = Vec2::new(0.0, 1.0);
        assert!(approx_eq(v1.cross(v2), 1.0));

        let v3 = Vec2::new(1.0, 2.0);
        let v4 = Vec2::new(3.0, 4.0);
        assert!(approx_eq(v3.cross(v4), -2.0));
        assert!(approx_eq(v4.cross(v3), 2.0));
    }

    #[test]
    fn test_cross_scalar() {
        let v = Vec2::new(1.0, 0.0);
        let result = v.cross_scalar(1.0);
        assert!(approx_eq(result.x, 0.0));
        assert!(approx_eq(result.y, 1.0));

        let result2 = v.cross_scalar(-1.0);
        assert!(approx_eq(result2.x, 0.0));
        assert!(approx_eq(result2.y, -1.0));
    }

    #[test]
    fn test_perp() {
        let v1 = Vec2::new(1.0, 0.0);
        let perp = v1.perp();
        assert!(approx_eq(perp.x, 0.0));
        assert!(approx_eq(perp.y, 1.0));

        let v2 = Vec2::new(0.0, 1.0);
        let perp2 = v2.perp();
        assert!(approx_eq(perp2.x, -1.0));
        assert!(approx_eq(perp2.y, 0.0));

        // Perpendicular vectors should be orthogonal
        let v3 = Vec2::new(3.0, 4.0);
        let perp3 = v3.perp();
        assert!(approx_eq(v3.dot(perp3), 0.0));
    }

    #[test]
    fn test_operators() {
        let v1 = Vec2::new(1.0, 2.0);
        let v2 = Vec2::new(3.0, 4.0);

        // Addition
        let sum = v1 + v2;
        assert_eq!(sum, Vec2::new(4.0, 6.0));

        // Subtraction
        let diff = v2 - v1;
        assert_eq!(diff, Vec2::new(2.0, 2.0));

        // Scalar multiplication
        let scaled = v1 * 2.0;
        assert_eq!(scaled, Vec2::new(2.0, 4.0));

        // Scalar division
        let divided = v2 / 2.0;
        assert_eq!(divided, Vec2::new(1.5, 2.0));

        // Negation
        let neg = -v1;
        assert_eq!(neg, Vec2::new(-1.0, -2.0));
    }

    #[test]
    fn test_assign_operators() {
        let mut v1 = Vec2::new(1.0, 2.0);
        let v2 = Vec2::new(3.0, 4.0);

        v1 += v2;
        assert_eq!(v1, Vec2::new(4.0, 6.0));

        v1 -= v2;
        assert_eq!(v1, Vec2::new(1.0, 2.0));

        v1 *= 2.0;
        assert_eq!(v1, Vec2::new(2.0, 4.0));

        v1 /= 2.0;
        assert_eq!(v1, Vec2::new(1.0, 2.0));
    }

    #[test]
    fn test_distance() {
        let v1 = Vec2::new(0.0, 0.0);
        let v2 = Vec2::new(3.0, 4.0);
        assert!(approx_eq(v1.distance(v2), 5.0));
        assert!(approx_eq(v1.distance_squared(v2), 25.0));
    }

    #[test]
    fn test_lerp() {
        let v1 = Vec2::new(0.0, 0.0);
        let v2 = Vec2::new(10.0, 10.0);
        
        let mid = v1.lerp(v2, 0.5);
        assert_eq!(mid, Vec2::new(5.0, 5.0));

        let quarter = v1.lerp(v2, 0.25);
        assert_eq!(quarter, Vec2::new(2.5, 2.5));
    }

    #[test]
    fn test_rotate() {
        let v = Vec2::new(1.0, 0.0);
        
        // Rotate 90 degrees (PI/2)
        let rotated_90 = v.rotate(crate::constants::PI_OVER_TWO);
        assert!(approx_eq_eps(rotated_90.x, 0.0, 1e-5));
        assert!(approx_eq_eps(rotated_90.y, 1.0, 1e-5));

        // Rotate 180 degrees (PI)
        let rotated_180 = v.rotate(crate::constants::PI);
        assert!(approx_eq_eps(rotated_180.x, -1.0, 1e-5));
        assert!(approx_eq_eps(rotated_180.y, 0.0, 1e-5));
    }

    #[test]
    fn test_angle() {
        let v1 = Vec2::new(1.0, 0.0);
        assert!(approx_eq_eps(v1.angle(), 0.0, 1e-5));

        let v2 = Vec2::new(0.0, 1.0);
        assert!(approx_eq_eps(v2.angle(), crate::constants::PI_OVER_TWO, 1e-5));

        let v3 = Vec2::new(-1.0, 0.0);
        assert!(approx_eq_eps(v3.angle(), crate::constants::PI, 1e-5));

        let v4 = Vec2::new(0.0, -1.0);
        assert!(approx_eq_eps(v4.angle(), -crate::constants::PI_OVER_TWO, 1e-5));
    }

    #[test]
    fn test_angle_between() {
        let v1 = Vec2::new(1.0, 0.0);
        let v2 = Vec2::new(0.0, 1.0);
        assert!(approx_eq_eps(v1.angle_between(v2), crate::constants::PI_OVER_TWO, 1e-5));

        let v3 = Vec2::new(1.0, 0.0);
        let v4 = Vec2::new(-1.0, 0.0);
        assert!(approx_eq_eps(v3.angle_between(v4), crate::constants::PI, 1e-5));
    }

    #[test]
    fn test_projection() {
        let v1 = Vec2::new(3.0, 4.0);
        let v2 = Vec2::new(1.0, 0.0);
        
        let proj_scalar = v1.project_onto_scalar(v2);
        assert!(approx_eq(proj_scalar, 3.0));

        let proj_vec = v1.project_onto(v2);
        assert_eq!(proj_vec, Vec2::new(3.0, 0.0));

        let reject = v1.reject_from(v2);
        assert_eq!(reject, Vec2::new(0.0, 4.0));
    }

    #[test]
    fn test_validation() {
        let v1 = Vec2::new(1.0, 2.0);
        assert!(v1.is_finite());
        assert!(!v1.is_nan());
        assert!(v1.validate());

        let v2 = Vec2::new(f32::NAN, 0.0);
        assert!(!v2.is_finite());
        assert!(v2.is_nan());
        assert!(!v2.validate());

        let v3 = Vec2::new(f32::INFINITY, 0.0);
        assert!(!v3.is_finite());
        assert!(!v3.is_nan());
        assert!(!v3.validate());

        let v4 = Vec2::new(1e30, 1e30);
        assert!(v4.is_finite());
        assert!(!v4.validate());
    }

    #[test]
    fn test_array_conversion() {
        let v = Vec2::new(1.0, 2.0);
        let arr = v.to_array();
        assert_eq!(arr, [1.0, 2.0]);

        let v2 = Vec2::from_array(arr);
        assert_eq!(v2, v);
    }

    #[test]
    fn test_clamp() {
        let v = Vec2::new(5.0, -5.0);
        let min = Vec2::new(0.0, -10.0);
        let max = Vec2::new(10.0, 0.0);
        
        let clamped = v.clamp(min, max);
        assert_eq!(clamped, Vec2::new(5.0, -5.0));

        let v2 = Vec2::new(-5.0, 5.0);
        let clamped2 = v2.clamp(min, max);
        assert_eq!(clamped2, Vec2::new(0.0, 0.0));
    }

    #[test]
    fn test_min_max() {
        let v1 = Vec2::new(1.0, 5.0);
        let v2 = Vec2::new(3.0, 2.0);
        
        let min = v1.min(v2);
        assert_eq!(min, Vec2::new(1.0, 2.0));

        let max = v1.max(v2);
        assert_eq!(max, Vec2::new(3.0, 5.0));
    }

    #[test]
    fn test_abs() {
        let v = Vec2::new(-1.0, -2.0);
        let abs = v.abs();
        assert_eq!(abs, Vec2::new(1.0, 2.0));
    }
}
