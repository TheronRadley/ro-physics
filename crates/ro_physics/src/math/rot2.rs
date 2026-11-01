//! 2D Rotation implementation for physics calculations.
//!
//! This module provides a rotation type that represents 2D rotations
//! and provides all operations needed for rigid-body physics.

use std::fmt;
use std::ops::{Mul, MulAssign, Neg};

use crate::constants::{EPSILON, PI, TWO_PI};
use crate::math::{utils::approx_eq, Vec2};

/// A 2D rotation represented by an angle in radians.
///
/// Rotations are stored as a single f32 angle, but provide methods
/// for working with sine, cosine, and matrix representations.
///
/// # Memory Layout
///
/// The rotation is stored as a single f32, making it compact and efficient.
/// For performance-critical operations, sine and cosine are cached
/// when computed.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Rot2 {
    /// Angle in radians
    angle: f32,
    /// Cached sine value (None if not computed)
    sin: Option<f32>,
    /// Cached cosine value (None if not computed)
    cos: Option<f32>,
}

impl Rot2 {
    /// Create a new rotation from an angle in radians
    #[inline]
    #[must_use]
    pub const fn new(angle: f32) -> Self {
        Self {
            angle,
            sin: None,
            cos: None,
        }
    }

    /// Create an identity rotation (0 radians)
    #[inline]
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            angle: 0.0,
            sin: Some(0.0),
            cos: Some(1.0),
        }
    }

    /// Get the angle in radians
    #[inline]
    #[must_use]
    pub fn angle(self) -> f32 {
        self.angle
    }

    /// Get the sine of the rotation angle
    #[inline]
    #[must_use]
    pub fn sin(self) -> f32 {
        self.sin.unwrap_or_else(|| self.angle.sin())
    }

    /// Get the cosine of the rotation angle
    #[inline]
    #[must_use]
    pub fn cos(self) -> f32 {
        self.cos.unwrap_or_else(|| self.angle.cos())
    }

    /// Get both sine and cosine of the rotation angle
    ///
    /// This is more efficient than calling sin() and cos() separately
    /// as it computes both at once.
    #[inline]
    #[must_use]
    pub fn sin_cos(self) -> (f32, f32) {
        if let (Some(s), Some(c)) = (self.sin, self.cos) {
            (s, c)
        } else {
            (self.angle.sin(), self.angle.cos())
        }
    }

    /// Check if this is an identity rotation
    ///
    /// Uses [`EPSILON`][crate::constants::EPSILON] as the tolerance.
    #[inline]
    #[must_use]
    pub fn is_identity(self) -> bool {
        self.is_identity_eps(EPSILON)
    }

    /// Check if this is an identity rotation within an explicit tolerance
    #[inline]
    #[must_use]
    pub fn is_identity_eps(self, epsilon: f32) -> bool {
        let wrapped = self.normalized().angle();
        wrapped.abs() <= epsilon
    }

    /// Invert the rotation
    #[inline]
    #[must_use]
    pub fn inverse(self) -> Self {
        Self {
            angle: -self.angle,
            sin: self.sin.map(|s| -s),
            cos: self.cos,
        }
    }

    /// Rotate a vector by this rotation
    #[inline]
    #[must_use]
    pub fn rotate(self, vector: Vec2) -> Vec2 {
        let (s, c) = self.sin_cos();
        Vec2::new(vector.x * c - vector.y * s, vector.x * s + vector.y * c)
    }

    /// Rotate a vector in place
    #[inline]
    pub fn rotate_mut(&self, vector: &mut Vec2) {
        *vector = self.rotate(*vector);
    }

    /// Create a rotation from a vector direction
    ///
    /// The rotation will point in the direction of the vector.
    #[inline]
    #[must_use]
    pub fn from_vector(vector: Vec2) -> Self {
        Self::new(vector.angle())
    }

    /// Create a rotation from one vector to another
    ///
    /// The rotation that transforms `from` to `to`.
    #[inline]
    #[must_use]
    pub fn from_to(from: Vec2, to: Vec2) -> Self {
        let angle = to.angle() - from.angle();
        Self::new(angle)
    }

    /// Compose this rotation with another (self * other)
    ///
    /// This is equivalent to rotating by `self` then by `other`.
    #[inline]
    #[must_use]
    pub fn compose(self, other: Self) -> Self {
        Self::new(self.angle + other.angle)
    }

    /// Linear interpolation to another rotation
    ///
    /// Interpolates along the shortest arc between the two angles.
    #[inline]
    #[must_use]
    pub fn lerp(self, other: Self, t: f32) -> Self {
        // Use the shortest path for interpolation
        let delta = wrap_angle(other.angle - self.angle);
        Self::new(self.angle + delta * t)
    }

    /// Spherical linear interpolation (slerp) to another rotation
    #[inline]
    #[must_use]
    pub fn slerp(self, other: Self, t: f32) -> Self {
        // For 2D, slerp is the same as lerp for angles
        self.lerp(other, t)
    }

    /// Normalize the angle to the range (-PI, PI]
    ///
    /// The normalized rotation is equivalent to `self` but its angle lies in
    /// `(-PI, PI]`. Non-finite angles are returned unchanged.
    #[inline]
    #[must_use]
    pub fn normalized(self) -> Self {
        Self {
            angle: wrap_angle(self.angle),
            sin: None,
            cos: None,
        }
    }

    /// Check if the rotation is finite (not NaN or infinity)
    #[inline]
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.angle.is_finite()
    }

    /// Validate that the rotation is finite and within reasonable bounds
    #[inline]
    #[must_use]
    pub fn validate(self) -> bool {
        if !self.is_finite() {
            return false;
        }
        self.angle.abs() < crate::constants::MAX_FLOAT
    }
}

/// Wrap an angle into the range `(-PI, PI]`.
///
/// Non-finite input is returned unchanged. The boundary is inclusive on the
/// positive side so that `wrap_angle(PI) == PI`, which makes interpolation
/// between `-PI/2` and `PI/2` take the short arc through zero.
#[inline]
#[must_use]
pub(crate) fn wrap_angle(angle: f32) -> f32 {
    if !angle.is_finite() {
        return angle;
    }

    let wrapped = (angle + PI) % TWO_PI;
    let wrapped = if wrapped <= 0.0 {
        wrapped + TWO_PI
    } else {
        wrapped
    };

    wrapped - PI
}

impl Default for Rot2 {
    fn default() -> Self {
        Self::identity()
    }
}

impl fmt::Display for Rot2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Rot2({:.4} rad)", self.angle)
    }
}

impl Mul for Rot2 {
    type Output = Self;

    #[inline]
    fn mul(self, other: Self) -> Self::Output {
        self.compose(other)
    }
}

impl MulAssign for Rot2 {
    #[inline]
    fn mul_assign(&mut self, other: Self) {
        *self = *self * other;
    }
}

impl Mul<Vec2> for Rot2 {
    type Output = Vec2;

    #[inline]
    fn mul(self, vector: Vec2) -> Self::Output {
        self.rotate(vector)
    }
}

impl std::ops::Add for Rot2 {
    type Output = Self;

    /// Compose two rotations (add their angles)
    #[inline]
    fn add(self, other: Self) -> Self::Output {
        Self::new(self.angle + other.angle)
    }
}

impl std::ops::AddAssign for Rot2 {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        *self = *self + other;
    }
}

impl std::ops::Sub for Rot2 {
    type Output = Self;

    /// Subtract another rotation (subtract the angles)
    #[inline]
    fn sub(self, other: Self) -> Self::Output {
        Self::new(self.angle - other.angle)
    }
}

impl std::ops::SubAssign for Rot2 {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        *self = *self - other;
    }
}

impl Neg for Rot2 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        self.inverse()
    }
}

impl PartialEq for Rot2 {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        // Compare angles modulo 2*PI: `wrap_angle` maps the difference into
        // (-PI, PI], where a difference of a whole turn becomes zero.
        let delta = wrap_angle(self.angle - other.angle);
        approx_eq(delta, 0.0)
    }
}

impl Eq for Rot2 {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::PI_OVER_TWO;
    use crate::math::utils::{approx_eq, approx_eq_eps};

    #[test]
    fn test_construction() {
        let r1 = Rot2::new(PI_OVER_TWO);
        assert!(approx_eq(r1.angle(), PI_OVER_TWO));

        let r2 = Rot2::identity();
        assert!(approx_eq(r2.angle(), 0.0));
        assert!(r2.is_identity());
    }

    #[test]
    fn test_trigonometry() {
        let r = Rot2::new(PI_OVER_TWO);
        assert!(approx_eq(r.sin(), 1.0));
        assert!(approx_eq_eps(r.cos(), 0.0, 1e-6));

        let (s, c) = r.sin_cos();
        assert!(approx_eq(s, 1.0));
        assert!(approx_eq_eps(c, 0.0, 1e-6));
    }

    #[test]
    fn test_inverse() {
        let r1 = Rot2::new(PI_OVER_TWO);
        let r2 = r1.inverse();
        assert!(approx_eq(r2.angle(), -PI_OVER_TWO));
        assert!(approx_eq(r2.sin(), -1.0));
        assert!(approx_eq_eps(r2.cos(), 0.0, 1e-6));

        // Composing a rotation with its inverse gives identity
        let composed = r1 * r2;
        assert!(composed.is_identity());
    }

    #[test]
    fn test_rotate() {
        let r = Rot2::new(PI_OVER_TWO);

        let v1 = Vec2::new(1.0, 0.0);
        let rotated = r.rotate(v1);
        assert!(approx_eq_eps(rotated.x, 0.0, 1e-5));
        assert!(approx_eq_eps(rotated.y, 1.0, 1e-5));

        let v2 = Vec2::new(0.0, 1.0);
        let rotated2 = r.rotate(v2);
        assert!(approx_eq_eps(rotated2.x, -1.0, 1e-5));
        assert!(approx_eq_eps(rotated2.y, 0.0, 1e-5));
    }

    #[test]
    fn test_compose() {
        let r1 = Rot2::new(PI_OVER_TWO);
        let r2 = Rot2::new(PI_OVER_TWO);

        let composed = r1.compose(r2);
        assert!(approx_eq_eps(composed.angle(), PI, 1e-5));

        let v = Vec2::new(1.0, 0.0);
        let rotated1 = r1.rotate(v);
        let rotated2 = r2.rotate(rotated1);
        let rotated_composed = composed.rotate(v);

        assert!(approx_eq_eps(rotated2.x, rotated_composed.x, 1e-5));
        assert!(approx_eq_eps(rotated2.y, rotated_composed.y, 1e-5));
    }

    #[test]
    fn test_from_vector() {
        let v1 = Vec2::new(1.0, 0.0);
        let r1 = Rot2::from_vector(v1);
        assert!(r1.is_identity());

        let v2 = Vec2::new(0.0, 1.0);
        let r2 = Rot2::from_vector(v2);
        assert!(approx_eq_eps(r2.angle(), PI_OVER_TWO, 1e-5));
    }

    #[test]
    fn test_from_to() {
        let from = Vec2::new(1.0, 0.0);
        let to = Vec2::new(0.0, 1.0);
        let r = Rot2::from_to(from, to);
        assert!(approx_eq_eps(r.angle(), PI_OVER_TWO, 1e-5));

        // Verify it transforms from to to
        let rotated = r.rotate(from);
        assert!(approx_eq_eps(rotated.x, to.x, 1e-5));
        assert!(approx_eq_eps(rotated.y, to.y, 1e-5));
    }

    #[test]
    fn test_lerp() {
        let r1 = Rot2::new(0.0);
        let r2 = Rot2::new(PI);

        let mid = r1.lerp(r2, 0.5);
        assert!(approx_eq_eps(mid.angle(), PI_OVER_TWO, 1e-5));

        // Test shortest path
        let r3 = Rot2::new(-PI_OVER_TWO);
        let r4 = Rot2::new(PI_OVER_TWO);
        let mid2 = r3.lerp(r4, 0.5);
        assert!(mid2.is_identity_eps(1e-5));
    }

    #[test]
    fn test_normalized() {
        // Angles are wrapped into (-PI, PI]
        let normalized = Rot2::new(3.0 * PI).normalized();
        assert!(approx_eq_eps(normalized.angle(), PI, 1e-5));

        let normalized = Rot2::new(3.0 * PI_OVER_TWO).normalized();
        assert!(approx_eq_eps(normalized.angle(), -PI_OVER_TWO, 1e-5));

        let normalized = Rot2::new(-3.0 * PI_OVER_TWO).normalized();
        assert!(approx_eq_eps(normalized.angle(), PI_OVER_TWO, 1e-5));

        // A whole number of turns is an identity rotation
        let normalized = Rot2::new(2.0 * TWO_PI).normalized();
        assert!(normalized.is_identity_eps(1e-5));
    }

    #[test]
    fn test_operators() {
        let r1 = Rot2::new(PI_OVER_TWO);
        let r2 = Rot2::new(PI_OVER_TWO);

        // Multiplication (composition)
        let product = r1 * r2;
        assert!(approx_eq_eps(product.angle(), PI, 1e-5));

        // Multiplication with vector
        let v = Vec2::new(1.0, 0.0);
        let rotated = r1 * v;
        assert!(approx_eq_eps(rotated.x, 0.0, 1e-5));
        assert!(approx_eq_eps(rotated.y, 1.0, 1e-5));

        // Negation
        let neg = -r1;
        assert!(approx_eq(neg.angle(), -PI_OVER_TWO));
    }

    #[test]
    fn test_equality() {
        let r1 = Rot2::new(0.0);
        let r2 = Rot2::new(TWO_PI);
        let r3 = Rot2::new(PI);

        assert_eq!(r1, r2);
        assert_ne!(r1, r3);
    }

    #[test]
    fn test_validation() {
        let r1 = Rot2::new(PI_OVER_TWO);
        assert!(r1.is_finite());
        assert!(r1.validate());

        let r2 = Rot2::new(f32::NAN);
        assert!(!r2.is_finite());
        assert!(!r2.validate());

        let r3 = Rot2::new(f32::INFINITY);
        assert!(!r3.is_finite());
        assert!(!r3.validate());

        let r4 = Rot2::new(1e30);
        assert!(r4.is_finite());
        assert!(!r4.validate());
    }

    #[test]
    fn test_identity_rotation() {
        let r = Rot2::identity();
        assert!(r.is_identity());

        let v = Vec2::new(1.0, 2.0);
        let rotated = r.rotate(v);
        assert_eq!(rotated, v);
    }

    #[test]
    fn test_rotation_operators() {
        let mut r = Rot2::identity();
        let other = Rot2::new(PI_OVER_TWO);

        r *= other;
        assert!(approx_eq(r.angle(), PI_OVER_TWO));
    }
}
