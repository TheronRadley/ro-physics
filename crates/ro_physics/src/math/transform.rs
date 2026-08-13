//! 2D Transform implementation for physics calculations.
//!
//! This module provides a transform type that represents position and rotation
//! in 2D space, with all operations needed for rigid-body physics.

use std::fmt;
use std::ops::{Mul, MulAssign};

use crate::constants::EPSILON;
use crate::math::{utils::approx_eq, Vec2};
use crate::math::{Rot2};

/// A 2D transform representing position and rotation.
///
/// This is the fundamental type for representing the state of rigid bodies
/// in the physics simulation. It supports composition, inversion, and
/// transformation of points and vectors.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Transform {
    /// Position (translation)
    pub position: Vec2,
    /// Rotation
    pub rotation: Rot2,
}

impl Transform {
    /// Create a new transform with the given position and rotation
    #[inline]
    #[must_use]
    pub const fn new(position: Vec2, rotation: Rot2) -> Self {
        Self { position, rotation }
    }

    /// Create an identity transform (position at origin, no rotation)
    #[inline]
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            position: Vec2::zero(),
            rotation: Rot2::identity(),
        }
    }

    /// Create a transform with only translation (no rotation)
    #[inline]
    #[must_use]
    pub fn from_translation(position: Vec2) -> Self {
        Self {
            position,
            rotation: Rot2::identity(),
        }
    }

    /// Create a transform with only rotation (position at origin)
    #[inline]
    #[must_use]
    pub fn from_rotation(rotation: Rot2) -> Self {
        Self {
            position: Vec2::zero(),
            rotation,
        }
    }

    /// Get the position component
    #[inline]
    #[must_use]
    pub const fn position(self) -> Vec2 {
        self.position
    }

    /// Get the rotation component
    #[inline]
    #[must_use]
    pub const fn rotation(self) -> Rot2 {
        self.rotation
    }

    /// Set the position
    #[inline]
    pub fn set_position(&mut self, position: Vec2) {
        self.position = position;
    }

    /// Set the rotation
    #[inline]
    pub fn set_rotation(&mut self, rotation: Rot2) {
        self.rotation = rotation;
    }

    /// Check if this is an identity transform
    #[inline]
    #[must_use]
    pub fn is_identity(self, epsilon: f32 = EPSILON) -> bool {
        self.position.is_zero(epsilon) && self.rotation.is_identity(epsilon)
    }

    /// Transform a point (applies both rotation and translation)
    ///
    /// This is the standard transformation for points in space.
    #[inline]
    #[must_use]
    pub fn transform_point(self, point: Vec2) -> Vec2 {
        self.rotation.rotate(point) + self.position
    }

    /// Transform a vector (applies only rotation, no translation)
    ///
    /// This is used for transforming direction vectors and forces.
    #[inline]
    #[must_use]
    pub fn transform_vector(self, vector: Vec2) -> Vec2 {
        self.rotation.rotate(vector)
    }

    /// Transform a point by the inverse of this transform
    #[inline]
    #[must_use]
    pub fn inverse_transform_point(self, point: Vec2) -> Vec2 {
        self.inverse().transform_point(point)
    }

    /// Transform a vector by the inverse of this transform
    #[inline]
    #[must_use]
    pub fn inverse_transform_vector(self, vector: Vec2) -> Vec2 {
        self.inverse().transform_vector(vector)
    }

    /// Calculate the inverse of this transform
    #[inline]
    #[must_use]
    pub fn inverse(self) -> Self {
        let inv_rotation = self.rotation.inverse();
        let inv_position = inv_rotation.rotate(-self.position);
        Self {
            position: inv_position,
            rotation: inv_rotation,
        }
    }

    /// Compose this transform with another (self * other)
    ///
    /// The resulting transform applies `other` first, then `self`.
    #[inline]
    #[must_use]
    pub fn compose(self, other: Self) -> Self {
        Self {
            position: self.position + self.rotation.rotate(other.position),
            rotation: self.rotation * other.rotation,
        }
    }

    /// Linear interpolation to another transform
    #[inline]
    #[must_use]
    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self {
            position: self.position.lerp(other.position, t),
            rotation: self.rotation.lerp(other.rotation, t),
        }
    }

    /// Calculate the relative transform from this to another
    ///
    /// The result is the transform that, when composed with `self`, gives `other`.
    #[inline]
    #[must_use]
    pub fn relative_to(self, other: Self) -> Self {
        self.inverse().compose(other)
    }

    /// Calculate the distance between two transforms (position difference)
    #[inline]
    #[must_use]
    pub fn distance_to(self, other: Self) -> f32 {
        (other.position - self.position).length()
    }

    /// Check if the transform is finite (no NaN or infinity)
    #[inline]
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.position.is_finite() && self.rotation.is_finite()
    }

    /// Validate that the transform is finite and within reasonable bounds
    #[inline]
    #[must_use]
    pub fn validate(self) -> bool {
        self.position.validate() && self.rotation.validate()
    }

    /// Convert to an affine transformation matrix representation
    ///
    /// Returns [a, b, tx, c, d, ty] where the matrix is:
    /// ```text
    /// [ a  b  tx ]
    /// [ c  d  ty ]
    /// [ 0  0  1  ]
    /// ```
    #[inline]
    #[must_use]
    pub fn to_matrix(self) -> [f32; 6] {
        let (s, c) = self.rotation.sin_cos();
        [
            c, -s, self.position.x,
            s,  c, self.position.y,
        ]
    }

    /// Create a transform from an affine transformation matrix
    #[inline]
    #[must_use]
    pub fn from_matrix(matrix: [f32; 6]) -> Self {
        // Extract rotation from the 2x2 submatrix
        // [a b] = [cos -sin]
        // [c d] = [sin  cos]
        let a = matrix[0];
        let b = matrix[1];
        let c = matrix[3];
        let d = matrix[4];
        
        // Calculate angle from the rotation matrix
        // atan2(sin, cos) = atan2(c, a)
        let angle = c.atan2(a);
        
        Self {
            position: Vec2::new(matrix[2], matrix[5]),
            rotation: Rot2::new(angle),
        }
    }
}

impl Default for Transform {
    fn default() -> Self {
        Self::identity()
    }
}

impl fmt::Display for Transform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Transform(position: {}, rotation: {})",
            self.position, self.rotation
        )
    }
}

impl Mul for Transform {
    type Output = Self;
    
    #[inline]
    fn mul(self, other: Self) -> Self::Output {
        self.compose(other)
    }
}

impl MulAssign for Transform {
    #[inline]
    fn mul_assign(&mut self, other: Self) {
        *self = *self * other;
    }
}

impl Mul<Vec2> for Transform {
    type Output = Vec2;
    
    #[inline]
    fn mul(self, point: Vec2) -> Self::Output {
        self.transform_point(point)
    }
}

impl PartialEq for Transform {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        approx_eq(self.position.x, other.position.x)
            && approx_eq(self.position.y, other.position.y)
            && self.rotation == other.rotation
    }
}

impl Eq for Transform {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{PI, PI_OVER_TWO};

    #[test]
    fn test_construction() {
        let t1 = Transform::identity();
        assert!(t1.position.is_zero());
        assert!(t1.rotation.is_identity());

        let pos = Vec2::new(1.0, 2.0);
        let rot = Rot2::new(PI_OVER_TWO);
        let t2 = Transform::new(pos, rot);
        assert_eq!(t2.position, pos);
        assert_eq!(t2.rotation, rot);

        let t3 = Transform::from_translation(pos);
        assert_eq!(t3.position, pos);
        assert!(t3.rotation.is_identity());

        let t4 = Transform::from_rotation(rot);
        assert!(t4.position.is_zero());
        assert_eq!(t4.rotation, rot);
    }

    #[test]
    fn test_transform_point() {
        let t = Transform::from_translation(Vec2::new(1.0, 2.0));
        let p = Vec2::new(3.0, 4.0);
        let transformed = t.transform_point(p);
        assert_eq!(transformed, Vec2::new(4.0, 6.0));

        let t2 = Transform::from_rotation(Rot2::new(PI_OVER_TWO));
        let p2 = Vec2::new(1.0, 0.0);
        let transformed2 = t2.transform_point(p2);
        assert!(approx_eq(transformed2.x, 0.0, 1e-5));
        assert!(approx_eq(transformed2.y, 1.0, 1e-5));

        // Combined transform
        let t3 = Transform::new(Vec2::new(1.0, 2.0), Rot2::new(PI_OVER_TWO));
        let p3 = Vec2::new(1.0, 0.0);
        let transformed3 = t3.transform_point(p3);
        // Rotate (1,0) by 90 degrees -> (0,1), then translate by (1,2) -> (1,3)
        assert!(approx_eq(transformed3.x, 1.0, 1e-5));
        assert!(approx_eq(transformed3.y, 3.0, 1e-5));
    }

    #[test]
    fn test_transform_vector() {
        let t = Transform::new(Vec2::new(1.0, 2.0), Rot2::new(PI_OVER_TWO));
        let v = Vec2::new(1.0, 0.0);
        let transformed = t.transform_vector(v);
        // Only rotation is applied, no translation
        assert!(approx_eq(transformed.x, 0.0, 1e-5));
        assert!(approx_eq(transformed.y, 1.0, 1e-5));
    }

    #[test]
    fn test_inverse() {
        let t = Transform::new(Vec2::new(1.0, 2.0), Rot2::new(PI_OVER_TWO));
        let inv = t.inverse();

        // Inverse should have negated position (rotated back)
        // and negated rotation
        assert!(approx_eq(inv.rotation.angle(), -PI_OVER_TWO));

        // Test that transform * inverse = identity
        let composed = t * inv;
        assert!(composed.is_identity(1e-5));

        // Test point transformation
        let p = Vec2::new(3.0, 4.0);
        let transformed = t.transform_point(p);
        let back = inv.transform_point(transformed);
        assert!(approx_eq(back.x, p.x, 1e-5));
        assert!(approx_eq(back.y, p.y, 1e-5));
    }

    #[test]
    fn test_compose() {
        let t1 = Transform::from_translation(Vec2::new(1.0, 0.0));
        let t2 = Transform::from_translation(Vec2::new(0.0, 2.0));
        
        let composed = t1.compose(t2);
        let p = Vec2::new(0.0, 0.0);
        let transformed = composed.transform_point(p);
        assert_eq!(transformed, Vec2::new(1.0, 2.0));

        // Test with rotation
        let t3 = Transform::from_rotation(Rot2::new(PI_OVER_TWO));
        let t4 = Transform::from_translation(Vec2::new(1.0, 0.0));
        let composed2 = t3.compose(t4);
        
        let p2 = Vec2::new(1.0, 0.0);
        let transformed2 = composed2.transform_point(p2);
        // First translate (1,0) by (1,0) -> (2,0), then rotate by 90 -> (0,2)
        assert!(approx_eq(transformed2.x, 0.0, 1e-5));
        assert!(approx_eq(transformed2.y, 2.0, 1e-5));
    }

    #[test]
    fn test_lerp() {
        let t1 = Transform::identity();
        let t2 = Transform::new(Vec2::new(10.0, 10.0), Rot2::new(PI));
        
        let mid = t1.lerp(t2, 0.5);
        assert_eq!(mid.position, Vec2::new(5.0, 5.0));
        assert!(approx_eq(mid.rotation.angle(), PI / 2.0, 1e-5));
    }

    #[test]
    fn test_relative_to() {
        let t1 = Transform::from_translation(Vec2::new(1.0, 2.0));
        let t2 = Transform::from_translation(Vec2::new(4.0, 6.0));
        
        let relative = t1.relative_to(t2);
        assert_eq!(relative.position, Vec2::new(3.0, 4.0));
        assert!(relative.rotation.is_identity());
    }

    #[test]
    fn test_distance_to() {
        let t1 = Transform::from_translation(Vec2::new(0.0, 0.0));
        let t2 = Transform::from_translation(Vec2::new(3.0, 4.0));
        
        assert!(approx_eq(t1.distance_to(t2), 5.0));
    }

    #[test]
    fn test_operators() {
        let t1 = Transform::from_translation(Vec2::new(1.0, 0.0));
        let t2 = Transform::from_translation(Vec2::new(0.0, 2.0));

        // Multiplication (composition)
        let product = t1 * t2;
        let p = Vec2::new(0.0, 0.0);
        let transformed = product.transform_point(p);
        assert_eq!(transformed, Vec2::new(1.0, 2.0));

        // Multiplication with point
        let transformed2 = t1 * p;
        assert_eq!(transformed2, Vec2::new(1.0, 0.0));
    }

    #[test]
    fn test_matrix_conversion() {
        let t = Transform::new(Vec2::new(1.0, 2.0), Rot2::new(PI_OVER_TWO));
        let matrix = t.to_matrix();
        
        // Matrix should be:
        // [0 -1 1]
        // [1  0 2]
        // [0  0 1]
        assert!(approx_eq(matrix[0], 0.0, 1e-5));
        assert!(approx_eq(matrix[1], -1.0, 1e-5));
        assert!(approx_eq(matrix[2], 1.0, 1e-5));
        assert!(approx_eq(matrix[3], 1.0, 1e-5));
        assert!(approx_eq(matrix[4], 0.0, 1e-5));
        assert!(approx_eq(matrix[5], 2.0, 1e-5));

        // Test round-trip
        let t2 = Transform::from_matrix(matrix);
        assert!(approx_eq(t2.position.x, 1.0, 1e-5));
        assert!(approx_eq(t2.position.y, 2.0, 1e-5));
        assert!(approx_eq(t2.rotation.angle(), PI_OVER_TWO, 1e-5));
    }

    #[test]
    fn test_equality() {
        let t1 = Transform::identity();
        let t2 = Transform::new(Vec2::new(0.0, 0.0), Rot2::new(0.0));
        let t3 = Transform::new(Vec2::new(1.0, 0.0), Rot2::identity());

        assert_eq!(t1, t2);
        assert_ne!(t1, t3);
    }

    #[test]
    fn test_validation() {
        let t1 = Transform::new(Vec2::new(1.0, 2.0), Rot2::new(PI_OVER_TWO));
        assert!(t1.is_finite());
        assert!(t1.validate());

        let t2 = Transform::new(Vec2::new(f32::NAN, 0.0), Rot2::identity());
        assert!(!t2.is_finite());
        assert!(!t2.validate());

        let t3 = Transform::new(Vec2::new(1.0, 2.0), Rot2::new(f32::INFINITY));
        assert!(!t3.is_finite());
        assert!(!t3.validate());

        let t4 = Transform::new(Vec2::new(1e30, 1e30), Rot2::new(1e30));
        assert!(t4.is_finite());
        assert!(!t4.validate());
    }

    #[test]
    fn test_inverse_transform() {
        let t = Transform::new(Vec2::new(1.0, 2.0), Rot2::new(PI_OVER_TWO));
        let p = Vec2::new(3.0, 4.0);
        
        let transformed = t.transform_point(p);
        let back = t.inverse_transform_point(transformed);
        
        assert!(approx_eq(back.x, p.x, 1e-5));
        assert!(approx_eq(back.y, p.y, 1e-5));
    }

    #[test]
    fn test_assign_operators() {
        let mut t1 = Transform::from_translation(Vec2::new(1.0, 0.0));
        let t2 = Transform::from_translation(Vec2::new(0.0, 2.0));
        
        t1 *= t2;
        let p = Vec2::new(0.0, 0.0);
        let transformed = t1.transform_point(p);
        assert_eq!(transformed, Vec2::new(1.0, 2.0));
    }
}
