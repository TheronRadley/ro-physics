//! Mass properties for physics shapes.
//!
//! This module provides mass, inertia, and center of mass calculations
//! for various shape types.

use crate::math::utils::approx_eq;
use crate::math::Vec2;

/// Mass properties for a shape.
///
/// Contains the mass, inertia, and center of mass information
/// needed for rigid-body physics.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct MassProperties {
    /// Mass of the shape
    pub mass: f32,
    /// Inverse mass (0 for static bodies)
    pub inv_mass: f32,
    /// Moment of inertia about the center of mass
    pub inertia: f32,
    /// Inverse inertia (0 for static bodies)
    pub inv_inertia: f32,
    /// Center of mass position
    pub center_of_mass: Vec2,
}

impl MassProperties {
    /// Create new mass properties
    #[inline]
    #[must_use]
    pub fn new(mass: f32, inertia: f32, center_of_mass: Vec2) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        let inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };

        Self {
            mass,
            inv_mass,
            inertia,
            inv_inertia,
            center_of_mass,
        }
    }

    /// Create mass properties for a static body (infinite mass)
    #[inline]
    #[must_use]
    pub fn static_body() -> Self {
        Self {
            mass: 0.0,
            inv_mass: 0.0,
            inertia: 0.0,
            inv_inertia: 0.0,
            center_of_mass: Vec2::zero(),
        }
    }

    /// Create mass properties for a dynamic body with given mass and inertia
    #[inline]
    #[must_use]
    pub fn dynamic(mass: f32, inertia: f32) -> Self {
        Self::new(mass, inertia, Vec2::zero())
    }

    /// Check if this represents a static body
    #[inline]
    #[must_use]
    pub fn is_static(self) -> bool {
        self.inv_mass == 0.0
    }

    /// Scale the mass properties by a factor
    #[inline]
    #[must_use]
    pub fn scaled(self, scale: f32) -> Self {
        Self {
            mass: self.mass * scale,
            inv_mass: if self.inv_mass > 0.0 {
                self.inv_mass / scale
            } else {
                0.0
            },
            inertia: self.inertia * scale,
            inv_inertia: if self.inv_inertia > 0.0 {
                self.inv_inertia / scale
            } else {
                0.0
            },
            center_of_mass: self.center_of_mass,
        }
    }

    /// Translate the center of mass
    #[inline]
    #[must_use]
    pub fn translated(self, translation: Vec2) -> Self {
        Self {
            center_of_mass: self.center_of_mass + translation,
            ..self
        }
    }

    /// Calculate mass properties for a transformed shape
    ///
    /// Uses the parallel axis theorem to adjust inertia for the translation.
    #[inline]
    #[must_use]
    pub fn transformed(self, translation: Vec2) -> Self {
        // Parallel axis theorem: I' = I + m * d^2
        let d_squared = translation.length_squared();
        let new_inertia = self.inertia + self.mass * d_squared;

        Self {
            mass: self.mass,
            inv_mass: self.inv_mass,
            inertia: new_inertia,
            inv_inertia: if new_inertia > 0.0 {
                1.0 / new_inertia
            } else {
                0.0
            },
            center_of_mass: self.center_of_mass + translation,
        }
    }
}

impl PartialEq for MassProperties {
    /// Compare with the usual floating point tolerance
    ///
    /// Masses and inertias are computed through several floating point steps,
    /// so an exact comparison would report equal shapes as different.
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        approx_eq(self.mass, other.mass)
            && approx_eq(self.inertia, other.inertia)
            && approx_eq(self.inv_mass, other.inv_mass)
            && approx_eq(self.inv_inertia, other.inv_inertia)
            && approx_eq(self.center_of_mass.x, other.center_of_mass.x)
            && approx_eq(self.center_of_mass.y, other.center_of_mass.y)
    }
}

impl Default for MassProperties {
    fn default() -> Self {
        Self::static_body()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::EPSILON;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_construction() {
        let mp1 = MassProperties::new(10.0, 5.0, Vec2::new(1.0, 2.0));
        assert_eq!(mp1.mass, 10.0);
        assert!(approx_eq(mp1.inv_mass, 0.1));
        assert_eq!(mp1.inertia, 5.0);
        assert!(approx_eq(mp1.inv_inertia, 0.2));
        assert_eq!(mp1.center_of_mass, Vec2::new(1.0, 2.0));

        let mp2 = MassProperties::static_body();
        assert_eq!(mp2.mass, 0.0);
        assert_eq!(mp2.inv_mass, 0.0);
        assert_eq!(mp2.inertia, 0.0);
        assert_eq!(mp2.inv_inertia, 0.0);

        let mp3 = MassProperties::dynamic(10.0, 5.0);
        assert_eq!(mp3.mass, 10.0);
        assert_eq!(mp3.center_of_mass, Vec2::zero());
    }

    #[test]
    fn test_is_static() {
        let mp1 = MassProperties::static_body();
        assert!(mp1.is_static());

        let mp2 = MassProperties::dynamic(10.0, 5.0);
        assert!(!mp2.is_static());
    }

    #[test]
    fn test_scaled() {
        let mp = MassProperties::dynamic(10.0, 5.0);
        let scaled = mp.scaled(2.0);

        assert_eq!(scaled.mass, 20.0);
        assert!(approx_eq(scaled.inv_mass, 0.05));
        assert_eq!(scaled.inertia, 10.0);
        assert!(approx_eq(scaled.inv_inertia, 0.1));
    }

    #[test]
    fn test_translated() {
        let mp = MassProperties::dynamic(10.0, 5.0);
        let translated = mp.translated(Vec2::new(1.0, 2.0));

        assert_eq!(translated.center_of_mass, Vec2::new(1.0, 2.0));
        assert_eq!(translated.mass, 10.0);
    }

    #[test]
    fn test_transformed() {
        let mp = MassProperties::dynamic(10.0, 5.0);
        let transformed = mp.transformed(Vec2::new(1.0, 2.0));

        // Parallel axis theorem: I' = I + m * d^2 = 5 + 10 * (1 + 4) = 55
        assert_eq!(transformed.inertia, 55.0);
        assert!(approx_eq(transformed.inv_inertia, 1.0 / 55.0));
        assert_eq!(transformed.center_of_mass, Vec2::new(1.0, 2.0));
    }

    #[test]
    fn test_approx_eq() {
        let mp1 = MassProperties::dynamic(10.0, 5.0);
        let mp2 = MassProperties::new(10.0, 5.0 + EPSILON, Vec2::zero());

        // This tests the PartialEq implementation
        assert_eq!(mp1, mp2);
    }
}
