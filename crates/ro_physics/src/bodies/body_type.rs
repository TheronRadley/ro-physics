//! Body type definitions.
//!
//! This module provides the BodyType enum for classifying bodies.

use std::fmt;

/// Type of rigid body.
///
/// Different body types have different behaviors in the physics simulation:
///
/// - **Static**: Fixed in place, infinite mass, does not move
/// - **Dynamic**: Affected by forces and collisions, normal physics behavior
/// - **Kinematic**: Moves according to velocity but not affected by forces
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum BodyType {
    /// Static body - fixed in place, infinite mass
    Static,
    /// Dynamic body - affected by forces and collisions
    #[default]
    Dynamic,
    /// Kinematic body - moves according to velocity but not affected by forces
    Kinematic,
}

impl BodyType {
    /// Check if the body type is static
    #[inline]
    #[must_use]
    pub fn is_static(self) -> bool {
        matches!(self, BodyType::Static)
    }

    /// Check if the body type is dynamic
    #[inline]
    #[must_use]
    pub fn is_dynamic(self) -> bool {
        matches!(self, BodyType::Dynamic)
    }

    /// Check if the body type is kinematic
    #[inline]
    #[must_use]
    pub fn is_kinematic(self) -> bool {
        matches!(self, BodyType::Kinematic)
    }
}

impl fmt::Display for BodyType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BodyType::Static => write!(f, "Static"),
            BodyType::Dynamic => write!(f, "Dynamic"),
            BodyType::Kinematic => write!(f, "Kinematic"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_body_type() {
        assert!(BodyType::Static.is_static());
        assert!(!BodyType::Static.is_dynamic());
        assert!(!BodyType::Static.is_kinematic());

        assert!(!BodyType::Dynamic.is_static());
        assert!(BodyType::Dynamic.is_dynamic());
        assert!(!BodyType::Dynamic.is_kinematic());

        assert!(!BodyType::Kinematic.is_static());
        assert!(!BodyType::Kinematic.is_dynamic());
        assert!(BodyType::Kinematic.is_kinematic());
    }

    #[test]
    fn test_default() {
        assert_eq!(BodyType::default(), BodyType::Dynamic);
    }

    #[test]
    fn test_display() {
        assert_eq!(format!("{}", BodyType::Static), "Static");
        assert_eq!(format!("{}", BodyType::Dynamic), "Dynamic");
        assert_eq!(format!("{}", BodyType::Kinematic), "Kinematic");
    }
}
