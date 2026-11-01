//! Serialization support for the physics engine.
//!
//! This module provides serialization and deserialization for physics types.

use serde::{Serialize, Deserialize};

/// Serialization utilities
pub mod utils {
    use super::*;

    /// Serialize a type to JSON
    #[inline]
    pub fn to_json<T: Serialize>(value: &T) -> Result<String, serde_json::Error> {
        serde_json::to_string(value)
    }

    /// Deserialize a type from JSON
    #[inline]
    pub fn from_json<T: for<'de> Deserialize<'de>>(json: &str) -> Result<T, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// Serialize a type to RON (Rusty Object Notation)
    #[inline]
    pub fn to_ron<T: Serialize>(value: &T) -> Result<String, ron::Error> {
        ron::to_string(value)
    }

    /// Deserialize a type from RON
    #[inline]
    ///
    /// # Errors
    ///
    /// Returns an error when `ron` is not valid RON for `T`.
    pub fn from_ron<T: for<'de> Deserialize<'de>>(ron: &str) -> Result<T, ron::error::SpannedError> {
        ron::from_str(ron)
    }

    /// Serialize a type to binary
    #[inline]
    pub fn to_binary<T: Serialize>(value: &T) -> Result<Vec<u8>, bincode::Error> {
        bincode::serialize(value)
    }

    /// Deserialize a type from binary
    #[inline]
    pub fn from_binary<T: for<'de> Deserialize<'de>>(bytes: &[u8]) -> Result<T, bincode::Error> {
        bincode::deserialize(bytes)
    }
}

pub use utils::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec2;

    #[test]
    fn test_json_serialization() {
        let vec = Vec2::new(1.0, 2.0);
        
        let json = to_json(&vec).unwrap();
        assert!(json.contains("1.0"));
        assert!(json.contains("2.0"));

        let deserialized: Vec2 = from_json(&json).unwrap();
        assert_eq!(deserialized, vec);
    }

    #[test]
    fn test_ron_serialization() {
        let vec = Vec2::new(1.0, 2.0);
        
        let ron = to_ron(&vec).unwrap();
        assert!(ron.contains("1.0"));
        assert!(ron.contains("2.0"));

        let deserialized: Vec2 = from_ron(&ron).unwrap();
        assert_eq!(deserialized, vec);
    }

    #[test]
    fn test_binary_serialization() {
        let vec = Vec2::new(1.0, 2.0);
        
        let bytes = to_binary(&vec).unwrap();
        assert!(!bytes.is_empty());

        let deserialized: Vec2 = from_binary(&bytes).unwrap();
        assert_eq!(deserialized, vec);
    }
}
