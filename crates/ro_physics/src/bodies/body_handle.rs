//! Body handle implementation.
//!
//! This module provides a handle type for referencing bodies.

use std::fmt;
use std::hash::{Hash, Hasher};

/// A handle to a body in the physics world.
///
/// Body handles are used to reference bodies after they are created.
/// They are lightweight and can be copied freely.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct BodyHandle {
    /// Unique identifier for the body
    pub id: u64,
    /// Generation counter to detect stale handles
    pub generation: u32,
}

impl BodyHandle {
    /// Create a new body handle
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

    /// Increment the generation (used when a body is removed and its slot reused)
    #[inline]
    #[must_use]
    pub fn next_generation(self) -> Self {
        Self {
            generation: self.generation.wrapping_add(1),
            ..self
        }
    }
}

impl Default for BodyHandle {
    fn default() -> Self {
        Self::null()
    }
}

impl fmt::Display for BodyHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_null() {
            write!(f, "BodyHandle(null)")
        } else {
            write!(f, "BodyHandle(id: {}, gen: {})", self.id, self.generation)
        }
    }
}

/// A generator for creating unique body handles
#[derive(Debug, Clone)]
pub struct BodyHandleGenerator {
    next_id: u64,
}

impl BodyHandleGenerator {
    /// Create a new handle generator
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self { next_id: 1 }
    }

    /// Generate a new body handle
    #[inline]
    #[must_use]
    pub fn generate(&mut self) -> BodyHandle {
        let id = self.next_id;
        self.next_id += 1;
        BodyHandle::new(id, 0)
    }

    /// Reset the generator
    #[inline]
    pub fn reset(&mut self) {
        self.next_id = 1;
    }
}

impl Default for BodyHandleGenerator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_body_handle() {
        let handle1 = BodyHandle::new(1, 0);
        let handle2 = BodyHandle::new(2, 0);
        let null = BodyHandle::null();

        assert!(!handle1.is_null());
        assert!(!handle2.is_null());
        assert!(null.is_null());

        assert_ne!(handle1, handle2);
        assert_ne!(handle1, null);
    }

    #[test]
    fn test_next_generation() {
        let handle = BodyHandle::new(1, 0);
        let next = handle.next_generation();

        assert_eq!(handle.id, next.id);
        assert_eq!(next.generation, 1);
    }

    #[test]
    fn test_default() {
        let handle = BodyHandle::default();
        assert!(handle.is_null());
    }

    #[test]
    fn test_display() {
        let handle = BodyHandle::new(1, 0);
        let null = BodyHandle::null();

        assert_eq!(format!("{}", handle), "BodyHandle(id: 1, gen: 0)");
        assert_eq!(format!("{}", null), "BodyHandle(null)");
    }

    #[test]
    fn test_handle_generator() {
        let mut generator = BodyHandleGenerator::new();

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
    fn test_handle_generator_reset() {
        let mut generator = BodyHandleGenerator::new();

        let h1 = generator.generate();
        generator.reset();
        let h2 = generator.generate();

        assert_eq!(h1.id, 1);
        assert_eq!(h2.id, 1);
    }

    #[test]
    fn test_hash() {
        let handle1 = BodyHandle::new(1, 0);
        let handle2 = BodyHandle::new(1, 0);
        let handle3 = BodyHandle::new(2, 0);

        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        handle1.hash(&mut hasher);
        let hash1 = hasher.finish();

        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        handle2.hash(&mut hasher);
        let hash2 = hasher.finish();

        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        handle3.hash(&mut hasher);
        let hash3 = hasher.finish();

        assert_eq!(hash1, hash2);
        assert_ne!(hash1, hash3);
    }
}
