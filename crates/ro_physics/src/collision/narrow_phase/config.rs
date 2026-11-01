//! Narrow-phase configuration.
//!
//! This module provides configuration options for narrow-phase collision detection.

/// Configuration for narrow-phase collision detection.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct NarrowPhaseConfig {
    /// Maximum number of contact points to generate per manifold
    pub max_contact_points: usize,
    /// Tolerance for contact point merging
    pub contact_tolerance: f32,
    /// Margin for collision detection
    pub collision_margin: f32,
    /// Whether to use precise collision detection
    pub precise: bool,
}

impl NarrowPhaseConfig {
    /// Create a new narrow-phase configuration
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            max_contact_points: 4,
            contact_tolerance: 0.01,
            collision_margin: 0.05,
            precise: true,
        }
    }

    /// Set the maximum number of contact points
    #[inline]
    #[must_use]
    pub const fn with_max_contact_points(mut self, max: usize) -> Self {
        self.max_contact_points = max;
        self
    }

    /// Set the contact tolerance
    #[inline]
    #[must_use]
    pub const fn with_contact_tolerance(mut self, tolerance: f32) -> Self {
        self.contact_tolerance = tolerance;
        self
    }

    /// Set the collision margin
    #[inline]
    #[must_use]
    pub const fn with_collision_margin(mut self, margin: f32) -> Self {
        self.collision_margin = margin;
        self
    }

    /// Set whether to use precise collision detection
    #[inline]
    #[must_use]
    pub const fn with_precise(mut self, precise: bool) -> Self {
        self.precise = precise;
        self
    }
}

impl Default for NarrowPhaseConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let config = NarrowPhaseConfig::new();
        assert_eq!(config.max_contact_points, 4);
        assert!(approx_eq(config.contact_tolerance, 0.01));
        assert!(approx_eq(config.collision_margin, 0.05));
        assert!(config.precise);
    }

    #[test]
    fn test_builder() {
        let config = NarrowPhaseConfig::new()
            .with_max_contact_points(8)
            .with_contact_tolerance(0.05)
            .with_collision_margin(0.1)
            .with_precise(false);

        assert_eq!(config.max_contact_points, 8);
        assert!(approx_eq(config.contact_tolerance, 0.05));
        assert!(approx_eq(config.collision_margin, 0.1));
        assert!(!config.precise);
    }

    #[test]
    fn test_default() {
        let config = NarrowPhaseConfig::default();
        assert_eq!(config.max_contact_points, 4);
    }

    // Helper function for tests
    #[allow(dead_code)]
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= crate::constants::EPSILON
    }

}
