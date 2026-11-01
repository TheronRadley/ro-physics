//! Broad-phase configuration.
//!
//! This module provides configuration options for the broad-phase.

/// Configuration for the broad-phase collision detection.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct BroadPhaseConfig {
    /// Initial capacity of the AABB tree
    pub aabb_tree_capacity: usize,
    /// Whether to use incremental updates
    pub incremental: bool,
    /// Margin to add to AABBs for fattening
    pub aabb_margin: f32,
}

impl BroadPhaseConfig {
    /// Create a new broad-phase configuration
    #[inline]
    #[must_use]
    pub const fn new(aabb_tree_capacity: usize) -> Self {
        Self {
            aabb_tree_capacity,
            incremental: true,
            aabb_margin: 0.1,
        }
    }

    /// Set the AABB tree capacity
    #[inline]
    #[must_use]
    pub const fn with_aabb_tree_capacity(mut self, capacity: usize) -> Self {
        self.aabb_tree_capacity = capacity;
        self
    }

    /// Set whether to use incremental updates
    #[inline]
    #[must_use]
    pub const fn with_incremental(mut self, incremental: bool) -> Self {
        self.incremental = incremental;
        self
    }

    /// Set the AABB margin
    #[inline]
    #[must_use]
    pub const fn with_aabb_margin(mut self, margin: f32) -> Self {
        self.aabb_margin = margin;
        self
    }
}

impl Default for BroadPhaseConfig {
    fn default() -> Self {
        Self {
            aabb_tree_capacity: 1024,
            incremental: true,
            aabb_margin: 0.1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let config = BroadPhaseConfig::new(100);
        assert_eq!(config.aabb_tree_capacity, 100);
        assert!(config.incremental);
        assert!(approx_eq(config.aabb_margin, 0.1));
    }

    #[test]
    fn test_builder() {
        let config = BroadPhaseConfig::new(100)
            .with_aabb_tree_capacity(200)
            .with_incremental(false)
            .with_aabb_margin(0.2);

        assert_eq!(config.aabb_tree_capacity, 200);
        assert!(!config.incremental);
        assert!(approx_eq(config.aabb_margin, 0.2));
    }

    #[test]
    fn test_default() {
        let config = BroadPhaseConfig::default();
        assert_eq!(config.aabb_tree_capacity, 1024);
        assert!(config.incremental);
        assert!(approx_eq(config.aabb_margin, 0.1));
    }

    // Helper function for tests
    #[allow(dead_code)]
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= 1e-6
    }

}
