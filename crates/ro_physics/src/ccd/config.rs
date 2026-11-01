//! CCD configuration.
//!
//! This module provides configuration options for continuous collision detection.

/// Configuration for continuous collision detection.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct CcdConfig {
    /// Whether CCD is enabled
    pub enabled: bool,
    /// Maximum number of TOI iterations
    pub max_toi_iterations: usize,
    /// TOI tolerance (convergence threshold)
    pub toi_tolerance: f32,
    /// Maximum TOI (prevents very small time steps)
    pub max_toi: f32,
    /// Whether to use conservative advancement
    pub conservative_advancement: bool,
}

impl CcdConfig {
    /// Create a new CCD configuration
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enabled: true,
            max_toi_iterations: 20,
            toi_tolerance: 1e-4,
            max_toi: 1.0,
            conservative_advancement: true,
        }
    }

    /// Enable or disable CCD
    #[inline]
    #[must_use]
    pub const fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set the maximum number of TOI iterations
    #[inline]
    #[must_use]
    pub const fn with_max_toi_iterations(mut self, iterations: usize) -> Self {
        self.max_toi_iterations = iterations;
        self
    }

    /// Set the TOI tolerance
    #[inline]
    #[must_use]
    pub const fn with_toi_tolerance(mut self, tolerance: f32) -> Self {
        self.toi_tolerance = tolerance;
        self
    }

    /// Set the maximum TOI
    #[inline]
    #[must_use]
    pub const fn with_max_toi(mut self, max_toi: f32) -> Self {
        self.max_toi = max_toi;
        self
    }

    /// Enable or disable conservative advancement
    #[inline]
    #[must_use]
    pub const fn with_conservative_advancement(mut self, enabled: bool) -> Self {
        self.conservative_advancement = enabled;
        self
    }
}

impl Default for CcdConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let config = CcdConfig::new();
        assert!(config.enabled);
        assert_eq!(config.max_toi_iterations, 20);
        assert!(approx_eq(config.toi_tolerance, 1e-4));
        assert!(approx_eq(config.max_toi, 1.0));
        assert!(config.conservative_advancement);
    }

    #[test]
    fn test_builder() {
        let config = CcdConfig::new()
            .with_enabled(false)
            .with_max_toi_iterations(10)
            .with_toi_tolerance(1e-5)
            .with_max_toi(0.5)
            .with_conservative_advancement(false);

        assert!(!config.enabled);
        assert_eq!(config.max_toi_iterations, 10);
        assert!(approx_eq(config.toi_tolerance, 1e-5));
        assert!(approx_eq(config.max_toi, 0.5));
        assert!(!config.conservative_advancement);
    }

    #[test]
    fn test_default() {
        let config = CcdConfig::default();
        assert!(config.enabled);
    }

    // Helper function for tests
    #[allow(dead_code)]
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= crate::constants::EPSILON
    }

}
