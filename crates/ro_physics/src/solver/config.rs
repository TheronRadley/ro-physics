//! Solver configuration.
//!
//! This module provides configuration options for the constraint solver.

/// Configuration for the constraint solver.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct SolverConfig {
    /// Number of velocity iterations
    pub velocity_iterations: usize,
    /// Number of position iterations
    pub position_iterations: usize,
    /// Baumgarte factor for position correction
    pub baumgarte_factor: f32,
    /// Maximum position correction per step
    pub max_position_correction: f32,
    /// Restitution threshold (below this, use 0 restitution)
    pub restitution_threshold: f32,
    /// Whether to use warm starting
    pub warm_starting: bool,
    /// Whether to use friction
    pub enable_friction: bool,
    /// Whether to use restitution
    pub enable_restitution: bool,
}

impl SolverConfig {
    /// Create a new solver configuration
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 3,
            baumgarte_factor: 0.2,
            max_position_correction: 0.1,
            restitution_threshold: 0.5,
            warm_starting: true,
            enable_friction: true,
            enable_restitution: true,
        }
    }

    /// Set the number of velocity iterations
    #[inline]
    #[must_use]
    pub const fn with_velocity_iterations(mut self, iterations: usize) -> Self {
        self.velocity_iterations = iterations;
        self
    }

    /// Set the number of position iterations
    #[inline]
    #[must_use]
    pub const fn with_position_iterations(mut self, iterations: usize) -> Self {
        self.position_iterations = iterations;
        self
    }

    /// Set the Baumgarte factor
    #[inline]
    #[must_use]
    pub const fn with_baumgarte_factor(mut self, factor: f32) -> Self {
        self.baumgarte_factor = factor;
        self
    }

    /// Set the maximum position correction
    #[inline]
    #[must_use]
    pub const fn with_max_position_correction(mut self, correction: f32) -> Self {
        self.max_position_correction = correction;
        self
    }

    /// Set the restitution threshold
    #[inline]
    #[must_use]
    pub const fn with_restitution_threshold(mut self, threshold: f32) -> Self {
        self.restitution_threshold = threshold;
        self
    }

    /// Enable or disable warm starting
    #[inline]
    #[must_use]
    pub const fn with_warm_starting(mut self, enable: bool) -> Self {
        self.warm_starting = enable;
        self
    }

    /// Enable or disable friction
    #[inline]
    #[must_use]
    pub const fn with_friction(mut self, enable: bool) -> Self {
        self.enable_friction = enable;
        self
    }

    /// Enable or disable restitution
    #[inline]
    #[must_use]
    pub const fn with_restitution(mut self, enable: bool) -> Self {
        self.enable_restitution = enable;
        self
    }
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let config = SolverConfig::new();
        assert_eq!(config.velocity_iterations, 8);
        assert_eq!(config.position_iterations, 3);
        assert!(approx_eq(config.baumgarte_factor, 0.2));
        assert!(approx_eq(config.max_position_correction, 0.1));
        assert!(approx_eq(config.restitution_threshold, 0.5));
        assert!(config.warm_starting);
        assert!(config.enable_friction);
        assert!(config.enable_restitution);
    }

    #[test]
    fn test_builder() {
        let config = SolverConfig::new()
            .with_velocity_iterations(10)
            .with_position_iterations(5)
            .with_baumgarte_factor(0.4)
            .with_max_position_correction(0.2)
            .with_restitution_threshold(0.3)
            .with_warm_starting(false)
            .with_friction(false)
            .with_restitution(false);

        assert_eq!(config.velocity_iterations, 10);
        assert_eq!(config.position_iterations, 5);
        assert!(approx_eq(config.baumgarte_factor, 0.4));
        assert!(approx_eq(config.max_position_correction, 0.2));
        assert!(approx_eq(config.restitution_threshold, 0.3));
        assert!(!config.warm_starting);
        assert!(!config.enable_friction);
        assert!(!config.enable_restitution);
    }

    #[test]
    fn test_default() {
        let config = SolverConfig::default();
        assert_eq!(config.velocity_iterations, 8);
    }

    // Helper function for tests
    #[allow(dead_code)]
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= crate::constants::EPSILON
    }

    fn approx_eq_eps(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() <= epsilon
    }
}
