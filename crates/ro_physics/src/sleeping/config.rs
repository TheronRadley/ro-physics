//! Sleeping configuration.
//!
//! This module provides configuration options for the sleeping system.

/// Configuration for the sleeping system.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct SleepingConfig {
    /// Linear velocity threshold for sleeping
    pub linear_threshold: f32,
    /// Angular velocity threshold for sleeping
    pub angular_threshold: f32,
    /// Time (in seconds) a body must be below thresholds before sleeping
    pub sleep_time: f32,
    /// Whether to enable sleeping
    pub enabled: bool,
}

impl SleepingConfig {
    /// Create a new sleeping configuration
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            linear_threshold: 0.01,
            angular_threshold: 0.01,
            sleep_time: 0.5,
            enabled: true,
        }
    }

    /// Set the linear velocity threshold
    #[inline]
    #[must_use]
    pub const fn with_linear_threshold(mut self, threshold: f32) -> Self {
        self.linear_threshold = threshold;
        self
    }

    /// Set the angular velocity threshold
    #[inline]
    #[must_use]
    pub const fn with_angular_threshold(mut self, threshold: f32) -> Self {
        self.angular_threshold = threshold;
        self
    }

    /// Set the sleep time
    #[inline]
    #[must_use]
    pub const fn with_sleep_time(mut self, time: f32) -> Self {
        self.sleep_time = time;
        self
    }

    /// Enable or disable sleeping
    #[inline]
    #[must_use]
    pub const fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }
}

impl Default for SleepingConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let config = SleepingConfig::new();
        assert!(approx_eq(config.linear_threshold, 0.01));
        assert!(approx_eq(config.angular_threshold, 0.01));
        assert!(approx_eq(config.sleep_time, 0.5));
        assert!(config.enabled);
    }

    #[test]
    fn test_builder() {
        let config = SleepingConfig::new()
            .with_linear_threshold(0.1)
            .with_angular_threshold(0.1)
            .with_sleep_time(1.0)
            .with_enabled(false);

        assert!(approx_eq(config.linear_threshold, 0.1));
        assert!(approx_eq(config.angular_threshold, 0.1));
        assert!(approx_eq(config.sleep_time, 1.0));
        assert!(!config.enabled);
    }

    #[test]
    fn test_default() {
        let config = SleepingConfig::default();
        assert!(approx_eq(config.linear_threshold, 0.01));
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
