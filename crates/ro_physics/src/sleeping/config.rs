//! Sleeping configuration.
//!
//! This module provides configuration options for the sleeping system.

/// Configuration for the sleeping system.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
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

impl SleepingConfig {
    /// Replace non-finite and negative values with the defaults
    ///
    /// A NaN threshold would make the comparison in the sleeping test always
    /// false, so bodies would never sleep (or never wake). `World::new` runs
    /// this before storing the config.
    #[inline]
    #[must_use]
    pub fn sanitize(mut self) -> Self {
        let defaults = Self::default();

        if !self.linear_threshold.is_finite() || self.linear_threshold < 0.0 {
            self.linear_threshold = defaults.linear_threshold;
        }

        if !self.angular_threshold.is_finite() || self.angular_threshold < 0.0 {
            self.angular_threshold = defaults.angular_threshold;
        }

        if !self.sleep_time.is_finite() || self.sleep_time < 0.0 {
            self.sleep_time = defaults.sleep_time;
        }

        self
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
}
