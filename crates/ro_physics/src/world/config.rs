//! World configuration.
//!
//! This module provides configuration options for the physics world.

use crate::ccd::CcdConfig;
use crate::math::Vec2;
use crate::sleeping::SleepingConfig;
use crate::solver::SolverConfig;

/// Configuration for the physics world.
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct WorldConfig {
    /// Gravity vector
    pub gravity: Vec2,
    /// Solver configuration
    pub solver_config: SolverConfig,
    /// Sleeping configuration
    pub sleeping_config: SleepingConfig,
    /// Maximum number of bodies
    pub max_bodies: usize,
    /// Maximum number of colliders
    pub max_colliders: usize,
    /// Maximum number of contacts
    pub max_contacts: usize,
    /// Maximum number of constraints
    pub max_constraints: usize,
    /// Whether to enable sleeping
    pub enable_sleeping: bool,
    /// Number of constraint solver iterations
    pub constraint_iterations: usize,
    /// CCD configuration
    pub ccd_config: CcdConfig,
}

impl WorldConfig {
    /// Create a new world configuration
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the gravity vector
    #[inline]
    #[must_use]
    pub fn with_gravity(mut self, gravity: Vec2) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set the solver configuration
    #[inline]
    #[must_use]
    pub fn with_solver_config(mut self, config: SolverConfig) -> Self {
        self.solver_config = config;
        self
    }

    /// Set the sleeping configuration
    #[inline]
    #[must_use]
    pub fn with_sleeping_config(mut self, config: SleepingConfig) -> Self {
        self.sleeping_config = config;
        self
    }

    /// Set the maximum number of bodies
    #[inline]
    #[must_use]
    pub fn with_max_bodies(mut self, max: usize) -> Self {
        self.max_bodies = max;
        self
    }

    /// Set the maximum number of colliders
    #[inline]
    #[must_use]
    pub fn with_max_colliders(mut self, max: usize) -> Self {
        self.max_colliders = max;
        self
    }

    /// Set the maximum number of contacts
    #[inline]
    #[must_use]
    pub fn with_max_contacts(mut self, max: usize) -> Self {
        self.max_contacts = max;
        self
    }

    /// Set the maximum number of constraints
    #[inline]
    #[must_use]
    pub fn with_max_constraints(mut self, max: usize) -> Self {
        self.max_constraints = max;
        self
    }

    /// Set the CCD configuration
    #[inline]
    #[must_use]
    pub fn with_ccd_config(mut self, config: CcdConfig) -> Self {
        self.ccd_config = config;
        self
    }

    /// Set the number of constraint solver iterations
    #[inline]
    #[must_use]
    pub fn with_constraint_iterations(mut self, iterations: usize) -> Self {
        self.constraint_iterations = iterations;
        self
    }

    /// Enable or disable sleeping
    #[inline]
    #[must_use]
    pub fn with_sleeping(mut self, enable: bool) -> Self {
        self.enable_sleeping = enable;
        self
    }
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            gravity: Vec2::new(0.0, -9.81),
            solver_config: SolverConfig::default(),
            sleeping_config: SleepingConfig::default(),
            ccd_config: CcdConfig::default(),
            max_bodies: 1024,
            max_colliders: 4096,
            max_contacts: 8192,
            max_constraints: 512,
            enable_sleeping: true,
            constraint_iterations: 8,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let config = WorldConfig::new();
        assert_eq!(config.gravity, Vec2::new(0.0, -9.81));
        assert_eq!(config.max_bodies, 1024);
        assert_eq!(config.max_colliders, 4096);
        assert_eq!(config.max_contacts, 8192);
        assert!(config.ccd_config.enabled);
        assert!(config.enable_sleeping);
    }

    #[test]
    fn test_builder() {
        let config = WorldConfig::new()
            .with_gravity(Vec2::new(0.0, -10.0))
            .with_max_bodies(2048)
            .with_max_colliders(8192)
            .with_max_contacts(16384)
            .with_ccd_config(CcdConfig::new().with_enabled(true))
            .with_sleeping(false);

        assert_eq!(config.gravity, Vec2::new(0.0, -10.0));
        assert_eq!(config.max_bodies, 2048);
        assert_eq!(config.max_colliders, 8192);
        assert_eq!(config.max_contacts, 16384);
        assert!(config.ccd_config.enabled);
        assert!(!config.enable_sleeping);
    }

    #[test]
    fn test_default() {
        let config = WorldConfig::default();
        assert_eq!(config.gravity, Vec2::new(0.0, -9.81));
    }
}
