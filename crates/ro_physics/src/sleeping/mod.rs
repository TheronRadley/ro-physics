//! Sleeping system for the physics engine.
//!
//! This module provides automatic sleeping of bodies to improve performance.

mod config;

pub use config::SleepingConfig;

use crate::bodies::BodyHandle;
use crate::math::Vec2;

/// Sleeping system for the physics world.
///
/// The sleeping system automatically deactivates bodies that are at rest
/// to improve performance. Bodies are woken up when they are affected
/// by external forces or collisions.
#[derive(Debug)]
pub struct SleepingSystem {
    /// Configuration
    config: SleepingConfig,
    /// Sleeping state for each body
    sleeping: std::collections::HashMap<BodyHandle, SleepState>,
}

impl SleepingSystem {
    /// Create a new sleeping system
    #[inline]
    #[must_use]
    pub fn new(config: SleepingConfig) -> Self {
        Self {
            config,
            sleeping: std::collections::HashMap::new(),
        }
    }

    /// Get the configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &SleepingConfig {
        &self.config
    }

    /// Check if a body is sleeping
    #[inline]
    #[must_use]
    pub fn is_sleeping(&self, handle: BodyHandle) -> bool {
        self.sleeping
            .get(&handle)
            .map(|state| state.sleeping)
            .unwrap_or(false)
    }

    /// Set if a body is sleeping
    #[inline]
    pub fn set_sleeping(&mut self, handle: BodyHandle, sleeping: bool) {
        let state = self.sleeping.entry(handle).or_insert_with(|| SleepState::new());
        state.sleeping = sleeping;
        if !sleeping {
            state.sleep_timer = 0.0;
        }
    }

    /// Wake up a body
    #[inline]
    pub fn wake_up(&mut self, handle: BodyHandle) {
        self.set_sleeping(handle, false);
    }

    /// Update the sleeping state for a body
    #[inline]
    pub fn update_body(&mut self, handle: BodyHandle, linear_velocity: Vec2, angular_velocity: f32) {
        let state = self.sleeping.entry(handle).or_insert_with(|| SleepState::new());

        // Check if the body is moving
        let speed_squared = linear_velocity.length_squared();
        let angular_speed = angular_velocity.abs();

        if speed_squared > self.config.linear_threshold * self.config.linear_threshold
            || angular_speed > self.config.angular_threshold
        {
            // Body is moving, wake it up
            state.sleeping = false;
            state.sleep_timer = 0.0;
        } else if !state.sleeping {
            // Body is not moving, increment sleep timer
            state.sleep_timer += 1.0 / 60.0; // Assuming 60 FPS
            
            // Check if we should sleep
            if state.sleep_timer >= self.config.sleep_time {
                state.sleeping = true;
            }
        }
    }

    /// Wake up all bodies
    #[inline]
    pub fn wake_up_all(&mut self) {
        for state in self.sleeping.values_mut() {
            state.sleeping = false;
            state.sleep_timer = 0.0;
        }
    }

    /// Clear all sleeping state
    #[inline]
    pub fn clear(&mut self) {
        self.sleeping.clear();
    }
}

impl Default for SleepingSystem {
    fn default() -> Self {
        Self::new(SleepingConfig::default())
    }
}

/// Sleeping state for a single body
#[derive(Debug, Clone)]
pub struct SleepState {
    /// Whether the body is sleeping
    pub sleeping: bool,
    /// Sleep timer (seconds)
    pub sleep_timer: f32,
}

impl SleepState {
    /// Create a new sleep state
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            sleeping: false,
            sleep_timer: 0.0,
        }
    }
}

impl Default for SleepState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_construction() {
        let system = SleepingSystem::new(SleepingConfig::default());
        assert_eq!(system.sleeping.len(), 0);

        let system2 = SleepingSystem::default();
        assert_eq!(system2.sleeping.len(), 0);
    }

    #[test]
    fn test_sleeping() {
        let mut system = SleepingSystem::default();
        let handle = BodyHandle::new(1, 0);

        assert!(!system.is_sleeping(handle));

        system.set_sleeping(handle, true);
        assert!(system.is_sleeping(handle));

        system.wake_up(handle);
        assert!(!system.is_sleeping(handle));
    }

    #[test]
    fn test_update_body() {
        let mut system = SleepingSystem::new(
            SleepingConfig::new()
                .with_linear_threshold(0.1)
                .with_angular_threshold(0.1)
                .with_sleep_time(1.0),
        );

        let handle = BodyHandle::new(1, 0);

        // Body is moving
        system.update_body(handle, Vec2::new(0.2, 0.0), 0.0);
        assert!(!system.is_sleeping(handle));

        // Body is not moving, but sleep timer not expired
        for _ in 0..59 {
            system.update_body(handle, Vec2::zero(), 0.0);
        }
        assert!(!system.is_sleeping(handle));

        // Body is not moving, sleep timer expired
        system.update_body(handle, Vec2::zero(), 0.0);
        assert!(system.is_sleeping(handle));

        // Body starts moving again
        system.update_body(handle, Vec2::new(0.2, 0.0), 0.0);
        assert!(!system.is_sleeping(handle));
    }

    #[test]
    fn test_wake_up_all() {
        let mut system = SleepingSystem::default();

        let handle1 = BodyHandle::new(1, 0);
        let handle2 = BodyHandle::new(2, 0);

        system.set_sleeping(handle1, true);
        system.set_sleeping(handle2, true);

        assert!(system.is_sleeping(handle1));
        assert!(system.is_sleeping(handle2));

        system.wake_up_all();

        assert!(!system.is_sleeping(handle1));
        assert!(!system.is_sleeping(handle2));
    }

    #[test]
    fn test_clear() {
        let mut system = SleepingSystem::default();

        let handle = BodyHandle::new(1, 0);
        system.set_sleeping(handle, true);

        assert_eq!(system.sleeping.len(), 1);

        system.clear();
        assert_eq!(system.sleeping.len(), 0);
    }

    #[test]
    fn test_sleep_state() {
        let state = SleepState::new();
        assert!(!state.sleeping);
        assert!(approx_eq(state.sleep_timer, 0.0));
    }
}
