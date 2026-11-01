//! Continuous Collision Detection (CCD) module for the physics engine.
//!
//! This module provides swept collision detection and time of impact (TOI) calculation
//! to prevent tunneling in fast-moving objects.

mod config;
mod swept_collision;
#[cfg(test)]
mod tests;
mod toi;

pub use config::CcdConfig;
pub use swept_collision::{CcdResult, SweptCollisionDetector};
pub use toi::{TimeOfImpact, ToiResult};

/// CCD utilities
pub mod utils {
    use crate::math::Vec2;

    /// Linear motion: p(t) = p0 + v * t
    #[inline]
    #[must_use]
    pub fn linear_motion(position: Vec2, velocity: Vec2, t: f32) -> Vec2 {
        position + velocity * t
    }

    /// Angular motion: angle(t) = angle0 + w * t
    #[inline]
    #[must_use]
    pub fn angular_motion(angle: f32, angular_velocity: f32, t: f32) -> f32 {
        angle + angular_velocity * t
    }

    /// Check if a time of impact is valid (within [0, 1] range)
    #[inline]
    #[must_use]
    pub fn is_valid_toi(toi: f32) -> bool {
        (0.0..=1.0).contains(&toi)
    }

    /// Clamp time of impact to valid range
    #[inline]
    #[must_use]
    pub fn clamp_toi(toi: f32) -> f32 {
        toi.clamp(0.0, 1.0)
    }
}

pub use utils::*;
