//! Debug utilities for the physics engine.
//!
//! This module provides debugging and visualization tools for the physics simulation.

use crate::geometry::Aabb;
use crate::math::{Transform, Vec2};

/// Debug rendering flags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DebugFlags {
    /// Show collider outlines
    pub show_colliders: bool,
    /// Show AABBs
    pub show_aabbs: bool,
    /// Show contact points
    pub show_contacts: bool,
    /// Show contact normals
    pub show_normals: bool,
    /// Show velocity vectors
    pub show_velocities: bool,
    /// Show centers of mass
    pub show_centers_of_mass: bool,
    /// Show sleeping state
    pub show_sleeping: bool,
    /// Show broad-phase structure
    pub show_broad_phase: bool,
    /// Show statistics
    pub show_stats: bool,
}

impl DebugFlags {
    /// Create new debug flags with all options disabled
    #[inline]
    #[must_use]
    pub const fn none() -> Self {
        Self {
            show_colliders: false,
            show_aabbs: false,
            show_contacts: false,
            show_normals: false,
            show_velocities: false,
            show_centers_of_mass: false,
            show_sleeping: false,
            show_broad_phase: false,
            show_stats: false,
        }
    }

    /// Create new debug flags with all options enabled
    #[inline]
    #[must_use]
    pub const fn all() -> Self {
        Self {
            show_colliders: true,
            show_aabbs: true,
            show_contacts: true,
            show_normals: true,
            show_velocities: true,
            show_centers_of_mass: true,
            show_sleeping: true,
            show_broad_phase: true,
            show_stats: true,
        }
    }

    /// Enable all debug options
    #[inline]
    pub fn enable_all(&mut self) {
        *self = Self::all();
    }

    /// Disable all debug options
    #[inline]
    pub fn disable_all(&mut self) {
        *self = Self::none();
    }
}

impl Default for DebugFlags {
    fn default() -> Self {
        Self {
            show_colliders: true,
            show_aabbs: false,
            show_contacts: true,
            show_normals: false,
            show_velocities: false,
            show_centers_of_mass: false,
            show_sleeping: false,
            show_broad_phase: false,
            show_stats: true,
        }
    }
}

/// Debug statistics for the physics simulation
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct DebugStats {
    /// Number of bodies
    pub body_count: usize,
    /// Number of active (non-sleeping) bodies
    pub active_body_count: usize,
    /// Number of colliders
    pub collider_count: usize,
    /// Number of contacts
    pub contact_count: usize,
    /// Number of broad-phase candidates
    pub broad_phase_candidates: usize,
    /// Number of narrow-phase checks
    pub narrow_phase_checks: usize,
    /// Number of solver iterations
    pub solver_iterations: usize,
    /// Simulation time
    pub simulation_time: f32,
    /// Frame time (last frame)
    pub frame_time: f32,
    /// FPS
    pub fps: f32,
}

impl DebugStats {
    /// Create new debug stats
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            body_count: 0,
            active_body_count: 0,
            collider_count: 0,
            contact_count: 0,
            broad_phase_candidates: 0,
            narrow_phase_checks: 0,
            solver_iterations: 0,
            simulation_time: 0.0,
            frame_time: 0.0,
            fps: 0.0,
        }
    }
}

impl Default for DebugStats {
    fn default() -> Self {
        Self::new()
    }
}

/// Debug renderer trait
pub trait DebugRenderer: Send + Sync {
    /// Begin rendering
    fn begin(&mut self);

    /// End rendering
    fn end(&mut self);

    /// Draw a line
    fn draw_line(&mut self, start: Vec2, end: Vec2, color: DebugColor);

    /// Draw a point
    fn draw_point(&mut self, point: Vec2, size: f32, color: DebugColor);

    /// Draw a circle
    fn draw_circle(&mut self, center: Vec2, radius: f32, color: DebugColor);

    /// Draw a polygon
    fn draw_polygon(&mut self, vertices: &[Vec2], color: DebugColor);

    /// Draw an AABB
    fn draw_aabb(&mut self, aabb: Aabb, color: DebugColor);

    /// Draw a transform
    fn draw_transform(&mut self, transform: Transform, size: f32, color: DebugColor);

    /// Draw a vector
    fn draw_vector(&mut self, start: Vec2, end: Vec2, color: DebugColor);

    /// Draw text
    fn draw_text(&mut self, text: &str, position: Vec2, color: DebugColor);
}

/// Debug color (RGBA)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct DebugColor {
    /// Red component (0-1)
    pub r: f32,
    /// Green component (0-1)
    pub g: f32,
    /// Blue component (0-1)
    pub b: f32,
    /// Alpha component (0-1)
    pub a: f32,
}

impl DebugColor {
    /// Create a new color
    #[inline]
    #[must_use]
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Create a new color with full opacity
    #[inline]
    #[must_use]
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Red
    #[inline]
    #[must_use]
    pub const fn red() -> Self {
        Self::rgb(1.0, 0.0, 0.0)
    }

    /// Green
    #[inline]
    #[must_use]
    pub const fn green() -> Self {
        Self::rgb(0.0, 1.0, 0.0)
    }

    /// Blue
    #[inline]
    #[must_use]
    pub const fn blue() -> Self {
        Self::rgb(0.0, 0.0, 1.0)
    }

    /// White
    #[inline]
    #[must_use]
    pub const fn white() -> Self {
        Self::rgb(1.0, 1.0, 1.0)
    }

    /// Black
    #[inline]
    #[must_use]
    pub const fn black() -> Self {
        Self::rgb(0.0, 0.0, 0.0)
    }

    /// Yellow
    #[inline]
    #[must_use]
    pub const fn yellow() -> Self {
        Self::rgb(1.0, 1.0, 0.0)
    }

    /// Cyan
    #[inline]
    #[must_use]
    pub const fn cyan() -> Self {
        Self::rgb(0.0, 1.0, 1.0)
    }

    /// Magenta
    #[inline]
    #[must_use]
    pub const fn magenta() -> Self {
        Self::rgb(1.0, 0.0, 1.0)
    }

    /// Gray
    #[inline]
    #[must_use]
    pub const fn gray() -> Self {
        Self::rgb(0.5, 0.5, 0.5)
    }
}

impl Default for DebugColor {
    fn default() -> Self {
        Self::white()
    }
}

/// Debug utilities
pub mod utils {
    use super::*;

    /// Generate a random color
    #[inline]
    #[must_use]
    pub fn random_color() -> DebugColor {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        DebugColor::rgb(rng.gen(), rng.gen(), rng.gen())
    }

    /// Interpolate between two colors
    #[inline]
    #[must_use]
    pub fn lerp_color(a: DebugColor, b: DebugColor, t: f32) -> DebugColor {
        DebugColor::new(
            a.r + (b.r - a.r) * t,
            a.g + (b.g - a.g) * t,
            a.b + (b.b - a.b) * t,
            a.a + (b.a - a.a) * t,
        )
    }
}

pub use utils::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_debug_flags() {
        let flags_none = DebugFlags::none();
        assert!(!flags_none.show_colliders);
        assert!(!flags_none.show_aabbs);

        let flags_all = DebugFlags::all();
        assert!(flags_all.show_colliders);
        assert!(flags_all.show_aabbs);

        let mut flags = DebugFlags::default();
        assert!(flags.show_colliders);
        assert!(!flags.show_aabbs);

        flags.enable_all();
        assert!(flags.show_colliders);
        assert!(flags.show_aabbs);

        flags.disable_all();
        assert!(!flags.show_colliders);
        assert!(!flags.show_aabbs);
    }

    #[test]
    fn test_debug_stats() {
        let stats = DebugStats::new();
        assert_eq!(stats.body_count, 0);
        assert_eq!(stats.fps, 0.0);

        let stats2 = DebugStats::default();
        assert_eq!(stats2.body_count, 0);
    }

    #[test]
    fn test_debug_color() {
        let red = DebugColor::red();
        assert_eq!(red.r, 1.0);
        assert_eq!(red.g, 0.0);
        assert_eq!(red.b, 0.0);
        assert_eq!(red.a, 1.0);

        let green = DebugColor::green();
        assert_eq!(green.r, 0.0);
        assert_eq!(green.g, 1.0);
        assert_eq!(green.b, 0.0);

        let blue = DebugColor::blue();
        assert_eq!(blue.r, 0.0);
        assert_eq!(blue.g, 0.0);
        assert_eq!(blue.b, 1.0);

        let custom = DebugColor::new(0.5, 0.5, 0.5, 0.5);
        assert_eq!(custom.r, 0.5);
        assert_eq!(custom.a, 0.5);

        let rgb = DebugColor::rgb(0.25, 0.5, 0.75);
        assert_eq!(rgb.r, 0.25);
        assert_eq!(rgb.g, 0.5);
        assert_eq!(rgb.b, 0.75);
        assert_eq!(rgb.a, 1.0);
    }

    #[test]
    fn test_color_lerp() {
        let a = DebugColor::red();
        let b = DebugColor::blue();

        let mid = lerp_color(a, b, 0.5);
        assert!(approx_eq(mid.r, 0.5));
        assert!(approx_eq(mid.g, 0.0));
        assert!(approx_eq(mid.b, 0.5));

        let to_b = lerp_color(a, b, 1.0);
        assert!(approx_eq(to_b.r, 0.0));
        assert!(approx_eq(to_b.b, 1.0));
    }
}
