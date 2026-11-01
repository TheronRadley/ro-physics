//! Shape implementations for the physics engine.
//!
//! This module provides various 2D shape types for collision detection
//! and physics simulation.

mod circle;
mod polygon;
mod shape;
mod mass_properties;

pub use circle::Circle;
pub use polygon::Polygon;
pub use shape::{Shape, ShapeType};
pub use mass_properties::MassProperties;

use std::fmt;

/// Reasons why a piece of geometry was rejected.
///
/// Returned by [`Polygon::try_new`] and [`Circle::try_new`] instead of
/// panicking, so that invalid scenes can be reported rather than aborting the
/// process.
#[derive(Debug, Clone, PartialEq)]
pub enum ShapeError {
    /// The shape has fewer vertices than the minimum (3 for polygons)
    TooFewVertices(usize),
    /// A vertex is NaN or infinite
    NonFiniteVertex {
        /// Index of the offending vertex
        index: usize,
    },
    /// A vertex lies outside the supported coordinate range
    OutOfRangeVertex {
        /// Index of the offending vertex
        index: usize,
    },
    /// The polygon has (nearly) zero area
    Degenerate,
    /// The polygon is not convex
    Concave,
    /// The radius is negative, non-finite or out of range
    InvalidRadius(f32),
}

impl fmt::Display for ShapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooFewVertices(count) => {
                write!(f, "polygon needs at least 3 vertices, got {count}")
            }
            Self::NonFiniteVertex { index } => {
                write!(f, "vertex {index} is not finite")
            }
            Self::OutOfRangeVertex { index } => {
                write!(f, "vertex {index} is outside the supported coordinate range")
            }
            Self::Degenerate => write!(f, "polygon is degenerate (zero area)"),
            Self::Concave => write!(f, "polygon is not convex"),
            Self::InvalidRadius(radius) => write!(f, "invalid circle radius: {radius}"),
        }
    }
}

impl std::error::Error for ShapeError {}

/// Shape utilities
pub mod utils {
    use crate::constants::EPSILON;
    use crate::math::Vec2;

    /// Calculate the moment of inertia for a circle
    #[inline]
    #[must_use]
    pub fn circle_inertia(mass: f32, radius: f32) -> f32 {
        0.5 * mass * radius * radius
    }

    /// Calculate the moment of inertia for a rectangle about its centre
    #[inline]
    #[must_use]
    pub fn rectangle_inertia(mass: f32, width: f32, height: f32) -> f32 {
        (mass * (width * width + height * height)) / 12.0
    }

    /// Calculate the polar moment of inertia of a polygon about a reference
    /// point (usually the centroid)
    ///
    /// Uses the standard decomposition of the polygon into triangles fanned out
    /// from `centroid`:
    ///
    /// ```text
    /// I = m * sum(|D_i| * (v_i . v_i + v_i . v_{i+1} + v_{i+1} . v_{i+1}))
    ///         / (12 * area)
    /// ```
    ///
    /// where `D_i = cross(v_i, v_{i+1})` and the vertices are relative to
    /// `centroid`. Returns 0 for degenerate polygons.
    #[inline]
    #[must_use]
    pub fn polygon_inertia(mass: f32, vertices: &[Vec2], centroid: Vec2) -> f32 {
        let n = vertices.len();
        if n < 3 || !(mass > 0.0) {
            return 0.0;
        }

        let mut double_area = 0.0;
        let mut numerator = 0.0;

        for i in 0..n {
            let v1 = vertices[i] - centroid;
            let v2 = vertices[(i + 1) % n] - centroid;

            let cross = v1.cross(v2);
            let dot = v1.dot(v1) + v1.dot(v2) + v2.dot(v2);

            double_area += cross;
            numerator += cross * dot;
        }

        let area = 0.5 * double_area.abs();
        if area <= EPSILON {
            return 0.0;
        }

        (mass * numerator.abs()) / (12.0 * area)
    }

    /// Calculate the centroid (area centre) of a polygon
    ///
    /// Falls back to the average of the vertices for degenerate polygons.
    #[inline]
    #[must_use]
    pub fn polygon_centroid(vertices: &[Vec2]) -> Vec2 {
        let n = vertices.len();
        if n == 0 {
            return Vec2::zero();
        }

        let mut area = 0.0;
        let mut centroid = Vec2::zero();

        for i in 0..n {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % n];

            let cross = v1.cross(v2);
            area += cross;
            centroid += (v1 + v2) * cross;
        }

        area /= 2.0;
        if area.abs() < EPSILON {
            // Degenerate polygon, return average of vertices
            let mut sum = Vec2::zero();
            for v in vertices {
                sum += *v;
            }
            sum / n as f32
        } else {
            centroid / (6.0 * area)
        }
    }

    /// Calculate the signed area of a polygon
    ///
    /// Positive for counter-clockwise winding, negative for clockwise.
    #[inline]
    #[must_use]
    pub fn polygon_signed_area(vertices: &[Vec2]) -> f32 {
        let n = vertices.len();
        if n < 3 {
            return 0.0;
        }

        let mut double_area = 0.0;
        for i in 0..n {
            double_area += vertices[i].cross(vertices[(i + 1) % n]);
        }

        double_area / 2.0
    }

    /// Calculate the (unsigned) area of a polygon
    #[inline]
    #[must_use]
    pub fn polygon_area(vertices: &[Vec2]) -> f32 {
        polygon_signed_area(vertices).abs()
    }

    /// Check if a polygon is convex
    ///
    /// Every polygon with fewer than four vertices is convex as long as it has
    /// a non-zero area.
    #[inline]
    #[must_use]
    pub fn is_convex(vertices: &[Vec2]) -> bool {
        let n = vertices.len();
        if n < 3 {
            return false;
        }
        if n == 3 {
            return polygon_area(vertices) > EPSILON;
        }

        let mut sign = 0.0_f32;

        for i in 0..n {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % n];
            let v3 = vertices[(i + 2) % n];

            let cross = (v2 - v1).cross(v3 - v2);

            if cross.abs() <= EPSILON {
                // Collinear (or degenerate) edges do not break convexity
                continue;
            }

            if sign == 0.0 {
                sign = cross;
            } else if cross * sign < 0.0 {
                return false;
            }
        }

        true
    }

    /// Normalize polygon winding to counter-clockwise
    ///
    /// Returns a copy with counter-clockwise winding together with a flag that
    /// is `false` when the input had to be reversed.
    #[inline]
    #[must_use]
    pub fn normalize_winding(vertices: &[Vec2]) -> (Vec<Vec2>, bool) {
        if vertices.len() < 3 {
            return (vertices.to_vec(), true);
        }

        if polygon_signed_area(vertices) < 0.0 {
            let mut reversed = vertices.to_vec();
            reversed.reverse();
            (reversed, false)
        } else {
            (vertices.to_vec(), true)
        }
    }

    /// Remove duplicate, collinear and degenerate vertices from a polygon
    ///
    /// A vertex is dropped when it coincides with its neighbour, or when the
    /// turn it forms with its neighbours is (numerically) zero. The result may
    /// contain fewer than three vertices, in which case the polygon is
    /// degenerate.
    #[inline]
    #[must_use]
    pub fn clean_polygon_vertices(vertices: &[Vec2]) -> Vec<Vec2> {
        let n = vertices.len();
        if n < 3 {
            return Vec::new();
        }

        let mut cleaned: Vec<Vec2> = Vec::with_capacity(n);

        for i in 0..n {
            let previous = vertices[(i + n - 1) % n];
            let current = vertices[i];
            let next = vertices[(i + 1) % n];

            let incoming = current - previous;
            let outgoing = next - current;

            let incoming_length_sq = incoming.length_squared();
            let outgoing_length_sq = outgoing.length_squared();

            // Drop coincident vertices
            if incoming_length_sq <= EPSILON * EPSILON
                || outgoing_length_sq <= EPSILON * EPSILON
            {
                continue;
            }

            // Drop collinear vertices (zero turn)
            let cross = incoming.cross(outgoing);
            let scale = (incoming_length_sq * outgoing_length_sq).sqrt();
            if cross.abs() <= EPSILON * scale {
                continue;
            }

            cleaned.push(current);
        }

        cleaned
    }
}

pub use utils::*;
