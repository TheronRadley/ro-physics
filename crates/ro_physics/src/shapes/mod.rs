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

/// Shape utilities
pub mod utils {
    use super::*;
    use crate::math::Vec2;

    /// Calculate the moment of inertia for a circle
    #[inline]
    #[must_use]
    pub fn circle_inertia(mass: f32, radius: f32) -> f32 {
        0.5 * mass * radius * radius
    }

    /// Calculate the moment of inertia for a rectangle
    #[inline]
    #[must_use]
    pub fn rectangle_inertia(mass: f32, width: f32, height: f32) -> f32 {
        (mass * (width * width + height * height)) / 12.0
    }

    /// Calculate the moment of inertia for a polygon
    ///
    /// Uses the parallel axis theorem with vertices relative to centroid.
    #[inline]
    #[must_use]
    pub fn polygon_inertia(mass: f32, vertices: &[Vec2], centroid: Vec2) -> f32 {
        let mut inertia = 0.0;
        let n = vertices.len();

        for i in 0..n {
            let v1 = vertices[i] - centroid;
            let v2 = vertices[(i + 1) % n] - centroid;
            
            let cross = v1.cross(v2);
            let dot = v1.dot(v1) + v1.dot(v2) + v2.dot(v2);
            
            inertia += cross * dot;
        }

        (mass * inertia.abs()) / 48.0
    }

    /// Calculate the centroid of a polygon
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
        if area.abs() < crate::constants::EPSILON {
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

    /// Calculate the area of a polygon
    #[inline]
    #[must_use]
    pub fn polygon_area(vertices: &[Vec2]) -> f32 {
        let n = vertices.len();
        if n < 3 {
            return 0.0;
        }

        let mut area = 0.0;
        for i in 0..n {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % n];
            area += v1.cross(v2);
        }

        area.abs() / 2.0
    }

    /// Check if a polygon is convex
    #[inline]
    #[must_use]
    pub fn is_convex(vertices: &[Vec2]) -> bool {
        if vertices.len() < 3 {
            return true;
        }

        let n = vertices.len();
        let mut prev_cross = 0.0;

        for i in 0..n {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % n];
            let v3 = vertices[(i + 2) % n];
            
            let edge1 = v2 - v1;
            let edge2 = v3 - v2;
            
            let cross = edge1.cross(edge2);
            
            if i == 0 {
                prev_cross = cross;
            } else if cross * prev_cross < -crate::constants::EPSILON {
                return false;
            }
        }

        true
    }

    /// Normalize polygon winding to counter-clockwise
    #[inline]
    #[must_use]
    pub fn normalize_winding(vertices: &mut [Vec2]) -> bool {
        if vertices.len() < 3 {
            return true;
        }

        let mut area = 0.0;
        let n = vertices.len();
        
        for i in 0..n {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % n];
            area += v1.cross(v2);
        }

        if area < 0.0 {
            vertices.reverse();
            false
        } else {
            true
        }
    }

    /// Remove duplicate and degenerate vertices from a polygon
    #[inline]
    pub fn clean_polygon_vertices(vertices: &mut Vec<Vec2>) {
        if vertices.len() < 3 {
            return;
        }

        let mut cleaned = Vec::with_capacity(vertices.len());
        let n = vertices.len();

        for i in 0..n {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % n];
            let v3 = vertices[(i + 2) % n];
            
            let edge1 = v2 - v1;
            let edge2 = v3 - v2;
            
            // Skip if edge1 is degenerate
            if edge1.length_squared() < crate::constants::EPSILON * crate::constants::EPSILON {
                continue;
            }
            
            // Skip if edge1 and edge2 are colinear
            let cross = edge1.cross(edge2);
            if cross.abs() < crate::constants::EPSILON {
                continue;
            }

            cleaned.push(v1);
        }

        // Ensure we have at least 3 vertices
        if cleaned.len() < 3 {
            *vertices = Vec::new();
        } else {
            *vertices = cleaned;
        }
    }
}

pub use utils::*;
