//! Polygon shape implementation.
//!
//! This module provides a convex polygon shape for collision detection
//! and physics simulation.

use std::fmt;

use crate::constants::EPSILON;
use crate::geometry::Aabb;
use crate::math::{Rot2, Transform, Vec2};
use crate::shapes::{MassProperties, ShapeType, is_convex, normalize_winding};

/// A convex polygon shape.
///
/// Polygons are defined by a list of vertices in counter-clockwise order.
/// The vertices are stored relative to the shape's local origin.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Polygon {
    /// Vertices of the polygon in counter-clockwise order
    pub vertices: Vec<Vec2>,
    /// Edge normals (computed from vertices)
    pub normals: Vec<Vec2>,
    /// Centroid of the polygon
    pub centroid: Vec2,
    /// Cached AABB in local space
    local_aabb: Option<Aabb>,
}

impl Polygon {
    /// Create a new polygon from vertices
    ///
    /// # Panics
    ///
    /// Panics if the polygon is not convex or has fewer than 3 vertices.
    #[inline]
    #[must_use]
    pub fn new(vertices: Vec<Vec2>) -> Self {
        debug_assert!(
            vertices.len() >= 3,
            "Polygon must have at least 3 vertices"
        );
        debug_assert!(
            is_convex(&vertices),
            "Polygon must be convex"
        );

        let mut vertices = vertices;
        
        // Normalize winding to counter-clockwise
        normalize_winding(&mut vertices);
        
        // Compute centroid
        let centroid = super::utils::polygon_centroid(&vertices);
        
        // Compute normals
        let normals = Self::compute_normals(&vertices);
        
        Self {
            vertices,
            normals,
            centroid,
            local_aabb: None,
        }
    }

    /// Create a polygon from a rectangle
    #[inline]
    #[must_use]
    pub fn from_rectangle(width: f32, height: f32) -> Self {
        let half_width = width / 2.0;
        let half_height = height / 2.0;

        let vertices = vec![
            Vec2::new(-half_width, -half_height),
            Vec2::new(half_width, -half_height),
            Vec2::new(half_width, half_height),
            Vec2::new(-half_width, half_height),
        ];

        Self::new(vertices)
    }

    /// Create a polygon from a box (axis-aligned rectangle)
    #[inline]
    #[must_use]
    pub fn from_box(half_extents: Vec2) -> Self {
        Self::from_rectangle(half_extents.x * 2.0, half_extents.y * 2.0)
    }

    /// Get the vertices
    #[inline]
    #[must_use]
    pub fn vertices(&self) -> &[Vec2] {
        &self.vertices
    }

    /// Get the normals
    #[inline]
    #[must_use]
    pub fn normals(&self) -> &[Vec2] {
        &self.normals
    }

    /// Get the centroid
    #[inline]
    #[must_use]
    pub fn centroid(&self) -> Vec2 {
        self.centroid
    }

    /// Get the number of vertices
    #[inline]
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Get the number of edges
    #[inline]
    #[must_use]
    pub fn edge_count(&self) -> usize {
        self.vertices.len()
    }

    /// Calculate the moment of inertia for a given mass
    #[inline]
    #[must_use]
    pub fn inertia(self, mass: f32) -> f32 {
        super::utils::polygon_inertia(mass, &self.vertices, self.centroid)
    }

    /// Calculate mass properties for a given density
    #[inline]
    #[must_use]
    pub fn mass_properties(self, density: f32) -> MassProperties {
        let area = self.area();
        let mass = area * density;
        let inertia = self.inertia(mass);

        MassProperties::new(mass, inertia, self.centroid)
    }

    /// Get the area of the polygon
    #[inline]
    #[must_use]
    pub fn area(self) -> f32 {
        super::utils::polygon_area(&self.vertices)
    }

    /// Calculate the AABB for the polygon in local space
    #[inline]
    #[must_use]
    pub fn local_aabb(&self) -> Aabb {
        if let Some(aabb) = self.local_aabb {
            return aabb;
        }

        let mut aabb = Aabb::invalid();
        for &v in &self.vertices {
            aabb.expand_by_point(v);
        }

        self.local_aabb = Some(aabb);
        aabb
    }

    /// Calculate the AABB for the polygon at a given transform
    #[inline]
    #[must_use]
    pub fn aabb(self, transform: Transform) -> Aabb {
        let local_aabb = self.local_aabb();
        
        // Transform the AABB corners and create a new AABB
        let corners = local_aabb.corners();
        let mut transformed_aabb = Aabb::invalid();
        
        for corner in corners {
            let transformed = transform.transform_point(corner);
            transformed_aabb.expand_by_point(transformed);
        }
        
        transformed_aabb
    }

    /// Get the corners of the local AABB
    #[inline]
    #[must_use]
    pub fn local_aabb_corners(&self) -> [Vec2; 4] {
        let aabb = self.local_aabb();
        aabb.corners()
    }

    /// Check if a point is inside the polygon
    #[inline]
    #[must_use]
    pub fn contains_point(self, point: Vec2) -> bool {
        // Use the winding number algorithm
        let mut winding = 0;
        let n = self.vertices.len();

        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];

            // Check if point is exactly on the edge
            if Self::point_on_edge(point, v1, v2) {
                return true;
            }

            // Check if the ray from point to the right crosses the edge
            if v1.y <= point.y {
                if v2.y > point.y && Self::is_ray_crossing_edge(point, v1, v2) {
                    winding += 1;
                }
            } else {
                if v2.y <= point.y && Self::is_ray_crossing_edge(point, v2, v1) {
                    winding -= 1;
                }
            }
        }

        winding != 0
    }

    /// Find the closest point on the polygon to a given point
    #[inline]
    #[must_use]
    pub fn closest_point(self, point: Vec2) -> Vec2 {
        let mut closest = self.vertices[0];
        let mut min_dist_sq = f32::INFINITY;

        // Check vertices
        for &v in &self.vertices {
            let dist_sq = (point - v).length_squared();
            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest = v;
            }
        }

        // Check edges
        let n = self.vertices.len();
        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];
            
            let edge = v2 - v1;
            let to_point = point - v1;
            
            let t = to_point.dot(edge) / edge.length_squared();
            let t_clamped = t.clamp(0.0, 1.0);
            
            let projection = v1 + edge * t_clamped;
            let dist_sq = (point - projection).length_squared();
            
            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest = projection;
            }
        }

        closest
    }

    /// Compute edge normals from vertices
    fn compute_normals(vertices: &[Vec2]) -> Vec<Vec2> {
        let n = vertices.len();
        let mut normals = Vec::with_capacity(n);

        for i in 0..n {
            let v1 = vertices[i];
            let v2 = vertices[(i + 1) % n];
            
            let edge = v2 - v1;
            let normal = Vec2::new(-edge.y, edge.x).normalize();
            
            normals.push(normal);
        }

        normals
    }

    /// Check if a point is on an edge
    fn point_on_edge(point: Vec2, v1: Vec2, v2: Vec2) -> bool {
        let edge = v2 - v1;
        let to_point = point - v1;
        
        // Check if point is colinear with the edge
        let cross = edge.cross(to_point);
        if cross.abs() > EPSILON {
            return false;
        }

        // Check if point is within the edge bounds
        let dot = to_point.dot(edge);
        dot >= 0.0 && dot <= edge.length_squared()
    }

    /// Check if a ray to the right crosses an edge
    fn is_ray_crossing_edge(point: Vec2, v1: Vec2, v2: Vec2) -> bool {
        // Check if the edge crosses the horizontal ray from point to the right
        let edge = v2 - v1;
        let to_point = point - v1;
        
        // The edge must cross the horizontal line at point.y
        // and be to the right of point
        if v1.y > point.y != v2.y > point.y {
            // Compute x coordinate of intersection
            let t = (point.y - v1.y) / edge.y;
            let x = v1.x + edge.x * t;
            return x > point.x;
        }
        
        false
    }

    /// Get the support point in a given direction
    ///
    /// The support point is the vertex that is farthest in the given direction.
    #[inline]
    #[must_use]
    pub fn support(self, direction: Vec2) -> Vec2 {
        let mut max_dot = f32::NEG_INFINITY;
        let mut support_vertex = self.vertices[0];

        for &v in &self.vertices {
            let dot = v.dot(direction);
            if dot > max_dot {
                max_dot = dot;
                support_vertex = v;
            }
        }

        support_vertex
    }

    /// Check if the polygon is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        if self.vertices.len() < 3 {
            return false;
        }

        if !is_convex(&self.vertices) {
            return false;
        }

        for v in &self.vertices {
            if !v.is_finite() {
                return false;
            }
        }

        true
    }

    /// Validate the polygon
    #[inline]
    #[must_use]
    pub fn validate(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        for v in &self.vertices {
            if v.abs().max_element() >= crate::constants::MAX_FLOAT {
                return false;
            }
        }

        true
    }

    /// Get the shape type
    #[inline]
    #[must_use]
    pub const fn shape_type() -> ShapeType {
        ShapeType::Polygon
    }
}

impl Polygon {
    /// Get the corners of the AABB
    pub fn corners(&self) -> [Vec2; 4] {
        let aabb = self.local_aabb();
        [
            Vec2::new(aabb.min.x, aabb.min.y),
            Vec2::new(aabb.max.x, aabb.min.y),
            Vec2::new(aabb.max.x, aabb.max.y),
            Vec2::new(aabb.min.x, aabb.max.y),
        ]
    }
}

impl Default for Polygon {
    fn default() -> Self {
        // Default to a unit square
        Self::from_rectangle(1.0, 1.0)
    }
}

impl fmt::Display for Polygon {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Polygon(vertices: {}, centroid: {})",
            self.vertices.len(),
            self.centroid
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_construction() {
        let vertices = vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(-1.0, 1.0),
        ];
        
        let polygon = Polygon::new(vertices);
        assert_eq!(polygon.vertex_count(), 4);
    }

    #[test]
    fn test_from_rectangle() {
        let polygon = Polygon::from_rectangle(2.0, 4.0);
        assert_eq!(polygon.vertex_count(), 4);
        
        let aabb = polygon.local_aabb();
        assert_eq!(aabb.min, Vec2::new(-1.0, -2.0));
        assert_eq!(aabb.max, Vec2::new(1.0, 2.0));
    }

    #[test]
    fn test_from_box() {
        let polygon = Polygon::from_box(Vec2::new(1.0, 2.0));
        assert_eq!(polygon.vertex_count(), 4);
        
        let aabb = polygon.local_aabb();
        assert_eq!(aabb.min, Vec2::new(-1.0, -2.0));
        assert_eq!(aabb.max, Vec2::new(1.0, 2.0));
    }

    #[test]
    fn test_properties() {
        let polygon = Polygon::from_rectangle(2.0, 4.0);
        
        let area = polygon.area();
        assert!(approx_eq(area, 8.0));
        
        let centroid = polygon.centroid();
        assert!(approx_eq(centroid.x, 0.0));
        assert!(approx_eq(centroid.y, 0.0));
    }

    #[test]
    fn test_inertia() {
        let polygon = Polygon::from_rectangle(2.0, 4.0);
        let mass = 8.0; // Area * density (density = 1)
        let inertia = polygon.inertia(mass);
        
        // For a rectangle: I = (m * (w^2 + h^2)) / 12
        // = (8 * (4 + 16)) / 12 = (8 * 20) / 12 = 160 / 12 = 13.333
        let expected = (mass * (2.0 * 2.0 + 4.0 * 4.0)) / 12.0;
        assert!(approx_eq(inertia, expected, 1e-5));
    }

    #[test]
    fn test_mass_properties() {
        let polygon = Polygon::from_rectangle(2.0, 4.0);
        let density = 1.0;
        let mp = polygon.mass_properties(density);
        
        assert!(approx_eq(mp.mass, 8.0));
        assert!(approx_eq(mp.center_of_mass.x, 0.0));
        assert!(approx_eq(mp.center_of_mass.y, 0.0));
    }

    #[test]
    fn test_contains_point() {
        let polygon = Polygon::from_rectangle(2.0, 2.0);
        
        // Point at center
        assert!(polygon.contains_point(Vec2::new(0.0, 0.0)));
        
        // Point at vertex
        assert!(polygon.contains_point(Vec2::new(1.0, 1.0)));
        
        // Point inside
        assert!(polygon.contains_point(Vec2::new(0.5, 0.5)));
        
        // Point on edge
        assert!(polygon.contains_point(Vec2::new(1.0, 0.0)));
        
        // Point outside
        assert!(!polygon.contains_point(Vec2::new(2.0, 0.0)));
    }

    #[test]
    fn test_closest_point() {
        let polygon = Polygon::from_rectangle(2.0, 2.0);
        
        // Point at center
        assert_eq!(
            polygon.closest_point(Vec2::new(0.0, 0.0)),
            Vec2::new(0.0, 0.0)
        );
        
        // Point inside
        assert_eq!(
            polygon.closest_point(Vec2::new(0.5, 0.5)),
            Vec2::new(0.5, 0.5)
        );
        
        // Point outside (right)
        let closest = polygon.closest_point(Vec2::new(5.0, 0.0));
        assert!(approx_eq(closest.x, 1.0));
        assert!(approx_eq(closest.y, 0.0));
    }

    #[test]
    fn test_support() {
        let polygon = Polygon::from_rectangle(2.0, 2.0);
        
        // Support in positive X direction
        let support_x = polygon.support(Vec2::new(1.0, 0.0));
        assert_eq!(support_x, Vec2::new(1.0, -1.0));
        
        // Support in negative X direction
        let support_neg_x = polygon.support(Vec2::new(-1.0, 0.0));
        assert_eq!(support_neg_x, Vec2::new(-1.0, 1.0));
        
        // Support in positive Y direction
        let support_y = polygon.support(Vec2::new(0.0, 1.0));
        assert_eq!(support_y, Vec2::new(1.0, 1.0));
    }

    #[test]
    fn test_aabb() {
        let polygon = Polygon::from_rectangle(2.0, 4.0);
        let transform = Transform::new(Vec2::new(10.0, 20.0), Rot2::identity());
        
        let aabb = polygon.aabb(transform);
        assert_eq!(aabb.min, Vec2::new(9.0, 18.0));
        assert_eq!(aabb.max, Vec2::new(11.0, 22.0));
    }

    #[test]
    fn test_validation() {
        let polygon1 = Polygon::from_rectangle(2.0, 2.0);
        assert!(polygon1.is_valid());
        assert!(polygon1.validate());

        // Test with NaN vertex
        let vertices_with_nan = vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(f32::NAN, -1.0),
            Vec2::new(1.0, 1.0),
        ];
        // This would panic in debug mode due to the assertion
        // In release mode, we can't test this easily

        // Test with too few vertices - would panic
    }

    #[test]
    fn test_shape_type() {
        assert_eq!(Polygon::shape_type(), ShapeType::Polygon);
    }

    #[test]
    fn test_normals() {
        let polygon = Polygon::from_rectangle(2.0, 2.0);
        let normals = polygon.normals();
        
        assert_eq!(normals.len(), 4);
        
        // Check that normals are perpendicular to edges
        for i in 0..4 {
            let v1 = polygon.vertices[i];
            let v2 = polygon.vertices[(i + 1) % 4];
            let edge = v2 - v1;
            let normal = normals[i];
            
            // Normal should be perpendicular to edge
            assert!(approx_eq(edge.dot(normal), 0.0, 1e-5));
            
            // Normal should be approximately unit length
            assert!(approx_eq(normal.length(), 1.0, 1e-5));
        }
    }

    #[test]
    fn test_winding() {
        // Create vertices in clockwise order
        let vertices_cw = vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(-1.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, -1.0),
        ];
        
        let polygon = Polygon::new(vertices_cw);
        
        // The constructor should have normalized the winding
        // Check by verifying the area is positive
        let area = polygon.area();
        assert!(area > 0.0);
    }

    #[test]
    fn test_default() {
        let polygon = Polygon::default();
        assert_eq!(polygon.vertex_count(), 4);
    }
}
