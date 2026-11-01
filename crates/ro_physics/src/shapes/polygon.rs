//! Polygon shape implementation.
//!
//! This module provides a convex polygon shape for collision detection
//! and physics simulation.

use std::fmt;

use crate::constants::EPSILON;
use crate::geometry::Aabb;
use crate::math::{Transform, Vec2};
use crate::shapes::{is_convex, MassProperties, ShapeError, ShapeType};

/// A convex polygon shape.
///
/// Vertices are stored relative to the shape's local origin and are
/// normalised to **counter-clockwise** winding on construction, so every edge
/// normal in [`Polygon::normals`] points **outwards**.
///
/// # Invariants
///
/// * at least three, finite, non-degenerate vertices
/// * convex, counter-clockwise winding
/// * `normals[i]` is the outward unit normal of the edge
///   `vertices[i] -> vertices[i + 1]`
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(
    feature = "serialization",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Polygon {
    /// Vertices of the polygon in counter-clockwise order
    pub vertices: Vec<Vec2>,
    /// Outward unit edge normals (`normals[i]` belongs to edge `i`)
    pub normals: Vec<Vec2>,
    /// Centroid (area centre) of the polygon
    pub centroid: Vec2,
}

impl Polygon {
    /// Create a new polygon from vertices, validating the geometry.
    ///
    /// # Errors
    ///
    /// Returns a [`ShapeError`] if the polygon has fewer than three distinct
    /// vertices, contains non-finite coordinates, is degenerate (zero area) or
    /// is concave.
    pub fn try_new(vertices: Vec<Vec2>) -> Result<Self, ShapeError> {
        if vertices.len() < 3 {
            return Err(ShapeError::TooFewVertices(vertices.len()));
        }

        for (index, vertex) in vertices.iter().enumerate() {
            if !vertex.is_finite() {
                return Err(ShapeError::NonFiniteVertex { index });
            }
            if !vertex.validate() {
                return Err(ShapeError::OutOfRangeVertex { index });
            }
        }

        let mut vertices = crate::shapes::utils::clean_polygon_vertices(&vertices);

        if vertices.len() < 3 {
            return Err(ShapeError::Degenerate);
        }

        // Normalise the winding so that the signed area is positive
        // (counter-clockwise in a right-handed coordinate system).
        if crate::shapes::utils::polygon_signed_area(&vertices) < 0.0 {
            vertices.reverse();
        }

        if !is_convex(&vertices) {
            return Err(ShapeError::Concave);
        }

        if crate::shapes::utils::polygon_area(&vertices) < EPSILON {
            return Err(ShapeError::Degenerate);
        }

        let centroid = crate::shapes::utils::polygon_centroid(&vertices);
        let normals = Self::compute_normals(&vertices);

        Ok(Self {
            vertices,
            normals,
            centroid,
        })
    }

    /// Create a new polygon from vertices
    ///
    /// # Panics
    ///
    /// Panics if the polygon is invalid (see [`Polygon::try_new`]). Use
    /// [`Polygon::try_new`] when the vertices come from an untrusted source
    /// such as a scene file.
    #[inline]
    #[must_use]
    pub fn new(vertices: Vec<Vec2>) -> Self {
        match Self::try_new(vertices) {
            Ok(polygon) => polygon,
            Err(error) => panic!("invalid polygon: {error}"),
        }
    }

    /// Create a polygon from a rectangle centred on the local origin
    ///
    /// # Panics
    ///
    /// Panics if the width or height is not positive and finite.
    #[inline]
    #[must_use]
    pub fn from_rectangle(width: f32, height: f32) -> Self {
        assert!(
            width > 0.0 && width.is_finite() && height > 0.0 && height.is_finite(),
            "Polygon::from_rectangle requires positive, finite dimensions"
        );

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

    /// Create a polygon from a box (axis-aligned rectangle) given by half extents
    ///
    /// # Panics
    ///
    /// Panics if either half extent is not positive and finite.
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

    /// Get the outward unit edge normals
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

    /// Calculate the moment of inertia about the centroid for a given mass
    #[inline]
    #[must_use]
    pub fn inertia(&self, mass: f32) -> f32 {
        crate::shapes::utils::polygon_inertia(mass, &self.vertices, self.centroid)
    }

    /// Calculate mass properties for a given density
    #[inline]
    #[must_use]
    pub fn mass_properties(&self, density: f32) -> MassProperties {
        let area = self.area();
        let mass = area * density;
        let inertia = self.inertia(mass);

        MassProperties::new(mass, inertia, self.centroid)
    }

    /// Get the area of the polygon
    #[inline]
    #[must_use]
    pub fn area(&self) -> f32 {
        crate::shapes::utils::polygon_area(&self.vertices)
    }

    /// Calculate the AABB for the polygon in local space
    ///
    /// Returns an invalid AABB for a polygon without vertices.
    #[inline]
    #[must_use]
    pub fn local_aabb(&self) -> Aabb {
        let mut aabb = Aabb::invalid();
        for &vertex in &self.vertices {
            aabb.expand_by_point(vertex);
        }
        aabb
    }

    /// Calculate the AABB for the polygon at a given transform
    ///
    /// The AABB is computed by rotating the local AABB corners, which is exact
    /// for axis-aligned polygons and slightly conservative for rotated ones.
    #[inline]
    #[must_use]
    pub fn aabb(&self, transform: Transform) -> Aabb {
        let local_aabb = self.local_aabb();

        if !local_aabb.is_valid() {
            return local_aabb;
        }

        let mut transformed_aabb = Aabb::invalid();
        for corner in local_aabb.corners() {
            transformed_aabb.expand_by_point(transform.transform_point(corner));
        }
        transformed_aabb
    }

    /// Get the corners of the local AABB
    #[inline]
    #[must_use]
    pub fn local_aabb_corners(&self) -> [Vec2; 4] {
        self.local_aabb().corners()
    }

    /// Get the corners of the local AABB
    #[inline]
    #[must_use]
    pub fn corners(&self) -> [Vec2; 4] {
        self.local_aabb().corners()
    }

    /// Check if a point is inside the polygon (boundary counts as inside)
    ///
    /// Uses the half-space test that is valid because polygons are convex and
    /// their normals point outwards.
    #[inline]
    #[must_use]
    pub fn contains_point(&self, point: Vec2) -> bool {
        if self.vertices.len() < 3 {
            return false;
        }

        for (index, &vertex) in self.vertices.iter().enumerate() {
            let normal = self.normals[index];
            if normal.dot(point - vertex) > 0.0 {
                return false;
            }
        }

        true
    }

    /// Find the closest point on the polygon to a given point
    ///
    /// Points inside the polygon map to themselves.
    #[inline]
    #[must_use]
    pub fn closest_point(&self, point: Vec2) -> Vec2 {
        if self.vertices.is_empty() {
            return Vec2::zero();
        }

        if self.contains_point(point) {
            return point;
        }

        let mut closest = self.vertices[0];
        let mut min_dist_sq = (point - closest).length_squared();

        let n = self.vertices.len();
        for i in 0..n {
            let v1 = self.vertices[i];
            let v2 = self.vertices[(i + 1) % n];

            let edge = v2 - v1;
            let edge_length_sq = edge.length_squared();

            let t = if edge_length_sq > EPSILON * EPSILON {
                (point - v1).dot(edge) / edge_length_sq
            } else {
                0.0
            };
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

    /// Compute outward edge normals from counter-clockwise vertices
    ///
    /// For a counter-clockwise polygon the outward normal of the edge
    /// `v1 -> v2` is the edge rotated by -90 degrees: `(edge.y, -edge.x)`.
    fn compute_normals(vertices: &[Vec2]) -> Vec<Vec2> {
        let n = vertices.len();
        let mut normals = Vec::with_capacity(n);

        for i in 0..n {
            let edge = vertices[(i + 1) % n] - vertices[i];
            let normal = Vec2::new(edge.y, -edge.x);
            let length = normal.length();

            normals.push(if length > EPSILON {
                normal / length
            } else {
                Vec2::zero()
            });
        }

        normals
    }

    /// Get the support point in a given direction
    ///
    /// The support point is the vertex that is farthest in the given direction.
    #[inline]
    #[must_use]
    pub fn support(&self, direction: Vec2) -> Vec2 {
        let Some(first) = self.vertices.first() else {
            return Vec2::zero();
        };

        let mut max_dot = first.dot(direction);
        let mut support_vertex = *first;

        for &vertex in &self.vertices {
            let dot = vertex.dot(direction);
            if dot > max_dot {
                max_dot = dot;
                support_vertex = vertex;
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

        if self.normals.len() != self.vertices.len() {
            return false;
        }

        if !is_convex(&self.vertices) {
            return false;
        }

        for vertex in &self.vertices {
            if !vertex.is_finite() {
                return false;
            }
        }

        true
    }

    /// Validate the polygon, including coordinate range checks
    #[inline]
    #[must_use]
    pub fn validate(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        for vertex in &self.vertices {
            if !vertex.validate() {
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
    use crate::math::{utils::approx_eq, utils::approx_eq_eps, Rot2};

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

        assert!(approx_eq(polygon.area(), 8.0));

        let centroid = polygon.centroid();
        assert!(approx_eq(centroid.x, 0.0));
        assert!(approx_eq(centroid.y, 0.0));
    }

    #[test]
    fn test_inertia_matches_analytic_rectangle() {
        let polygon = Polygon::from_rectangle(2.0, 4.0);
        let mass = 8.0; // Area * density (density = 1)
        let inertia = polygon.inertia(mass);

        // For a rectangle: I = (m * (w^2 + h^2)) / 12
        let expected = (mass * (2.0 * 2.0 + 4.0 * 4.0)) / 12.0;
        assert!(
            approx_eq_eps(inertia, expected, 1e-4),
            "{inertia} != {expected}"
        );
    }

    #[test]
    fn test_inertia_matches_analytic_triangle() {
        // Right triangle with legs of length 2, centroid at (2/3, 2/3).
        let polygon = Polygon::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(0.0, 2.0),
        ]);
        let mass = 2.0 * 2.0 / 2.0; // area = 2 with unit density
        let inertia = polygon.inertia(mass);

        // Polar moment of a triangle about its centroid:
        // I = m / 36 * (a^2 + b^2 + c^2) with side lengths a, b, c.
        // (m / 18 is the moment about the right-angle vertex, twice as large.)
        let a_sq = 4.0;
        let b_sq = 4.0;
        let c_sq = 8.0;
        let expected = mass / 36.0 * (a_sq + b_sq + c_sq);

        assert!(
            approx_eq_eps(inertia, expected, 1e-4),
            "{inertia} != {expected}"
        );
    }

    #[test]
    fn test_mass_properties() {
        let polygon = Polygon::from_rectangle(2.0, 4.0);
        let mp = polygon.mass_properties(1.0);

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

        // Point at center: the point itself
        assert_eq!(
            polygon.closest_point(Vec2::new(0.0, 0.0)),
            Vec2::new(0.0, 0.0)
        );

        // Point inside: the point itself
        assert_eq!(
            polygon.closest_point(Vec2::new(0.5, 0.5)),
            Vec2::new(0.5, 0.5)
        );

        // Point outside (right)
        let closest = polygon.closest_point(Vec2::new(5.0, 0.0));
        assert!(approx_eq(closest.x, 1.0));
        assert!(approx_eq(closest.y, 0.0));

        // Point outside (diagonal)
        let closest = polygon.closest_point(Vec2::new(5.0, 5.0));
        assert!(approx_eq(closest.x, 1.0));
        assert!(approx_eq(closest.y, 1.0));
    }

    #[test]
    fn test_support() {
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        // The rectangle spans [-1, 1] on both axes, so the support along an
        // axis has a dot product of 1.0 with it. With axis aligned directions
        // two vertices tie, and any of them is a valid support point.
        for direction in [
            Vec2::new(1.0, 0.0),
            Vec2::new(-1.0, 0.0),
            Vec2::new(0.0, 1.0),
            Vec2::new(0.0, -1.0),
        ] {
            let support = polygon.support(direction);
            let best = polygon
                .vertices()
                .iter()
                .map(|vertex| vertex.dot(direction))
                .fold(f32::NEG_INFINITY, f32::max);

            assert!(approx_eq(support.dot(direction), best));
        }

        // A diagonal direction has a single maximal vertex
        let support = polygon.support(Vec2::new(1.0, 1.0));
        assert_eq!(support, Vec2::new(1.0, 1.0));
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
    fn test_aabb_rotated() {
        // A 2x2 square rotated by 45 degrees spans 2 * sqrt(2).
        let polygon = Polygon::from_rectangle(2.0, 2.0);
        let transform = Transform::from_rotation(Rot2::new(crate::constants::PI_OVER_FOUR));

        let aabb = polygon.aabb(transform);
        let expected = 2.0 * (2.0_f32).sqrt();
        assert!(approx_eq_eps(aabb.width(), expected, 1e-5));
        assert!(approx_eq_eps(aabb.height(), expected, 1e-5));
        assert!(approx_eq_eps(aabb.center().x, 0.0, 1e-6));
        assert!(approx_eq_eps(aabb.center().y, 0.0, 1e-6));
    }

    #[test]
    fn test_validation() {
        let polygon1 = Polygon::from_rectangle(2.0, 2.0);
        assert!(polygon1.is_valid());
        assert!(polygon1.validate());

        // Out of range coordinates fail full validation
        let huge = Polygon::try_new(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(1.0, crate::constants::MAX_FLOAT * 10.0),
        ]);
        assert!(huge.is_err());
    }

    #[test]
    fn test_try_new_rejects_invalid_geometry() {
        // Too few vertices
        assert_eq!(
            Polygon::try_new(vec![Vec2::zero(), Vec2::new(1.0, 0.0)]),
            Err(ShapeError::TooFewVertices(2))
        );

        // Non-finite vertex
        assert!(matches!(
            Polygon::try_new(vec![
                Vec2::new(-1.0, -1.0),
                Vec2::new(f32::NAN, -1.0),
                Vec2::new(1.0, 1.0),
            ]),
            Err(ShapeError::NonFiniteVertex { index: 1 })
        ));

        // Degenerate: all points collinear
        assert_eq!(
            Polygon::try_new(vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 1.0),
                Vec2::new(2.0, 2.0),
            ]),
            Err(ShapeError::Degenerate)
        );

        // Concave (a chevron)
        assert_eq!(
            Polygon::try_new(vec![
                Vec2::new(-1.0, -1.0),
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, -1.0),
                Vec2::new(0.0, 1.0),
            ]),
            Err(ShapeError::Concave)
        );
    }

    #[test]
    #[should_panic]
    fn test_new_panics_on_invalid_geometry() {
        let _ = Polygon::new(vec![Vec2::zero(), Vec2::new(1.0, 0.0)]);
    }

    #[test]
    fn test_shape_type() {
        assert_eq!(Polygon::shape_type(), ShapeType::Polygon);
    }

    #[test]
    fn test_normals_are_outward() {
        let polygon = Polygon::from_rectangle(2.0, 2.0);
        let normals = polygon.normals();

        assert_eq!(normals.len(), 4);

        let vertex_count = polygon.vertices.len();
        for (i, (&v1, &normal)) in polygon.vertices.iter().zip(normals).take(4).enumerate() {
            let v2 = polygon.vertices[(i + 1) % vertex_count];
            let edge = v2 - v1;

            // Normal is perpendicular to the edge and unit length
            assert!(approx_eq_eps(edge.dot(normal), 0.0, 1e-5));
            assert!(approx_eq_eps(normal.length(), 1.0, 1e-5));

            // The normal points away from the polygon centre
            let edge_midpoint = (v1 + v2) / 2.0;
            assert!(
                normal.dot(edge_midpoint - polygon.centroid()) > 0.0,
                "normal {i} points inwards"
            );

            // Moving a point along the normal leaves the polygon
            assert!(!polygon.contains_point(edge_midpoint + normal * 0.01));
            assert!(polygon.contains_point(edge_midpoint - normal * 0.01));
        }
    }

    #[test]
    fn test_normals_of_triangle() {
        let polygon = Polygon::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, 1.0),
        ]);

        // Bottom edge from (0,0) to (1,0): outward normal is (0,-1)
        assert_eq!(polygon.normals()[0], Vec2::new(0.0, -1.0));
        // Hypotenuse from (1,0) to (0,1): outward normal is (1,1)/sqrt(2)
        let expected = Vec2::new(1.0, 1.0).normalize();
        assert_eq!(polygon.normals()[1], expected);
    }

    #[test]
    fn test_winding_is_normalised() {
        // Create vertices in clockwise order
        let vertices_cw = vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(-1.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, -1.0),
        ];

        let polygon = Polygon::new(vertices_cw);

        // The constructor normalises the winding to counter-clockwise
        assert!(crate::shapes::utils::polygon_signed_area(&polygon.vertices) > 0.0);
        assert!(polygon.area() > 0.0);
    }

    #[test]
    fn test_default() {
        let polygon = Polygon::default();
        assert_eq!(polygon.vertex_count(), 4);
    }
}
