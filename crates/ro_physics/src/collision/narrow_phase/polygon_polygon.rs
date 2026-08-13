//! Polygon-polygon collision detection using Separating Axis Theorem (SAT).
//!
//! This module provides robust collision detection between two convex polygons.
//! The algorithm uses SAT to find separating axes and generates proper contact manifolds.

use crate::constants::EPSILON;
use crate::math::{Rot2, Transform, Vec2};
use crate::shapes::Polygon;
use super::NarrowPhaseConfig;
use super::super::{ContactId, ContactManifold};

/// Detect collision between two polygons using SAT.
///
/// Algorithm:
/// 1. Transform both polygons to world space
/// 2. Test all edges of both polygons as separating axes
/// 3. If any axis separates, return None
/// 4. Find the axis with minimum penetration
/// 5. Generate contact manifold on that axis
///
/// Returns a contact manifold if they overlap, None otherwise.
#[inline]
#[must_use]
pub fn polygon_polygon_collision(
    polygon_a: &Polygon,
    transform_a: Transform,
    polygon_b: &Polygon,
    transform_b: Transform,
    config: &NarrowPhaseConfig,
) -> Option<ContactManifold> {
    // Get vertices in world space
    let vertices_a: Vec<Vec2> = polygon_a
        .vertices()
        .iter()
        .map(|&v| transform_a.transform_point(v))
        .collect();
    
    let vertices_b: Vec<Vec2> = polygon_b
        .vertices()
        .iter()
        .map(|&v| transform_b.transform_point(v))
        .collect();
    
    // Get edges in world space
    let edges_a: Vec<Vec2> = get_edges(&vertices_a);
    let edges_b: Vec<Vec2> = get_edges(&vertices_b);
    
    // Get normals in world space
    let normals_a: Vec<Vec2> = edges_a.iter().map(|&e| Vec2::new(-e.y, e.x).normalize()).collect();
    let normals_b: Vec<Vec2> = edges_b.iter().map(|&e| Vec2::new(-e.y, e.x).normalize()).collect();
    
    // Find the separating axis using SAT
    let sat_result = find_separating_axis(
        &vertices_a, &vertices_b,
        &normals_a, &normals_b,
    );
    
    match sat_result {
        SATResult::Separated => None,
        SATResult::Overlapping { min_axis, min_penetration } => {
            // Generate contact manifold
            generate_polygon_polygon_manifold(
                polygon_a, transform_a,
                polygon_b, transform_b,
                &vertices_a, &vertices_b,
                &edges_a, &edges_b,
                min_axis, min_penetration,
                config,
            )
        }
    }
}

/// Result of SAT test
#[derive(Debug, Clone, Copy)]
enum SATResult {
    /// Polygons are separated
    Separated,
    /// Polygons overlap, with minimum penetration info
    Overlapping {
        min_axis: Vec2,
        min_penetration: f32,
    },
}

/// Find separating axis using SAT
fn find_separating_axis(
    vertices_a: &[Vec2],
    vertices_b: &[Vec2],
    normals_a: &[Vec2],
    normals_b: &[Vec2],
) -> SATResult {
    let mut min_penetration = f32::INFINITY;
    let mut min_axis: Option<Vec2> = None;
    
    // Test all normals from polygon A
    for &normal in normals_a {
        let (penetration, separated) = test_axis(vertices_a, vertices_b, normal);
        
        if separated {
            return SATResult::Separated;
        }
        
        if penetration < min_penetration {
            min_penetration = penetration;
            min_axis = Some(normal);
        }
    }
    
    // Test all normals from polygon B
    for &normal in normals_b {
        let (penetration, separated) = test_axis(vertices_a, vertices_b, normal);
        
        if separated {
            return SATResult::Separated;
        }
        
        if penetration < min_penetration {
            min_penetration = penetration;
            min_axis = Some(normal);
        }
    }
    
    // No separating axis found
    SATResult::Overlapping {
        min_axis: min_axis.unwrap_or_else(|| Vec2::unit_x()),
        min_penetration,
    }
}

/// Test a single axis for separation
///
/// Returns (penetration_depth, is_separated)
fn test_axis(vertices_a: &[Vec2], vertices_b: &[Vec2], axis: Vec2) -> (f32, bool) {
    let (min_a, max_a) = project_vertices(vertices_a, axis);
    let (min_b, max_b) = project_vertices(vertices_b, axis);
    
    // Check for separation
    if max_a < min_b || max_b < min_a {
        return (0.0, true);
    }
    
    // Calculate penetration depth
    let penetration = (max_a - min_b).min(max_b - min_a);
    
    (penetration, false)
}

/// Project vertices onto an axis and return min and max projections
fn project_vertices(vertices: &[Vec2], axis: Vec2) -> (f32, f32) {
    debug_assert!(!vertices.is_empty(), "Cannot project empty vertices");
    
    let mut min = vertices[0].dot(axis);
    let mut max = min;
    
    for &v in vertices.iter().skip(1) {
        let proj = v.dot(axis);
        if proj < min {
            min = proj;
        }
        if proj > max {
            max = proj;
        }
    }
    
    (min, max)
}

/// Get edges from vertices
fn get_edges(vertices: &[Vec2]) -> Vec<Vec2> {
    let n = vertices.len();
    let mut edges = Vec::with_capacity(n);
    
    for i in 0..n {
        let v1 = vertices[i];
        let v2 = vertices[(i + 1) % n];
        edges.push(v2 - v1);
    }
    
    edges
}

/// Generate contact manifold for overlapping polygons
fn generate_polygon_polygon_manifold(
    polygon_a: &Polygon,
    transform_a: Transform,
    polygon_b: &Polygon,
    transform_b: Transform,
    vertices_a: &[Vec2],
    vertices_b: &[Vec2],
    edges_a: &[Vec2],
    edges_b: &[Vec2],
    min_axis: Vec2,
    min_penetration: f32,
    config: &NarrowPhaseConfig,
) -> Option<ContactManifold> {
    // Determine which polygon is the reference and which is the incident
    // Reference: polygon with edge most perpendicular to min_axis
    let (reference_poly, reference_vertices, reference_edges, reference_transform, reference_polygon) =
        if is_reference(polygon_a, transform_a, vertices_a, edges_a, min_axis) {
            (polygon_a, vertices_a, edges_a, transform_a, true)
        } else {
            (polygon_b, vertices_b, edges_b, transform_b, false)
        };
    
    let (incident_poly, incident_vertices, incident_transform, incident_polygon) = 
        if reference_polygon {
            (polygon_b, vertices_b, transform_b, false)
        } else {
            (polygon_a, vertices_a, transform_a, true)
        };
    
    // Find the reference edge (most perpendicular to min_axis)
    let reference_edge_index = find_reference_edge(reference_edges, min_axis);
    let reference_edge = reference_edges[reference_edge_index];
    let reference_normal = Vec2::new(-reference_edge.y, reference_edge.x).normalize();
    
    // Make sure the reference normal points toward the incident polygon
    let reference_normal = if reference_normal.dot(min_axis) < 0.0 {
        -reference_normal
    } else {
        reference_normal
    };
    
    // Get the reference edge vertices
    let ref_v1 = reference_vertices[reference_edge_index];
    let ref_v2 = reference_vertices[(reference_edge_index + 1) % reference_vertices.len()];
    
    // Clip the incident polygon against the reference face
    let mut contacts = Vec::new();
    
    // Transform incident vertices to reference space
    let incident_in_reference: Vec<Vec2> = incident_vertices
        .iter()
        .map(|&v| reference_transform.inverse_transform_point(v))
        .collect();
    
    // Clip against the reference edge
    let clipped = clip_polygon_to_edge(
        &incident_in_reference,
        reference_edge,
        reference_normal,
    );
    
    // For each clipped point, create a contact
    for (i, clipped_point) in clipped.iter().enumerate() {
        if i >= config.max_contact_points {
            break;
        }
        
        // Transform back to world space
        let contact_point_world = reference_transform.transform_point(*clipped_point);
        
        // Calculate penetration at this contact point
        let to_contact = contact_point_world - ref_v1;
        let distance_along_normal = to_contact.dot(reference_normal);
        let penetration = min_penetration - distance_along_normal;
        
        // Create contact
        let contact_normal = if reference_polygon {
            reference_normal
        } else {
            -reference_normal
        };
        
        let contact_id = if reference_polygon {
            ContactId::new(reference_edge_index as u32, i as u32)
        } else {
            ContactId::new(i as u32, reference_edge_index as u32)
        };
        
        contacts.push((contact_point_world, contact_normal, penetration, contact_id));
    }
    
    if contacts.is_empty() {
        return None;
    }
    
    // Create manifold
    let mut manifold = ContactManifold::new();
    
    for (point, normal, penetration, id) in contacts {
        manifold.add_contact(point, normal, penetration, 0.0, 0.0, id);
    }
    
    Some(manifold)
}

/// Check if a polygon should be the reference
fn is_reference(
    polygon: &Polygon,
    _transform: Transform,
    vertices: &[Vec2],
    edges: &[Vec2],
    min_axis: Vec2,
) -> bool {
    // Find the edge most perpendicular to the min_axis
    let reference_edge_index = find_reference_edge(edges, min_axis);
    let reference_edge = edges[reference_edge_index];
    let reference_normal = Vec2::new(-reference_edge.y, reference_edge.x).normalize();
    
    // Check if this normal is close to the min_axis
    reference_normal.dot(min_axis) > 0.0
}

/// Find the reference edge (most perpendicular to the axis)
fn find_reference_edge(edges: &[Vec2], axis: Vec2) -> usize {
    let mut max_dot = f32::NEG_INFINITY;
    let mut best_index = 0;
    
    for (i, &edge) in edges.iter().enumerate() {
        let normal = Vec2::new(-edge.y, edge.x).normalize();
        let dot = normal.dot(axis);
        
        if dot > max_dot {
            max_dot = dot;
            best_index = i;
        }
    }
    
    best_index
}

/// Clip a polygon against an edge (Sutherland-Hodgman algorithm)
fn clip_polygon_to_edge(
    polygon: &[Vec2],
    edge: Vec2,
    normal: Vec2,
) -> Vec<Vec2> {
    let mut output = Vec::new();
    
    if polygon.is_empty() {
        return output;
    }
    
    let n = polygon.len();
    
    for i in 0..n {
        let current = polygon[i];
        let prev = polygon[(i + n - 1) % n];
        
        let current_dot = current.dot(normal);
        let prev_dot = prev.dot(normal);
        
        // Current point is inside
        if current_dot <= 0.0 {
            // Previous point is outside, add intersection
            if prev_dot > 0.0 {
                let intersection = find_edge_intersection(prev, current, edge, normal);
                output.push(intersection);
            }
            output.push(current);
        } else {
            // Current point is outside, previous is inside, add intersection
            if prev_dot <= 0.0 {
                let intersection = find_edge_intersection(prev, current, edge, normal);
                output.push(intersection);
            }
        }
    }
    
    output
}

/// Find the intersection point of an edge with the reference edge
fn find_edge_intersection(
    v1: Vec2,
    v2: Vec2,
    reference_edge: Vec2,
    reference_normal: Vec2,
) -> Vec2 {
    let edge = v2 - v1;
    
    // The intersection occurs where the edge crosses the reference edge
    // This is where dot(normal, point) = 0
    
    let d1 = v1.dot(reference_normal);
    let d2 = v2.dot(reference_normal);
    
    if d1 == d2 {
        return v1; // Parallel, shouldn't happen
    }
    
    let t = -d1 / (d2 - d1);
    v1 + edge * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_separated_polygons() {
        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(5.0, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &polygon_a, transform_a, &polygon_b, transform_b, &config
        );

        assert!(result.is_none(), "Separated polygons should not collide");
    }

    #[test]
    fn test_overlapping_polygons() {
        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(1.5, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &polygon_a, transform_a, &polygon_b, transform_b, &config
        );

        assert!(result.is_some(), "Overlapping polygons should collide");
        let manifold = result.unwrap();
        assert!(manifold.contact_count() >= 1, "Should have at least one contact");
    }

    #[test]
    fn test_touching_polygons() {
        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        // Polygons touching at edge (x=2.0 for both)
        let transform_a = Transform::from_translation(Vec2::new(-1.0, 0.0));
        let transform_b = Transform::from_translation(Vec2::new(1.0, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &polygon_a, transform_a, &polygon_b, transform_b, &config
        );

        assert!(result.is_some(), "Touching polygons should collide");
    }

    #[test]
    fn test_rotated_polygons() {
        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::new(
            Vec2::new(1.5, 0.0),
            Rot2::new(crate::constants::PI_OVER_FOUR)
        );

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &polygon_a, transform_a, &polygon_b, transform_b, &config
        );

        assert!(result.is_some(), "Rotated overlapping polygons should collide");
    }

    #[test]
    fn test_identical_polygons() {
        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &polygon_a, transform_a, &polygon_b, transform_b, &config
        );

        assert!(result.is_some(), "Identical polygons should collide");
    }

    #[test]
    fn test_contained_polygon() {
        let polygon_a = Polygon::from_rectangle(10.0, 10.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &polygon_a, transform_a, &polygon_b, transform_b, &config
        );

        assert!(result.is_some(), "Contained polygon should collide");
    }

    #[test]
    fn test_triangle_collision() {
        let triangle_a = Polygon::new(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(0.0, 1.0),
        ]);
        let triangle_b = Polygon::new(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(0.0, 1.0),
        ]);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(0.5, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &triangle_a, transform_a, &triangle_b, transform_b, &config
        );

        assert!(result.is_some(), "Overlapping triangles should collide");
    }

    #[test]
    fn test_contact_normal_orientation() {
        let polygon_a = Polygon::from_rectangle(2.0, 2.0);
        let polygon_b = Polygon::from_rectangle(2.0, 2.0);

        // polygon_a at (-1,0), polygon_b at (1,0) - they touch at x=0
        let transform_a = Transform::from_translation(Vec2::new(-1.0, 0.0));
        let transform_b = Transform::from_translation(Vec2::new(1.0, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = polygon_polygon_collision(
            &polygon_a, transform_a, &polygon_b, transform_b, &config
        );

        assert!(result.is_some());
        let manifold = result.unwrap();
        let contact = manifold.contact(0);
        
        // Normal should point from A to B (rightward)
        assert!(contact.normal.x > 0.0, "Normal should point from A to B");
    }

    #[test]
    fn test_projection() {
        let vertices = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];

        let axis = Vec2::unit_x();
        let (min, max) = project_vertices(&vertices, axis);
        
        assert!(approx_eq(min, 0.0));
        assert!(approx_eq(max, 2.0));

        let axis = Vec2::unit_y();
        let (min, max) = project_vertices(&vertices, axis);
        
        assert!(approx_eq(min, 0.0));
        assert!(approx_eq(max, 2.0));
    }

    #[test]
    fn test_sat_separation() {
        let vertices_a = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let vertices_b = vec![
            Vec2::new(5.0, 0.0),
            Vec2::new(7.0, 0.0),
            Vec2::new(7.0, 2.0),
            Vec2::new(5.0, 2.0),
        ];

        let normals_a = vec![
            Vec2::new(0.0, -1.0),  // bottom
            Vec2::new(1.0, 0.0),   // right
            Vec2::new(0.0, 1.0),   // top
            Vec2::new(-1.0, 0.0),  // left
        ];
        let normals_b = normals_a.clone();

        let result = find_separating_axis(&vertices_a, &vertices_b, &normals_a, &normals_b);
        
        match result {
            SATResult::Separated => {},
            _ => panic!("Polygons should be separated"),
        }
    }

    #[test]
    fn test_sat_overlap() {
        let vertices_a = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let vertices_b = vec![
            Vec2::new(1.5, 0.0),
            Vec2::new(3.5, 0.0),
            Vec2::new(3.5, 2.0),
            Vec2::new(1.5, 2.0),
        ];

        let normals_a = vec![
            Vec2::new(0.0, -1.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, 1.0),
            Vec2::new(-1.0, 0.0),
        ];
        let normals_b = normals_a.clone();

        let result = find_separating_axis(&vertices_a, &vertices_b, &normals_a, &normals_b);
        
        match result {
            SATResult::Overlapping { .. } => {},
            _ => panic!("Polygons should overlap"),
        }
    }
}
