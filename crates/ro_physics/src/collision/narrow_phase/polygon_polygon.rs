//! Polygon-polygon collision detection using the Separating Axis Theorem (SAT)
//! with reference-face clipping.
//!
//! The algorithm follows the standard approach used by mature 2D engines:
//!
//! 1. transform both polygons into world space,
//! 2. test every face normal of both polygons as a separating axis and keep the
//!    axis with the largest separation (the "minimum penetration" axis),
//! 3. pick the polygon owning that axis as the *reference* polygon and the
//!    other one as the *incident* polygon,
//! 4. find the incident face whose normal is most anti-parallel to the
//!    reference normal,
//! 5. clip the incident face against the side planes of the reference face and
//!    keep the points that lie behind the reference plane.
//!
//! The resulting manifold is expressed for the pair (A, B): the normal always
//! points from A to B.

use super::super::{ContactId, ContactManifold};
use super::NarrowPhaseConfig;
use crate::math::{Transform, Vec2};
use crate::shapes::Polygon;

/// Relative tolerance used when deciding which polygon provides the reference
/// face. It biases the choice towards polygon A so that the reference face does
/// not flip-flop between frames for near-symmetric configurations.
const RELATIVE_TOLERANCE: f32 = 0.98;

/// Absolute tolerance (in metres) of the same test.
const ABSOLUTE_TOLERANCE: f32 = 0.001;

/// Detect collision between two polygons.
///
/// Returns a contact manifold if they overlap, `None` otherwise.
#[inline]
#[must_use]
pub fn polygon_polygon_collision(
    polygon_a: &Polygon,
    transform_a: Transform,
    polygon_b: &Polygon,
    transform_b: Transform,
    config: &NarrowPhaseConfig,
) -> Option<ContactManifold> {
    if polygon_a.vertex_count() < 3 || polygon_b.vertex_count() < 3 {
        return None;
    }

    // Vertices and outward normals in world space
    let vertices_a = transform_vertices(polygon_a, transform_a);
    let vertices_b = transform_vertices(polygon_b, transform_b);
    let normals_a = rotate_normals(polygon_a, transform_a);
    let normals_b = rotate_normals(polygon_b, transform_b);

    // Find the maximum separation along the faces of each polygon. A positive
    // separation means there is a separating axis, i.e. no collision.
    let (separation_a, face_a) = max_separation(&vertices_a, &normals_a, &vertices_b);
    if separation_a > 0.0 {
        return None;
    }

    let (separation_b, face_b) = max_separation(&vertices_b, &normals_b, &vertices_a);
    if separation_b > 0.0 {
        return None;
    }

    // Choose the reference polygon: the one whose best face is "most
    // separating", with a bias towards A for stability.
    let (
        flipped,
        reference_vertices,
        reference_normals,
        incident_vertices,
        incident_normals,
        reference_face,
    ) = if separation_b > RELATIVE_TOLERANCE * separation_a + ABSOLUTE_TOLERANCE {
        (
            true,
            &vertices_b,
            &normals_b,
            &vertices_a,
            &normals_a,
            face_b,
        )
    } else {
        (
            false,
            &vertices_a,
            &normals_a,
            &vertices_b,
            &normals_b,
            face_a,
        )
    };

    let reference_count = reference_vertices.len();
    let v1 = reference_vertices[reference_face];
    let v2 = reference_vertices[(reference_face + 1) % reference_count];
    let reference_normal = reference_normals[reference_face];

    // Incident face: the one most anti-parallel to the reference normal
    let incident_face = find_incident_face(incident_normals, reference_normal);
    let incident_count = incident_vertices.len();
    let i1 = incident_vertices[incident_face];
    let i2 = incident_vertices[(incident_face + 1) % incident_count];

    // Clip the incident face against the side planes of the reference face
    let tangent = (v2 - v1).normalize();
    let (p0, p1) = clip_segment(i1, i2, -tangent, -tangent.dot(v1))?;
    let (p0, p1) = clip_segment(p0, p1, tangent, tangent.dot(v2))?;

    // Keep the points that lie behind the reference face
    let mut contacts: Vec<(Vec2, f32)> = Vec::with_capacity(2);

    for point in [p0, p1] {
        let separation = reference_normal.dot(point - v1);
        // Points within the linear slop are kept as touching contacts so that
        // resting bodies do not lose their manifold between frames.
        if separation <= crate::constants::LINEAR_SLOP {
            contacts.push((point, (-separation).max(0.0)));
        }
    }

    if contacts.is_empty() {
        return None;
    }

    // Manifold normal always points from A to B
    let normal = if flipped {
        -reference_normal
    } else {
        reference_normal
    };

    let mut manifold = ContactManifold::new();
    manifold.set_normal(normal);

    for (index, (point, penetration)) in contacts.iter().enumerate() {
        if index >= config.max_contact_points.max(1) {
            break;
        }

        let contact_id = if flipped {
            ContactId::flipped(index as u32, reference_face as u32)
        } else {
            ContactId::new(reference_face as u32, index as u32)
        };

        manifold.add_contact(*point, normal, *penetration, 0.0, 0.0, contact_id);
    }

    if manifold.contact_count() == 0 {
        return None;
    }

    Some(manifold)
}

/// Transform a polygon's vertices into world space.
fn transform_vertices(polygon: &Polygon, transform: Transform) -> Vec<Vec2> {
    polygon
        .vertices()
        .iter()
        .map(|&vertex| transform.transform_point(vertex))
        .collect()
}

/// Rotate a polygon's outward normals into world space.
fn rotate_normals(polygon: &Polygon, transform: Transform) -> Vec<Vec2> {
    polygon
        .normals()
        .iter()
        .map(|&normal| transform.transform_vector(normal))
        .collect()
}

/// Find the face of `reference` with the largest separation from `incident`.
///
/// Returns `(separation, face_index)`; a positive separation means the polygons
/// are separated along that face normal.
fn max_separation(
    reference_vertices: &[Vec2],
    reference_normals: &[Vec2],
    incident_vertices: &[Vec2],
) -> (f32, usize) {
    let mut best_separation = f32::NEG_INFINITY;
    let mut best_face = 0usize;

    for (index, &normal) in reference_normals.iter().enumerate() {
        let point = reference_vertices[index];

        let mut separation = f32::INFINITY;
        for &vertex in incident_vertices {
            separation = separation.min(normal.dot(vertex - point));
        }

        if separation > best_separation {
            best_separation = separation;
            best_face = index;
        }
    }

    (best_separation, best_face)
}

/// Find the face of the incident polygon that is most anti-parallel to
/// `reference_normal`.
fn find_incident_face(incident_normals: &[Vec2], reference_normal: Vec2) -> usize {
    let mut min_dot = f32::INFINITY;
    let mut incident_face = 0usize;

    for (index, &normal) in incident_normals.iter().enumerate() {
        let dot = normal.dot(reference_normal);
        if dot < min_dot {
            min_dot = dot;
            incident_face = index;
        }
    }

    incident_face
}

/// Clip the segment `v0 -> v1` against the half-space
/// `dot(normal, p) - offset <= 0`.
///
/// Returns the clipped segment, or `None` when the segment lies entirely
/// outside the half-space.
fn clip_segment(v0: Vec2, v1: Vec2, normal: Vec2, offset: f32) -> Option<(Vec2, Vec2)> {
    let distance0 = normal.dot(v0) - offset;
    let distance1 = normal.dot(v1) - offset;

    // Both endpoints inside
    if distance0 <= 0.0 && distance1 <= 0.0 {
        return Some((v0, v1));
    }

    // v0 inside, v1 outside
    if distance0 <= 0.0 && distance1 > 0.0 {
        let t = distance0 / (distance0 - distance1);
        return Some((v0, v0 + (v1 - v0) * t));
    }

    // v1 inside, v0 outside
    if distance1 <= 0.0 && distance0 > 0.0 {
        let t = distance1 / (distance1 - distance0);
        return Some((v1, v1 + (v0 - v1) * t));
    }

    // Both endpoints outside
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Rot2;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= 1e-4
    }

    /// A 2x2 box (half extents 1) centred on the origin.
    fn box_polygon() -> Polygon {
        Polygon::from_rectangle(2.0, 2.0)
    }

    #[test]
    fn test_box_resting_on_box() {
        let lower = box_polygon();
        let upper = box_polygon();
        let config = NarrowPhaseConfig::default();

        // The upper box overlaps the lower one by 0.25
        let manifold = polygon_polygon_collision(
            &upper,
            Transform::from_translation(Vec2::new(0.0, 1.75)),
            &lower,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        // Normal points from A (upper) to B (lower): downwards
        assert!(approx_eq(manifold.normal().x, 0.0));
        assert!(approx_eq(manifold.normal().y, -1.0));

        assert_eq!(manifold.contact_count(), 2);

        for index in 0..manifold.contact_count() {
            let contact = manifold.contact(index);
            assert!(approx_eq(contact.penetration(), 0.25));
            // Contact points lie on the top face of the lower box
            assert!(approx_eq(contact.position().y, 1.0));
            assert!(contact.position().x <= 1.0 + 1e-4);
            assert!(contact.position().x >= -1.0 - 1e-4);
        }

        // The two contact points are distinct
        let dx = (manifold.contact(0).position() - manifold.contact(1).position()).length();
        assert!(dx > 1.0, "contacts should span the face, got {dx}");
    }

    #[test]
    fn test_separated_boxes() {
        let a = box_polygon();
        let b = box_polygon();
        let config = NarrowPhaseConfig::default();

        let manifold = polygon_polygon_collision(
            &a,
            Transform::from_translation(Vec2::zero()),
            &b,
            Transform::from_translation(Vec2::new(0.0, 2.5)),
            &config,
        );

        assert!(manifold.is_none());
    }

    #[test]
    fn test_touching_boxes() {
        let a = box_polygon();
        let b = box_polygon();
        let config = NarrowPhaseConfig::default();

        let manifold = polygon_polygon_collision(
            &a,
            Transform::from_translation(Vec2::zero()),
            &b,
            Transform::from_translation(Vec2::new(0.0, 2.0)),
            &config,
        )
        .expect("touching boxes touch");

        for index in 0..manifold.contact_count() {
            assert!(approx_eq(manifold.contact(index).penetration(), 0.0));
        }
    }

    #[test]
    fn test_rotated_box_on_box() {
        let lower = box_polygon();
        let upper = box_polygon();
        let config = NarrowPhaseConfig::default();

        let manifold = polygon_polygon_collision(
            &upper,
            Transform::new(
                Vec2::new(0.0, 1.75),
                Rot2::new(crate::constants::PI_OVER_FOUR),
            ),
            &lower,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        assert!(manifold.contact_count() >= 1);
        // Every contact must be at or below the top face of the lower box
        for index in 0..manifold.contact_count() {
            assert!(manifold.contact(index).penetration() >= -1e-3);
            assert!(manifold.contact(index).position().y <= 1.0 + 1e-3);
        }
    }

    #[test]
    fn test_normal_direction_is_a_to_b() {
        let a = box_polygon();
        let b = box_polygon();
        let config = NarrowPhaseConfig::default();

        // A on the left, overlapping B which sits to its right
        let manifold = polygon_polygon_collision(
            &a,
            Transform::from_translation(Vec2::zero()),
            &b,
            Transform::from_translation(Vec2::new(1.75, 0.0)),
            &config,
        )
        .expect("expected a contact");

        assert!(approx_eq(manifold.normal().x, 1.0));
        assert!(approx_eq(manifold.normal().y, 0.0));

        // Swapping the polygons must flip the normal
        let swapped = polygon_polygon_collision(
            &b,
            Transform::from_translation(Vec2::new(1.75, 0.0)),
            &a,
            Transform::from_translation(Vec2::zero()),
            &config,
        )
        .expect("expected a contact");

        assert!(approx_eq(swapped.normal().x, -1.0));
    }

    #[test]
    fn test_deep_overlap_penetration() {
        let a = box_polygon();
        let b = box_polygon();
        let config = NarrowPhaseConfig::default();

        // Half-overlapped vertically: penetration is 1.0
        let manifold = polygon_polygon_collision(
            &a,
            Transform::from_translation(Vec2::new(0.0, 1.0)),
            &b,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        for index in 0..manifold.contact_count() {
            assert!(approx_eq(manifold.contact(index).penetration(), 1.0));
        }
    }

    #[test]
    fn test_triangle_on_box() {
        let triangle = Polygon::new(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(0.0, 1.0),
        ]);
        let ground = box_polygon();
        let config = NarrowPhaseConfig::default();

        let manifold = polygon_polygon_collision(
            &triangle,
            Transform::from_translation(Vec2::new(0.0, 1.75)),
            &ground,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        assert!(manifold.contact_count() >= 1);
        // The triangle rests on the ground, so the normal points down
        assert!(manifold.normal().y < 0.0);
    }
}
