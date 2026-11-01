//! Circle-polygon collision detection.
//!
//! The algorithm works in the polygon's local space:
//!
//! 1. transform the circle centre into polygon local space,
//! 2. handle the "centre inside the polygon" case separately, because there
//!    the closest feature is a face and the centre has to be pushed *out*,
//! 3. otherwise use the closest feature (vertex or face) of the polygon.
//!
//! The generated manifold is expressed for the pair (circle = A,
//! polygon = B), so the normal points **from the circle towards the polygon**
//! and the impulse pushes the circle along `-normal`, i.e. out of the polygon.

use crate::constants::EPSILON;
use crate::math::{Transform, Vec2};
use crate::shapes::{Circle, Polygon};
use super::NarrowPhaseConfig;
use super::super::{ContactId, ContactManifold};

/// Detect collision between a circle (A) and a polygon (B).
///
/// Returns a contact manifold if they overlap, `None` otherwise.
#[inline]
#[must_use]
pub fn circle_polygon_collision(
    circle: &Circle,
    transform_circle: Transform,
    polygon: &Polygon,
    transform_polygon: Transform,
    _config: &NarrowPhaseConfig,
) -> Option<ContactManifold> {
    if polygon.vertex_count() < 3 {
        return None;
    }

    let radius = circle.radius();

    // Transform the circle centre into polygon local space
    let center_local = transform_polygon.inverse_transform_point(transform_circle.position);

    let vertices = polygon.vertices();
    let normals = polygon.normals();
    let count = vertices.len();

    let (normal_local, penetration, contact_local, feature) =
        if polygon.contains_point(center_local) {
            inside_contact(center_local, radius, vertices, normals)?
        } else {
            outside_contact(center_local, radius, vertices, normals, count)?
        };

    // Transform back into world space. The normal is a direction, so only the
    // rotation of the polygon applies.
    let normal_world = transform_polygon.transform_vector(normal_local);
    let contact_world = transform_polygon.transform_point(contact_local);

    let mut manifold = ContactManifold::new();
    manifold.add_contact(
        contact_world,
        normal_world,
        penetration,
        0.0,
        0.0,
        ContactId::new(0, feature),
    );

    Some(manifold)
}

/// Contact data for a circle centre that lies inside the polygon.
///
/// Returns `(normal_local, penetration, contact_local, feature)`.
fn inside_contact(
    center_local: Vec2,
    radius: f32,
    vertices: &[Vec2],
    normals: &[Vec2],
) -> Option<(Vec2, f32, Vec2, u32)> {
    let mut best_index = 0usize;
    let mut best_separation = f32::NEG_INFINITY;

    for (index, normal) in normals.iter().enumerate() {
        // Signed distance of the centre from the face plane; negative inside.
        let separation = normal.dot(center_local - vertices[index]);
        if separation > best_separation {
            best_separation = separation;
            best_index = index;
        }
    }

    // The centre is inside, so `best_separation` is negative: the magnitude is
    // the depth of the centre below the nearest face. The circle overlaps the
    // polygon by `radius + depth`.
    let depth = -best_separation;
    let penetration = radius + depth;

    if !(penetration > 0.0) {
        return None;
    }

    let outward = normals[best_index];

    // Contact point: the centre projected onto the nearest face.
    let contact_local = center_local - outward * best_separation;

    // The circle leaves the polygon along the outward normal, so the manifold
    // normal (A -> B, i.e. circle -> polygon) points the other way.
    let normal_local = -outward;

    Some((normal_local, penetration, contact_local, best_index as u32))
}

/// Contact data for a circle centre that lies outside the polygon.
fn outside_contact(
    center_local: Vec2,
    radius: f32,
    vertices: &[Vec2],
    normals: &[Vec2],
    count: usize,
) -> Option<(Vec2, f32, Vec2, u32)> {
    let (closest_local, feature) = closest_feature(center_local, vertices, count);

    let to_center = center_local - closest_local;
    let distance = to_center.length();

    if distance > radius {
        return None;
    }

    let outward = if distance > EPSILON {
        to_center / distance
    } else {
        // The centre sits exactly on the boundary: fall back to the normal of
        // the closest face.
        let fallback = feature.min(count as u32 - 1) as usize;
        normals[fallback]
    };

    // Normal points from the polygon towards the circle (B -> A), the manifold
    // normal points from A to B.
    let normal_local = -outward;

    Some((normal_local, radius - distance, closest_local, feature))
}

/// Find the closest point on the polygon boundary and its feature index.
///
/// Feature indices below `count` refer to vertices, indices `count..2*count`
/// refer to faces.
fn closest_feature(point: Vec2, vertices: &[Vec2], count: usize) -> (Vec2, u32) {
    let mut closest = vertices[0];
    let mut min_distance_sq = (point - vertices[0]).length_squared();
    let mut feature = 0u32;

    for (index, &vertex) in vertices.iter().enumerate() {
        let distance_sq = (point - vertex).length_squared();
        if distance_sq < min_distance_sq {
            min_distance_sq = distance_sq;
            closest = vertex;
            feature = index as u32;
        }
    }

    for index in 0..count {
        let v1 = vertices[index];
        let v2 = vertices[(index + 1) % count];

        let edge = v2 - v1;
        let edge_length_sq = edge.length_squared();

        let t = if edge_length_sq > EPSILON * EPSILON {
            ((point - v1).dot(edge) / edge_length_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let projection = v1 + edge * t;
        let distance_sq = (point - projection).length_squared();

        if distance_sq < min_distance_sq {
            min_distance_sq = distance_sq;
            closest = projection;
            feature = (count + index) as u32;
        }
    }

    (closest, feature)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Rot2;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= 1e-5
    }

    /// A 2x2 box centred on the origin.
    fn box_polygon() -> Polygon {
        Polygon::from_rectangle(2.0, 2.0)
    }

    #[test]
    fn test_circle_resting_on_top_of_box() {
        let circle = Circle::new(0.5);
        let polygon = box_polygon();
        let config = NarrowPhaseConfig::default();

        // Circle centre 0.25 above the top face (y = 1)
        let manifold = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(0.0, 1.25)),
            &polygon,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        let contact = manifold.contact(0);
        // The normal points from the circle down into the box
        assert!(approx_eq(contact.normal().x, 0.0));
        assert!(approx_eq(contact.normal().y, -1.0));
        assert!(approx_eq(contact.penetration(), 0.25));
        // Contact point sits on the top face
        assert!(approx_eq(contact.position().y, 1.0));
    }

    #[test]
    fn test_circle_below_box() {
        let circle = Circle::new(0.5);
        let polygon = box_polygon();
        let config = NarrowPhaseConfig::default();

        let manifold = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(0.0, -1.25)),
            &polygon,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        let contact = manifold.contact(0);
        // Normal points from the circle up into the box
        assert!(approx_eq(contact.normal().y, 1.0));
        assert!(approx_eq(contact.penetration(), 0.25));
    }

    #[test]
    fn test_circle_near_corner() {
        let circle = Circle::new(0.5);
        let polygon = box_polygon();
        let config = NarrowPhaseConfig::default();

        let manifold = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(1.25, 1.25)),
            &polygon,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        let contact = manifold.contact(0);
        // The closest feature is the corner (1, 1)
        assert!(approx_eq(contact.position().x, 1.0));
        assert!(approx_eq(contact.position().y, 1.0));
        // Normal points from the circle towards the corner
        let expected = Vec2::new(-1.0, -1.0).normalize();
        assert!(approx_eq(contact.normal().x, expected.x));
        assert!(approx_eq(contact.normal().y, expected.y));
    }

    #[test]
    fn test_circle_separated() {
        let circle = Circle::new(0.5);
        let polygon = box_polygon();
        let config = NarrowPhaseConfig::default();

        let manifold = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(0.0, 3.0)),
            &polygon,
            Transform::identity(),
            &config,
        );

        assert!(manifold.is_none());
    }

    #[test]
    fn test_circle_center_inside_polygon() {
        let circle = Circle::new(0.5);
        let polygon = box_polygon();
        let config = NarrowPhaseConfig::default();

        // Centre 0.75 below the top face, so the circle pokes 0.25 out of it
        let manifold = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(0.0, 0.25)),
            &polygon,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        let contact = manifold.contact(0);
        // The circle must be pushed out through the nearest face (the top one)
        assert!(approx_eq(contact.normal().y, -1.0), "normal was {}", contact.normal().y);
        // Overlap = radius + depth = 0.5 + 0.75
        assert!(approx_eq(contact.penetration(), 1.25));
        // Contact point lies on the top face
        assert!(approx_eq(contact.position().y, 1.0));
    }

    #[test]
    fn test_circle_deep_inside_polygon_escapes_nearest_face() {
        let circle = Circle::new(0.25);
        let polygon = Polygon::from_rectangle(20.0, 20.0);
        let config = NarrowPhaseConfig::default();

        // Centre near the right-hand face
        let manifold = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(9.9, 0.0)),
            &polygon,
            Transform::identity(),
            &config,
        )
        .expect("expected a contact");

        let contact = manifold.contact(0);
        assert!(approx_eq(contact.normal().x, -1.0));
        // Overlap = radius + depth below the face = 0.25 + 0.1
        assert!(approx_eq(contact.penetration(), 0.35));
    }

    #[test]
    fn test_rotated_polygon() {
        let circle = Circle::new(0.5);
        let polygon = box_polygon();
        let config = NarrowPhaseConfig::default();

        // Rotate the box by 90 degrees: the contact normal rotates with it.
        let manifold = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(1.25, 0.0)),
            &polygon,
            Transform::from_rotation(Rot2::new(crate::constants::PI_OVER_TWO)),
            &config,
        )
        .expect("expected a contact");

        let contact = manifold.contact(0);
        assert!(approx_eq(contact.normal().x, -1.0));
        assert!(approx_eq(contact.normal().y, 0.0));
        assert!(approx_eq(contact.penetration(), 0.25));
    }

    #[test]
    fn test_contact_id_stays_stable() {
        let circle = Circle::new(0.5);
        let polygon = box_polygon();
        let config = NarrowPhaseConfig::default();

        let first = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(0.0, 1.25)),
            &polygon,
            Transform::identity(),
            &config,
        )
        .unwrap();
        let second = circle_polygon_collision(
            &circle,
            Transform::from_translation(Vec2::new(0.1, 1.2)),
            &polygon,
            Transform::identity(),
            &config,
        )
        .unwrap();

        // Both contacts are generated by the same face of the polygon
        assert_eq!(
            first.contact(0).contact_id(),
            second.contact(0).contact_id()
        );
    }
}
