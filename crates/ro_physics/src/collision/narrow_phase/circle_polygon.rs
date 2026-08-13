//! Circle-polygon collision detection using SAT and closest point methods.
//!
//! This module provides robust collision detection between circles and convex polygons.
//! The algorithm finds the closest point on the polygon to the circle center and
//! checks for overlap.

use crate::constants::EPSILON;
use crate::math::{Rot2, Transform, Vec2};
use crate::shapes::{Circle, Polygon};
use super::NarrowPhaseConfig;
use super::super::{ContactId, ContactManifold};

/// Detect collision between a circle and a polygon.
///
/// Uses a hybrid approach:
/// 1. Transform circle center to polygon local space
/// 2. Find closest point on polygon to circle center
/// 3. Check if distance < circle radius
/// 4. Generate contact manifold with proper normal and penetration
///
/// Returns a contact manifold if they overlap, None otherwise.
#[inline]
#[must_use]
pub fn circle_polygon_collision(
    circle: &Circle,
    transform_circle: Transform,
    polygon: &Polygon,
    transform_polygon: Transform,
    config: &NarrowPhaseConfig,
) -> Option<ContactManifold> {
    // Transform circle center to polygon local space
    let circle_center_local = transform_polygon
        .inverse_transform_point(transform_circle.position);
    
    // Find the closest point on the polygon to the circle center
    let (closest_point_local, closest_feature) = 
        find_closest_point_on_polygon(polygon, circle_center_local);
    
    // Calculate vector from closest point to circle center
    let to_circle = circle_center_local - closest_point_local;
    let distance_squared = to_circle.length_squared();
    
    // Check if circle center is inside the polygon
    let circle_inside = polygon.contains_point(circle_center_local);
    
    // If circle center is inside polygon, we need special handling
    if circle_inside {
        // Find the closest edge and use its normal
        let (edge_normal, penetration) = find_closest_edge_penetration(
            polygon, circle_center_local, circle.radius
        );
        
        // Transform back to world space
        let edge_normal_world = transform_polygon.rotation.rotate(edge_normal);
        let closest_point_world = transform_polygon.transform_point(closest_point_local);
        let contact_point_world = closest_point_world + edge_normal_world * (circle.radius - penetration);
        
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            contact_point_world,
            edge_normal_world,
            penetration,
            0.0,
            0.0,
            ContactId::new(closest_feature, 0),
        );
        
        return Some(manifold);
    }
    
    // Check for separation
    let distance = distance_squared.sqrt();
    if distance > circle.radius {
        return None;
    }
    
    // Calculate penetration depth
    let penetration = circle.radius - distance;
    
    // Handle near-zero distance (circle center very close to polygon)
    let normal_local = if distance > EPSILON {
        to_circle / distance
    } else {
        // Use the closest edge normal as fallback
        find_normal_from_polygon(polygon, closest_point_local)
    };
    
    // Transform normal to world space
    let normal_world = transform_polygon.rotation.rotate(normal_local);
    
    // Calculate contact point in world space
    // Contact point is on the polygon surface
    let closest_point_world = transform_polygon.transform_point(closest_point_local);
    let contact_point_world = closest_point_world + normal_world * (penetration - circle.radius);
    
    // Create contact manifold
    let mut manifold = ContactManifold::new();
    manifold.add_contact(
        contact_point_world,
        normal_world,
        penetration,
        0.0,
        0.0,
        ContactId::new(closest_feature, 0),
    );
    
    // Check if we need a second contact point (for shallow penetration)
    if config.precise && penetration < config.collision_margin {
        // Try to find a second contact point
        if let Some(second_contact) = find_second_contact(
            circle,
            transform_circle,
            polygon,
            transform_polygon,
            &manifold,
            config,
        ) {
            manifold.add_contact(
                second_contact.position,
                second_contact.normal,
                second_contact.penetration,
                0.0,
                0.0,
                ContactId::new(second_contact.contact_id.0, second_contact.contact_id.1),
            );
        }
    }
    
    Some(manifold)
}

/// Find the closest point on a polygon to a given point.
///
/// Returns the closest point and the feature index (vertex or edge).
fn find_closest_point_on_polygon(polygon: &Polygon, point: Vec2) -> (Vec2, u32) {
    let vertices = polygon.vertices();
    let n = vertices.len();
    
    let mut closest_point = vertices[0];
    let mut min_dist_sq = f32::INFINITY;
    let mut closest_feature = 0u32;
    
    // Check all vertices
    for (i, &vertex) in vertices.iter().enumerate() {
        let delta = point - vertex;
        let dist_sq = delta.length_squared();
        
        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_point = vertex;
            closest_feature = i as u32;
        }
    }
    
    // Check all edges
    for i in 0..n {
        let v1 = vertices[i];
        let v2 = vertices[(i + 1) % n];
        
        let edge = v2 - v1;
        let to_point = point - v1;
        
        // Project point onto edge
        let t = to_point.dot(edge) / edge.length_squared();
        let t_clamped = t.clamp(0.0, 1.0);
        
        let projection = v1 + edge * t_clamped;
        let delta = point - projection;
        let dist_sq = delta.length_squared();
        
        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_point = projection;
            // Feature is the edge index (offset by vertex count)
            closest_feature = (n + i) as u32;
        }
    }
    
    (closest_point, closest_feature)
}

/// Find the closest edge and penetration when circle is inside polygon.
fn find_closest_edge_penetration(
    polygon: &Polygon,
    point: Vec2,
    radius: f32,
) -> (Vec2, f32) {
    let vertices = polygon.vertices();
    let normals = polygon.normals();
    let n = vertices.len();
    
    let mut min_penetration = f32::INFINITY;
    let mut best_normal = Vec2::zero();
    
    // For each edge, calculate how far we need to move to escape
    for i in 0..n {
        let v1 = vertices[i];
        let normal = normals[i];
        
        // Distance from point to edge (signed)
        let to_v1 = point - v1;
        let distance = to_v1.dot(normal);
        
        // Penetration = radius + distance (since point is inside, distance is negative)
        let penetration = radius + distance;
        
        if penetration < min_penetration {
            min_penetration = penetration;
            best_normal = normal;
        }
    }
    
    (best_normal, min_penetration)
}

/// Find a normal from the polygon for a point very close to the surface.
fn find_normal_from_polygon(polygon: &Polygon, point: Vec2) -> Vec2 {
    let vertices = polygon.vertices();
    let normals = polygon.normals();
    let n = vertices.len();
    
    // Find the closest edge
    let mut closest_edge = 0;
    let mut min_dist = f32::INFINITY;
    
    for i in 0..n {
        let v1 = vertices[i];
        let v2 = vertices[(i + 1) % n];
        
        let dist = distance_point_to_edge(point, v1, v2);
        if dist < min_dist {
            min_dist = dist;
            closest_edge = i;
        }
    }
    
    normals[closest_edge]
}

/// Calculate distance from point to edge.
fn distance_point_to_edge(point: Vec2, v1: Vec2, v2: Vec2) -> f32 {
    let edge = v2 - v1;
    let to_point = point - v1;
    
    let t = to_point.dot(edge) / edge.length_squared();
    let t_clamped = t.clamp(0.0, 1.0);
    
    let projection = v1 + edge * t_clamped;
    (point - projection).length()
}

/// Attempt to find a second contact point for shallow penetration.
///
/// This improves stability for near-flat contacts.
fn find_second_contact(
    circle: &Circle,
    transform_circle: Transform,
    polygon: &Polygon,
    transform_polygon: Transform,
    existing_manifold: &ContactManifold,
    config: &NarrowPhaseConfig,
) -> Option<Contact> {
    use super::super::Contact;
    
    // Transform circle to polygon local space
    let circle_center_local = transform_polygon
        .inverse_transform_point(transform_circle.position);
    
    let vertices = polygon.vertices();
    let normals = polygon.normals();
    let n = vertices.len();
    
    let mut best_contact: Option<Contact> = None;
    let mut best_separation = f32::INFINITY;
    
    // Check all edges for potential second contact
    for i in 0..n {
        let v1 = vertices[i];
        let v2 = vertices[(i + 1) % n];
        let edge = v2 - v1;
        let normal = normals[i];
        
        // Project circle center onto edge
        let to_v1 = circle_center_local - v1;
        let t = to_v1.dot(edge) / edge.length_squared();
        
        // Only consider points on the edge segment
        if t < 0.0 || t > 1.0 {
            continue;
        }
        
        let projection = v1 + edge * t;
        let to_projection = circle_center_local - projection;
        let distance = to_projection.length();
        
        // Check if this could be a valid second contact
        if distance < circle.radius {
            let penetration = circle.radius - distance;
            
            // Check if this contact is significantly different from the first
            if let Some(existing) = existing_manifold.contacts().first() {
                let existing_local = transform_polygon
                    .inverse_transform_point(existing.position);
                let delta = projection - existing_local;
                
                // Only add if contacts are separated
                if delta.length_squared() > config.contact_tolerance * config.contact_tolerance {
                    let separation = -penetration; // More negative = better candidate
                    
                    if separation < best_separation {
                        best_separation = separation;
                        
                        let normal_world = transform_polygon.rotation.rotate(normal);
                        let projection_world = transform_polygon.transform_point(projection);
                        let contact_point_world = projection_world + normal_world * (penetration - circle.radius);
                        
                        best_contact = Some(Contact::new(
                            contact_point_world,
                            normal_world,
                            penetration,
                            0.0,
                            0.0,
                            ContactId::new(i as u32, 1),
                        ));
                    }
                }
            }
        }
    }
    
    best_contact
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_circle_outside_polygon() {
        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        let transform_circle = Transform::from_translation(Vec2::new(5.0, 0.0));
        let transform_polygon = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &polygon, transform_polygon, &config
        );

        assert!(result.is_none(), "Circle outside polygon should not collide");
    }

    #[test]
    fn test_circle_touching_polygon() {
        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        // Place circle touching the polygon edge (at x=2.0, which is the right edge)
        let transform_circle = Transform::from_translation(Vec2::new(2.0, 0.0));
        let transform_polygon = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &polygon, transform_polygon, &config
        );

        assert!(result.is_some(), "Touching circle should collide");
        let manifold = result.unwrap();
        assert!(manifold.contact_count() >= 1);
        
        let contact = manifold.contact(0);
        assert!(approx_eq(contact.penetration, 0.0, 1e-5), 
                "Penetration should be ~0 at touching point");
    }

    #[test]
    fn test_circle_overlapping_polygon() {
        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        // Place circle overlapping the polygon
        let transform_circle = Transform::from_translation(Vec2::new(1.5, 0.0));
        let transform_polygon = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &polygon, transform_polygon, &config
        );

        assert!(result.is_some(), "Overlapping circle should collide");
        let manifold = result.unwrap();
        assert!(manifold.contact_count() >= 1);
        
        let contact = manifold.contact(0);
        assert!(contact.penetration > 0.0, "Penetration should be positive");
    }

    #[test]
    fn test_circle_inside_polygon() {
        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(10.0, 10.0);

        // Place circle inside the polygon
        let transform_circle = Transform::identity();
        let transform_polygon = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &polygon, transform_polygon, &config
        );

        assert!(result.is_some(), "Circle inside polygon should collide");
        let manifold = result.unwrap();
        assert!(manifold.contact_count() >= 1);
    }

    #[test]
    fn test_circle_vs_rotated_box() {
        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        // Rotate the box by 45 degrees
        let transform_circle = Transform::from_translation(Vec2::new(2.0, 0.0));
        let transform_polygon = Transform::new(
            Vec2::new(0.0, 0.0),
            Rot2::new(crate::constants::PI_OVER_FOUR)
        );

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &polygon, transform_polygon, &config
        );

        assert!(result.is_some(), "Circle should collide with rotated box");
    }

    #[test]
    fn test_circle_vs_triangle() {
        let circle = Circle::new(1.0);
        let triangle = Polygon::new(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(0.0, 1.0),
        ]);

        let transform_circle = Transform::from_translation(Vec2::new(0.0, 0.5));
        let transform_polygon = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &triangle, transform_polygon, &config
        );

        assert!(result.is_some(), "Circle should collide with triangle");
    }

    #[test]
    fn test_contact_normal_direction() {
        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        // Circle to the right of polygon, normal should point left (toward circle)
        let transform_circle = Transform::from_translation(Vec2::new(2.5, 0.0));
        let transform_polygon = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &polygon, transform_polygon, &config
        );

        assert!(result.is_some());
        let manifold = result.unwrap();
        let contact = manifold.contact(0);
        
        // Normal should point from polygon to circle (rightward)
        assert!(contact.normal.x > 0.0, "Normal should point toward circle");
    }

    #[test]
    fn test_penetration_depth() {
        let circle = Circle::new(1.0);
        let polygon = Polygon::from_rectangle(2.0, 2.0);

        // Circle center at (1.5, 0), polygon from (-1,-1) to (1,1)
        // Distance from circle center to polygon edge at x=1 is 0.5
        // Penetration should be 1.0 - 0.5 = 0.5
        let transform_circle = Transform::from_translation(Vec2::new(1.5, 0.0));
        let transform_polygon = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_polygon_collision(
            &circle, transform_circle, &polygon, transform_polygon, &config
        );

        assert!(result.is_some());
        let manifold = result.unwrap();
        let contact = manifold.contact(0);
        
        assert!(approx_eq(contact.penetration, 0.5, 1e-4),
                "Penetration should be 0.5");
    }
}
