//! Circle-circle collision detection.
//!
//! This module provides collision detection between two circles.

use crate::constants::EPSILON;
use crate::math::{Transform, Vec2};
use crate::shapes::Circle;
use super::NarrowPhaseConfig;
use super::super::ContactManifold;

/// Detect collision between two circles
///
/// Returns a contact manifold if the circles overlap, None otherwise.
#[inline]
#[must_use]
pub fn circle_circle_collision(
    circle_a: &Circle,
    transform_a: Transform,
    circle_b: &Circle,
    transform_b: Transform,
    _config: &NarrowPhaseConfig,
) -> Option<ContactManifold> {
    // Calculate the distance between centers
    let center_a = transform_a.position;
    let center_b = transform_b.position;
    let delta = center_b - center_a;
    let distance_squared = delta.length_squared();
    let radius_sum = circle_a.radius + circle_b.radius;

    // Check for separation
    if distance_squared > radius_sum * radius_sum {
        return None;
    }

    // Calculate the distance
    let distance = distance_squared.sqrt();

    // Calculate the contact normal
    let normal = if distance > EPSILON {
        delta / distance
    } else {
        // Circles are concentric, use arbitrary normal
        Vec2::unit_x()
    };

    // Calculate penetration depth
    let penetration = radius_sum - distance;

    // Calculate the contact point (midway between the two circles)
    let contact_point = center_a + normal * (circle_a.radius - penetration / 2.0);

    // Create the contact manifold
    let mut manifold = ContactManifold::new();
    manifold.add_contact(contact_point, normal, penetration, 0.0, 0.0, 0);

    Some(manifold)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_overlapping_circles() {
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(1.5, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = circle_circle_collision(circle_a, transform_a, circle_b, transform_b, &config);

        assert!(result.is_some());
        let manifold = result.unwrap();
        assert_eq!(manifold.contact_count(), 1);

        let contact = manifold.contact(0);
        assert!(approx_eq(contact.penetration, 0.5, 1e-5));
    }

    #[test]
    fn test_touching_circles() {
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(2.0, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = circle_circle_collision(circle_a, transform_a, circle_b, transform_b, &config);

        assert!(result.is_some());
        let manifold = result.unwrap();
        assert_eq!(manifold.contact_count(), 1);

        let contact = manifold.contact(0);
        assert!(approx_eq(contact.penetration, 0.0, 1e-5));
    }

    #[test]
    fn test_separated_circles() {
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(3.0, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = circle_circle_collision(circle_a, transform_a, circle_b, transform_b, &config);

        assert!(result.is_none());
    }

    #[test]
    fn test_concentric_circles() {
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::identity();

        let config = NarrowPhaseConfig::default();
        let result = circle_circle_collision(circle_a, transform_a, circle_b, transform_b, &config);

        assert!(result.is_some());
        let manifold = result.unwrap();
        assert_eq!(manifold.contact_count(), 1);

        let contact = manifold.contact(0);
        assert!(approx_eq(contact.penetration, 2.0, 1e-5));
    }

    #[test]
    fn test_unequal_radii() {
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(2.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(2.5, 0.0));

        let config = NarrowPhaseConfig::default();
        let result = circle_circle_collision(circle_a, transform_a, circle_b, transform_b, &config);

        assert!(result.is_some());
        let manifold = result.unwrap();
        assert_eq!(manifold.contact_count(), 1);

        let contact = manifold.contact(0);
        // Penetration = (1 + 2) - 2.5 = 0.5
        assert!(approx_eq(contact.penetration, 0.5, 1e-5));
    }

    #[test]
    fn test_contact_normal() {
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        let transform_a = Transform::identity();
        let transform_b = Transform::from_translation(Vec2::new(1.5, 0.5));

        let config = NarrowPhaseConfig::default();
        let result = circle_circle_collision(circle_a, transform_a, circle_b, transform_b, &config);

        assert!(result.is_some());
        let manifold = result.unwrap();
        let contact = manifold.contact(0);

        // Normal should point from A to B
        let expected_normal = Vec2::new(1.5, 0.5).normalize();
        assert!(approx_eq(contact.normal.x, expected_normal.x, 1e-5));
        assert!(approx_eq(contact.normal.y, expected_normal.y, 1e-5));
    }
}
