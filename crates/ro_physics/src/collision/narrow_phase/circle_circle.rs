//! Circle-circle collision detection.

use super::super::{ContactId, ContactManifold};
use super::NarrowPhaseConfig;
use crate::constants::EPSILON;
use crate::math::{Transform, Vec2};
use crate::shapes::Circle;

/// Detect collision between two circles.
///
/// The returned manifold's normal points from A to B and its penetration is
/// positive while the circles overlap. Concentric circles fall back to the
/// +X axis, which is arbitrary but deterministic.
#[inline]
#[must_use]
pub fn circle_circle_collision(
    circle_a: &Circle,
    transform_a: Transform,
    circle_b: &Circle,
    transform_b: Transform,
    _config: &NarrowPhaseConfig,
) -> Option<ContactManifold> {
    let center_a = transform_a.position;
    let center_b = transform_b.position;
    let delta = center_b - center_a;
    let distance_squared = delta.length_squared();
    let radius_sum = circle_a.radius() + circle_b.radius();

    // Check for separation
    if distance_squared > radius_sum * radius_sum {
        return None;
    }

    // Calculate the distance
    let distance = distance_squared.sqrt();

    // Calculate the contact normal (from A to B)
    let normal = if distance > EPSILON {
        delta / distance
    } else {
        // Circles are concentric, use an arbitrary but deterministic normal
        Vec2::unit_x()
    };

    // Calculate penetration depth
    let penetration = radius_sum - distance;

    // Contact point: halfway between the two overlapping surfaces
    let contact_point = center_a + normal * (circle_a.radius() - penetration / 2.0);

    // Create the contact manifold
    let mut manifold = ContactManifold::new();
    manifold.add_contact(
        contact_point,
        normal,
        penetration,
        0.0,
        0.0,
        ContactId::new(0, 0),
    );

    Some(manifold)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_overlapping_circles() {
        let a = Circle::new(1.0);
        let b = Circle::new(1.0);
        let config = NarrowPhaseConfig::default();

        let manifold = circle_circle_collision(
            &a,
            Transform::from_translation(Vec2::zero()),
            &b,
            Transform::from_translation(Vec2::new(1.5, 0.0)),
            &config,
        )
        .expect("expected a contact");

        assert_eq!(manifold.contact_count(), 1);

        let contact = manifold.contact(0);
        // Normal points from A to B
        assert_eq!(contact.normal(), Vec2::unit_x());
        assert!(approx_eq(contact.penetration(), 0.5));
        // Contact point lies midway between the two surfaces
        assert!(approx_eq(contact.position().x, 0.75));
        assert!(approx_eq(contact.position().y, 0.0));
    }

    #[test]
    fn test_touching_circles() {
        let a = Circle::new(1.0);
        let b = Circle::new(1.0);
        let config = NarrowPhaseConfig::default();

        let manifold = circle_circle_collision(
            &a,
            Transform::from_translation(Vec2::zero()),
            &b,
            Transform::from_translation(Vec2::new(2.0, 0.0)),
            &config,
        )
        .expect("touching circles produce a contact");

        assert!(approx_eq(manifold.contact(0).penetration(), 0.0));
        assert!(approx_eq(manifold.contact(0).position().x, 1.0));
    }

    #[test]
    fn test_separated_circles() {
        let a = Circle::new(1.0);
        let b = Circle::new(1.0);
        let config = NarrowPhaseConfig::default();

        let manifold = circle_circle_collision(
            &a,
            Transform::from_translation(Vec2::zero()),
            &b,
            Transform::from_translation(Vec2::new(3.0, 0.0)),
            &config,
        );

        assert!(manifold.is_none());
    }

    #[test]
    fn test_concentric_circles() {
        let a = Circle::new(1.0);
        let b = Circle::new(0.5);
        let config = NarrowPhaseConfig::default();

        let manifold = circle_circle_collision(
            &a,
            Transform::from_translation(Vec2::new(2.0, 3.0)),
            &b,
            Transform::from_translation(Vec2::new(2.0, 3.0)),
            &config,
        )
        .expect("concentric circles overlap");

        // Both centres coincide, so the small circle sits entirely inside the
        // big one: separating them takes the sum of both radii. The normal has
        // no geometric meaning here, but it must stay finite and deterministic.
        assert!(approx_eq(manifold.contact(0).penetration(), 1.5));
        assert!(manifold.normal().is_finite());
        assert_eq!(manifold.normal(), Vec2::unit_x());
    }
}
