//! Property-based tests for Ro-Physics.
//!
//! The tests in `conservation.rs` check hand-picked scenarios against
//! closed-form answers. These check invariants that have to hold for *any*
//! input: shapes stay valid, geometry queries agree with the analytic result,
//! and a world made of arbitrary bodies never produces a non-finite state.
//!
//! Run with `cargo test --test properties`.

use proptest::prelude::*;
use ro_physics::constants::PI;
use ro_physics::prelude::*;

/// Build a world with a static floor and one dynamic circle per generated body
fn build_world(positions: &[(f32, f32)], radii: &[f32]) -> World {
    let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

    let ground = world.create_static_body(Transform::new(Vec2::new(0.0, -10.0), Rot2::identity()));
    let _ = world.add_box(ground, Vec2::new(50.0, 1.0), 0.0);

    for (position, radius) in positions.iter().zip(radii.iter()) {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(position.0, position.1),
            Rot2::identity(),
        ));
        let _ = world.add_circle(body, *radius, 1.0);
    }

    world
}

/// Relative comparison, so the tolerance scales with the magnitude
fn close(a: f32, b: f32, tolerance: f32) -> bool {
    (a - b).abs() <= tolerance * a.abs().max(b.abs()).max(1.0)
}

proptest! {
    /// A circle's area and inertia are the closed-form values
    #[test]
    fn circle_area_and_inertia_match_the_analytic_formula(
        radius in 1e-3f32..1e3,
        mass in 1e-3f32..1e3,
    ) {
        let circle = Circle::try_new(radius).expect("a positive radius is valid");

        let area = circle.area();
        prop_assert!(close(area, PI * radius * radius, 1e-4), "area = {area}");

        let inertia = circle.inertia(mass);
        prop_assert!(inertia.is_finite() && inertia > 0.0, "inertia = {inertia}");
        prop_assert!(
            close(inertia, 0.5 * mass * radius * radius, 1e-4),
            "inertia = {inertia}"
        );
    }

    /// An invalid radius is refused instead of producing a broken shape
    #[test]
    fn circle_rejects_bad_radii(radius in -1e3f32..0.0) {
        prop_assert!(Circle::try_new(radius).is_err());
    }

    /// Rotating a vector never changes its length
    #[test]
    fn rotation_preserves_length(
        angle in -1e3f32..1e3,
        x in -1e3f32..1e3,
        y in -1e3f32..1e3,
    ) {
        let vector = Vec2::new(x, y);
        let rotated = Rot2::new(angle).rotate(vector);

        prop_assert!(
            close(rotated.length(), vector.length(), 1e-4),
            "|{:?}| became |{:?}|", vector, rotated
        );
    }

    /// A transform maps its own origin onto its position
    #[test]
    fn transform_maps_the_origin_to_its_position(
        x in -1e3f32..1e3,
        y in -1e3f32..1e3,
        angle in -1e2f32..1e2,
    ) {
        let position = Vec2::new(x, y);
        let transform = Transform::new(position, Rot2::new(angle));
        let mapped = transform.transform_point(Vec2::zero());

        prop_assert!(
            close(mapped.distance(position), 0.0, 1e-4),
            "{transform:?} mapped the origin to {mapped:?} instead of {position:?}"
        );
    }

    /// A collider's AABB is valid and at least as large as the shape it wraps
    #[test]
    fn collider_aabb_contains_the_shape(
        x in -1e2f32..1e2,
        y in -1e2f32..1e2,
        radius in 1e-2f32..1e2,
        angle in -1e2f32..1e2,
    ) {
        let transform = Transform::new(Vec2::new(x, y), Rot2::new(angle));
        let shape = Shape::Circle(Circle::try_new(radius).expect("a positive radius is valid"));
        let collider = Collider::new(ColliderHandle::new(1, 0), BodyHandle::new(1, 0), shape);
        let aabb = collider.aabb(transform);

        prop_assert!(aabb.is_valid(), "invalid AABB: {aabb:?}");
        prop_assert!(aabb.contains_point(Vec2::new(x, y)));
        prop_assert!(aabb.width() >= 2.0 * radius - 1e-3, "width = {}", aabb.width());
        prop_assert!(aabb.height() >= 2.0 * radius - 1e-3, "height = {}", aabb.height());
    }

    /// Three points make either a valid triangle or a rejected one
    #[test]
    fn three_points_make_either_a_triangle_or_an_error(
        ax in -1e2f32..1e2,
        ay in -1e2f32..1e2,
        bx in -1e2f32..1e2,
        by in -1e2f32..1e2,
        cx in -1e2f32..1e2,
        cy in -1e2f32..1e2,
    ) {
        let vertices = vec![Vec2::new(ax, ay), Vec2::new(bx, by), Vec2::new(cx, cy)];

        if let Ok(polygon) = Polygon::try_new(vertices) {
            prop_assert!(polygon.area() > 0.0, "area = {}", polygon.area());
            prop_assert!(polygon.is_valid());
            prop_assert!(polygon.validate());
            prop_assert_eq!(polygon.normals().len(), polygon.vertex_count());

            let aabb = polygon.aabb(Transform::identity());
            prop_assert!(aabb.is_valid(), "invalid AABB: {aabb:?}");
            prop_assert!(aabb.contains_point(polygon.centroid()));
        }
        // A near-degenerate (collinear) triangle may be refused
    }

    /// Mass properties follow from density and area
    #[test]
    fn mass_follows_density_and_area(radius in 1e-2f32..1e1, density in 1e-2f32..1e2) {
        let mut world = World::default();
        let handle = world.create_dynamic_body(Transform::identity());
        let collider = world.add_circle(handle, radius, density);
        prop_assert!(!collider.is_null());

        let body = world.get_body(handle).expect("the body was just created");
        let expected = density * PI * radius * radius;

        prop_assert!(close(body.mass(), expected, 1e-3), "mass = {}", body.mass());
        prop_assert!(body.mass() > 0.0);
        prop_assert!(close(body.inv_mass(), 1.0 / body.mass(), 1e-3));
        prop_assert!(body.inertia() > 0.0);
    }

    /// Whatever the inputs, a stepped world stays finite
    #[test]
    fn stepping_a_generated_world_stays_finite(
        positions in prop::collection::vec((-1e2f32..1e2, -1e2f32..1e2), 1..8),
        radii in prop::collection::vec(1e-1f32..1e1, 1..8),
        frames in 1usize..60,
    ) {
        let mut world = build_world(&positions, &radii);

        for _ in 0..frames {
            world.step(1.0 / 60.0);
        }

        for handle in world.body_handles() {
            let body = world.get_body(handle).expect("a live handle");

            prop_assert!(body.position().is_finite(), "position = {:?}", body.position());
            prop_assert!(
                body.linear_velocity().is_finite(),
                "velocity = {:?}",
                body.linear_velocity()
            );
            prop_assert!(
                body.angular_velocity().is_finite(),
                "angular velocity = {}",
                body.angular_velocity()
            );
            prop_assert!(body.rotation().angle().is_finite());
            prop_assert!(body.mass() >= 0.0 && body.mass().is_finite());
        }
    }

    /// The same inputs produce the same world, bit for bit
    #[test]
    fn the_same_inputs_produce_the_same_world(
        positions in prop::collection::vec((-1e1f32..1e1, -1e1f32..1e1), 1..6),
        radii in prop::collection::vec(1e-1f32..1e1, 1..6),
    ) {
        let mut left = build_world(&positions, &radii);
        let mut right = build_world(&positions, &radii);

        for _ in 0..60 {
            left.step(1.0 / 60.0);
            right.step(1.0 / 60.0);
        }

        for (a, b) in left.body_handles().iter().zip(right.body_handles().iter()) {
            let left_body = left.get_body(*a).expect("a live handle");
            let right_body = right.get_body(*b).expect("a live handle");

            prop_assert_eq!(left_body.position(), right_body.position());
            prop_assert_eq!(left_body.linear_velocity(), right_body.linear_velocity());
            prop_assert_eq!(left_body.rotation().angle(), right_body.rotation().angle());
            prop_assert_eq!(left_body.angular_velocity(), right_body.angular_velocity());
        }
    }
}

#[test]
fn creation_and_deletion_cycles_keep_the_world_consistent() {
    let mut world = World::default();

    for cycle in 0..200 {
        let body = world.create_dynamic_body(Transform::identity());
        let collider = world.add_circle(body, 1.0, 1.0);
        assert!(!body.is_null(), "cycle {cycle}: body was refused");
        assert!(!collider.is_null(), "cycle {cycle}: collider was refused");

        assert_eq!(world.body_count(), 1);
        assert_eq!(world.collider_count(), 1);

        assert!(world.destroy_body(body));
        assert_eq!(world.body_count(), 0, "cycle {cycle}: body leaked");
        assert_eq!(world.collider_count(), 0, "cycle {cycle}: collider leaked");

        // The handles are stale now
        assert!(world.get_body(body).is_none());
        assert!(!world.has_body(body));
    }
}

#[test]
fn a_long_run_stays_bounded() {
    let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

    let ground = world.create_static_body(Transform::new(Vec2::new(0.0, -10.0), Rot2::identity()));
    let _ = world.add_box(ground, Vec2::new(50.0, 1.0), 0.0);

    for index in 0..40 {
        let x = (index % 8) as f32 * 2.0 - 7.0;
        let y = (index / 8) as f32 * 2.0;
        let body = world.create_dynamic_body(Transform::new(Vec2::new(x, y), Rot2::identity()));
        let _ = world.add_circle(body, 0.5, 1.0);
    }

    // Ten thousand steps: longer than any game session, and long enough for
    // numerical drift to show up if there is any.
    for _ in 0..10_000 {
        world.step(1.0 / 60.0);
    }

    for handle in world.body_handles() {
        let body = world.get_body(handle).expect("a live handle");
        assert!(
            body.position().is_finite(),
            "position drifted to {:?}",
            body.position()
        );
        assert!(
            body.linear_velocity().is_finite(),
            "velocity drifted to {:?}",
            body.linear_velocity()
        );
        // Nothing may sink through the floor or fly away
        assert!(
            body.position().y > -20.0,
            "body fell through the floor: y = {}",
            body.position().y
        );
        assert!(
            body.position().length() < 1.0e4,
            "body escaped: {:?}",
            body.position()
        );
    }
}
