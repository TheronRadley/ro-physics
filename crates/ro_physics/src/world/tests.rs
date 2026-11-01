//! Integration tests for the world module.

use super::*;
use crate::ccd::CcdConfig;
use crate::math::{Rot2, Transform, Vec2};
use crate::solver::SolverConfig;
use crate::sleeping::SleepingConfig;

#[test]
fn test_world_creation() {
    let world = World::new(WorldConfig::default());
    assert_eq!(world.body_count(), 0);
    assert_eq!(world.collider_count(), 0);
}

#[test]
fn test_world_with_gravity() {
    let world = World::with_gravity(Vec2::new(0.0, -9.81));
    assert_eq!(world.gravity(), Vec2::new(0.0, -9.81));
}

#[test]
fn test_create_bodies() {
    let mut world = World::default();

    let body1 = world.create_dynamic_body(Transform::identity());
    let body2 = world.create_static_body(Transform::new(Vec2::new(0.0, -1.0), Rot2::identity()));

    assert_eq!(world.body_count(), 2);
    assert!(world.has_body(body1));
    assert!(world.has_body(body2));
}

#[test]
fn test_create_colliders() {
    let mut world = World::default();

    let body = world.create_dynamic_body(Transform::identity());
    let circle = world.add_circle(body, 1.0, 1.0);

    assert_eq!(world.collider_count(), 1);
    assert!(world.has_collider(circle));
}

#[test]
fn test_step_simulation() {
    let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

    // Create a dynamic body
    let body = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), Rot2::new(0.0)));
    let _ = world.add_circle(body, 0.5, 1.0);

    // Create a static ground
    let ground = world.create_static_body(Transform::identity());
    let _ = world.add_box(ground, Vec2::new(10.0, 0.5), 1.0);

    // Step the simulation
    world.step(1.0 / 60.0);

    // Body should have fallen
    let body_after = world.get_body(body).unwrap();
    assert!(
        body_after.position().y < 5.0,
        "Body should fall due to gravity"
    );
}

#[test]
fn test_collision_detection() {
    let mut world = World::default();

    // Create two circles that overlap
    let body1 = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body1, 1.0, 1.0);

    let body2 = world.create_dynamic_body(Transform::new(Vec2::new(1.5, 0.0), Rot2::new(0.0)));
    let _ = world.add_circle(body2, 1.0, 1.0);

    // Step the simulation
    world.step(1.0 / 60.0);

    // At least one manifold should be generated
    // Note: We can't directly check manifolds as it's private
    // But we can verify the world doesn't crash
}

#[test]
fn test_circle_vs_box_collision() {
    let mut world = World::default();

    // Create a dynamic circle
    let circle_body =
        world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), Rot2::new(0.0)));
    let _ = world.add_circle(circle_body, 0.5, 1.0);

    // Create a static box
    let box_body = world.create_static_body(Transform::identity());
    let _ = world.add_box(box_body, Vec2::new(2.0, 2.0), 1.0);

    // Step the simulation
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }

    // Circle should have fallen and potentially collided
    let circle = world.get_body(circle_body).unwrap();
    // Just verify it doesn't crash and the body moves
    assert!(circle.position().y < 5.0);
}

#[test]
fn test_box_vs_box_collision() {
    let mut world = World::default();

    // Create two dynamic boxes
    let body1 = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), Rot2::new(0.0)));
    let _ = world.add_box(body1, Vec2::new(0.5, 0.5), 1.0);

    let body2 = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 6.0), Rot2::new(0.0)));
    let _ = world.add_box(body2, Vec2::new(0.5, 0.5), 1.0);

    // Step the simulation
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }

    // Boxes should have fallen
    let box1 = world.get_body(body1).unwrap();
    let box2 = world.get_body(body2).unwrap();
    assert!(box1.position().y < 5.0);
    assert!(box2.position().y < 6.0);
}

#[test]
fn test_stacking_stability() {
    let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

    // Create ground
    let ground = world.create_static_body(Transform::identity());
    let _ = world.add_box(ground, Vec2::new(20.0, 0.5), 1.0);

    // Create a stack of boxes
    let mut y = 1.0;
    for _ in 0..5 {
        let body = world.create_dynamic_body(Transform::new(Vec2::new(0.0, y), Rot2::new(0.0)));
        let _ = world.add_box(body, Vec2::new(1.0, 0.5), 1.0);
        y += 1.0;
    }

    // Step the simulation
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    // Verify the world doesn't crash
    // Note: Without proper solver, boxes may not stack perfectly
    // but this test verifies the basic collision detection works
    assert_eq!(world.body_count(), 6); // 1 ground + 5 boxes
}

#[test]
fn test_distance_joint_basic() {
    let mut world = World::default();

    let body_a = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body_a, 1.0, 1.0);

    let body_b = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 2.0), Rot2::identity()));
    let _ = world.add_circle(body_b, 1.0, 1.0);

    let joint = world.create_distance_joint(body_a, body_b, Vec2::zero(), Vec2::zero());
    assert!(world.has_constraint(joint));
    assert_eq!(world.constraint_count(), 1);

    // Step the simulation
    world.step(1.0 / 60.0);

    // Bodies should still exist
    assert!(world.has_body(body_a));
    assert!(world.has_body(body_b));
}

#[test]
fn test_revolute_joint_basic() {
    let mut world = World::default();

    let body_a = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body_a, 1.0, 1.0);

    let body_b = world.create_dynamic_body(Transform::new(Vec2::new(2.0, 0.0), Rot2::identity()));
    let _ = world.add_circle(body_b, 1.0, 1.0);

    let joint = world.create_revolute_joint(body_a, body_b, Vec2::new(1.0, 0.0));
    assert!(world.has_constraint(joint));
    assert_eq!(world.constraint_count(), 1);

    // Step the simulation
    world.step(1.0 / 60.0);

    // Bodies should still exist
    assert!(world.has_body(body_a));
    assert!(world.has_body(body_b));
}

#[test]
fn test_motor_joint_basic() {
    let mut world = World::default();

    let body_a = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body_a, 1.0, 1.0);

    let body_b = world.create_dynamic_body(Transform::new(Vec2::new(2.0, 0.0), Rot2::identity()));
    let _ = world.add_circle(body_b, 1.0, 1.0);

    let joint = world.create_motor_joint(body_a, body_b);
    assert!(world.has_constraint(joint));
    assert_eq!(world.constraint_count(), 1);

    // Step the simulation
    world.step(1.0 / 60.0);

    // Bodies should still exist
    assert!(world.has_body(body_a));
    assert!(world.has_body(body_b));
}

#[test]
fn test_rotated_box_collision() {
    let mut world = World::default();

    // Create a static rotated box
    let static_body = world.create_static_body(Transform::new(
        Vec2::new(0.0, 0.0),
        Rot2::new(crate::constants::PI_OVER_FOUR),
    ));
    let _ = world.add_box(static_body, Vec2::new(2.0, 2.0), 1.0);

    // Create a dynamic circle
    let circle_body =
        world.create_dynamic_body(Transform::new(Vec2::new(3.0, 0.0), Rot2::new(0.0)));
    let _ = world.add_circle(circle_body, 1.0, 1.0);

    // Step the simulation
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }

    // Verify no crash
    assert!(world.body_count() == 2);
}

#[test]
fn test_event_handlers() {
    let mut world = World::default();

    let handler = Box::new(crate::collision::SimpleCollisionEventHandler::new());
    world.add_event_handler(handler);

    // Create two overlapping circles
    let body1 = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body1, 1.0, 1.0);

    let body2 = world.create_dynamic_body(Transform::new(Vec2::new(1.5, 0.0), Rot2::new(0.0)));
    let _ = world.add_circle(body2, 1.0, 1.0);

    // Step the simulation
    world.step(1.0 / 60.0);

    // Events should have been generated
    // Note: We can't directly check the handler's events as it's in a Box<dyn Trait>
    // but we can verify no crash occurs
}

#[test]
fn test_destroy_body() {
    let mut world = World::default();

    let body = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body, 1.0, 1.0);

    assert_eq!(world.body_count(), 1);
    assert_eq!(world.collider_count(), 1);

    assert!(world.destroy_body(body));
    assert_eq!(world.body_count(), 0);
    assert_eq!(world.collider_count(), 0);
}

#[test]
fn test_clear_world() {
    let mut world = World::default();

    let body1 = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body1, 1.0, 1.0);

    let body2 = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::new(0.0)));
    let _ = world.add_circle(body2, 1.0, 1.0);

    assert_eq!(world.body_count(), 2);
    assert_eq!(world.collider_count(), 2);

    world.clear();
    assert_eq!(world.body_count(), 0);
    assert_eq!(world.collider_count(), 0);

#[test]
fn test_invalid_input_is_refused_instead_of_panicking() {
    let mut world = World::default();
    let body = world.create_dynamic_body(Transform::identity());

    // Shapes that cannot exist
    assert!(world.add_circle(body, -1.0, 1.0).is_null());
    assert!(world.add_circle(body, f32::NAN, 1.0).is_null());
    assert!(world.add_circle(body, f32::INFINITY, 1.0).is_null());
    assert!(world
        .add_box(body, Vec2::new(f32::NAN, 1.0), 1.0)
        .is_null());
    assert!(world.add_polygon(body, Vec::new(), 1.0).is_null());
    assert!(world
        .add_polygon(body, vec![Vec2::zero(), Vec2::new(1.0, 0.0)], 1.0)
        .is_null());

    // Densities that would give the body a NaN or negative mass
    assert!(world.add_circle(body, 1.0, -1.0).is_null());
    assert!(world.add_circle(body, 1.0, f32::NAN).is_null());

    // Bodies that no spatial index could hold
    let bad = Transform::new(Vec2::new(f32::NAN, 0.0), Rot2::identity());
    assert!(world.create_dynamic_body(bad).is_null());
    assert!(world.create_static_body(bad).is_null());

    // Nothing invalid was stored
    assert_eq!(world.collider_count(), 0);
    assert_eq!(world.body_count(), 1);

    // Gravity survives a non-finite value
    let gravity = world.gravity();
    world.set_gravity(Vec2::new(f32::NAN, 0.0));
    assert_eq!(world.gravity(), gravity);

    // Valid input still works after all that
    assert!(!world.add_circle(body, 1.0, 1.0).is_null());
    assert_eq!(world.collider_count(), 1);
}

#[test]
fn test_world_config_sanitizes_non_finite_values() {
    let config = WorldConfig {
        gravity: Vec2::new(f32::NAN, f32::INFINITY),
        solver_config: SolverConfig {
            baumgarte_factor: f32::NAN,
            max_position_correction: -1.0,
            restitution_threshold: f32::NAN,
            ..SolverConfig::default()
        },
        sleeping_config: SleepingConfig {
            linear_threshold: -1.0,
            angular_threshold: f32::NAN,
            sleep_time: f32::NAN,
            ..SleepingConfig::default()
        },
        ccd_config: CcdConfig {
            toi_tolerance: 0.0,
            max_toi: f32::NAN,
            max_toi_iterations: 0,
            ..CcdConfig::default()
        },
        ..WorldConfig::default()
    };

    let sanitized = config.sanitize();

    assert_eq!(sanitized.gravity, Vec2::zero());
    assert!(sanitized.solver_config.baumgarte_factor.is_finite());
    assert!(sanitized.solver_config.max_position_correction >= 0.0);
    assert!(sanitized.solver_config.restitution_threshold >= 0.0);
    assert!(sanitized.sleeping_config.linear_threshold >= 0.0);
    assert!(sanitized.sleeping_config.angular_threshold.is_finite());
    assert!(sanitized.sleeping_config.sleep_time >= 0.0);
    assert!(sanitized.ccd_config.toi_tolerance > 0.0);
    assert!(sanitized.ccd_config.max_toi.is_finite());
    assert!(sanitized.ccd_config.max_toi_iterations > 0);

    // A world built from the raw config must be usable
    let mut world = World::new(config);
    assert_eq!(world.gravity(), Vec2::zero());

    let body = world.create_dynamic_body(Transform::identity());
    let collider = world.add_circle(body, 1.0, 1.0);
    assert!(!body.is_null());
    assert!(!collider.is_null());

    world.step(1.0 / 60.0);
    assert!(world
        .get_body(body)
        .is_some_and(|body| body.position().is_finite()));
}
}
