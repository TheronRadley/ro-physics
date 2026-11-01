//! Stress test scene for the Ro-Physics demo.
//!
//! This scene demonstrates the engine's robustness by creating a complex
//! simulation with many bodies, collisions, and constraints.

use ro_physics::prelude::*;

/// Create a stress test scene
pub fn create_stress_scene(world: &mut World) {
    // Create ground
    let ground = world.create_static_body(Transform::identity());
    let _ = world.add_box(ground, Vec2::new(50.0, 0.5), 0.0);

    // Create walls
    let left_wall =
        world.create_static_body(Transform::new(Vec2::new(-25.0, 15.0), Rot2::new(0.0)));
    let _ = world.add_box(left_wall, Vec2::new(0.5, 30.0), 0.0);

    let right_wall =
        world.create_static_body(Transform::new(Vec2::new(25.0, 15.0), Rot2::new(0.0)));
    let _ = world.add_box(right_wall, Vec2::new(0.5, 30.0), 0.0);

    // Create a pyramid of boxes
    let mut y = 1.0;
    let mut row_length = 15;

    while row_length > 0 {
        let x_start = -((row_length as f32 - 1.0) * 1.1) / 2.0;

        for i in 0..row_length {
            let x = x_start + i as f32 * 1.1;
            let body = world.create_dynamic_body(Transform::new(Vec2::new(x, y), Rot2::new(0.0)));
            let _ = world.add_box(body, Vec2::new(0.5, 0.5), 1.0);

            // Set random friction and restitution
            let handle = *world.collider_handles().last().expect("collider");
            if let Some(collider) = world.get_collider_mut(handle) {
                collider.set_friction(0.3 + rand::random::<f32>() * 0.7);
                collider.set_restitution(0.2 + rand::random::<f32>() * 0.6);
            }
        }

        y += 1.1;
        row_length -= 1;
    }

    // Add some circles on top
    for _ in 0..20 {
        let x = -10.0 + rand::random::<f32>() * 20.0;
        let y = 20.0 + rand::random::<f32>() * 5.0;
        let body = world.create_dynamic_body(Transform::new(Vec2::new(x, y), Rot2::new(0.0)));
        let _ = world.add_circle(body, 0.5 + rand::random::<f32>() * 0.5, 1.0);

        if let Some(b) = world.get_body_mut(body) {
            b.set_linear_velocity(Vec2::new(
                (rand::random::<f32>() - 0.5) * 10.0,
                (rand::random::<f32>() - 0.5) * 5.0,
            ));
        }
    }

    // Add some polygons
    for i in 0..10 {
        let x = -15.0 + rand::random::<f32>() * 30.0;
        let y = 15.0 + rand::random::<f32>() * 10.0;

        // Create a random polygon
        let n = 3 + (i % 7) as usize;
        let mut vertices = Vec::with_capacity(n);
        for j in 0..n {
            let angle = std::f32::consts::TAU * (j as f32) / (n as f32);
            let radius = 0.5 + rand::random::<f32>() * 0.3;
            vertices.push(Vec2::new(angle.cos() * radius, angle.sin() * radius));
        }

        let body = world.create_dynamic_body(Transform::new(Vec2::new(x, y), Rot2::new(0.0)));
        let _ = world.add_polygon(body, vertices, 1.0);

        if let Some(b) = world.get_body_mut(body) {
            b.set_angular_velocity((rand::random::<f32>() - 0.5) * 5.0);
        }
    }

    // Add a ragdoll
    create_ragdoll_at(world, Vec2::new(-5.0, 25.0));

    // Add some fast-moving projectiles (CCD test)
    for i in 0..5 {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(-20.0 + i as f32 * 10.0, 25.0),
            Rot2::new(0.0),
        ));
        let _ = world.add_circle(body, 0.3, 1.0);

        if let Some(b) = world.get_body_mut(body) {
            b.set_linear_velocity(Vec2::new(100.0, -50.0));
        }
    }
}

/// Create a ragdoll at a specific position
fn create_ragdoll_at(world: &mut World, offset: Vec2) {
    // Torso
    let torso = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x, offset.y),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(torso, Vec2::new(0.4, 0.6), 10.0);

    // Head
    let head = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x, offset.y + 0.8),
        Rot2::new(0.0),
    ));
    let _ = world.add_circle(head, 0.3, 10.0);

    // Arms
    let left_upper_arm = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x - 0.5, offset.y + 0.5),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(left_upper_arm, Vec2::new(0.15, 0.4), 10.0);

    let left_lower_arm = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x - 0.5, offset.y - 0.2),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(left_lower_arm, Vec2::new(0.15, 0.4), 10.0);

    let right_upper_arm = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x + 0.5, offset.y + 0.5),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(right_upper_arm, Vec2::new(0.15, 0.4), 10.0);

    let right_lower_arm = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x + 0.5, offset.y - 0.2),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(right_lower_arm, Vec2::new(0.15, 0.4), 10.0);

    // Legs
    let left_upper_leg = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x - 0.2, offset.y - 0.4),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(left_upper_leg, Vec2::new(0.15, 0.5), 10.0);

    let left_lower_leg = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x - 0.2, offset.y - 1.2),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(left_lower_leg, Vec2::new(0.15, 0.5), 10.0);

    let right_upper_leg = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x + 0.2, offset.y - 0.4),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(right_upper_leg, Vec2::new(0.15, 0.5), 10.0);

    let right_lower_leg = world.create_dynamic_body(Transform::new(
        Vec2::new(offset.x + 0.2, offset.y - 1.2),
        Rot2::new(0.0),
    ));
    let _ = world.add_box(right_lower_leg, Vec2::new(0.15, 0.5), 10.0);

    // Create constraints
    let _ = world.create_revolute_joint(head, torso, Vec2::new(0.0, -0.3));
    let _ = world.create_revolute_joint(torso, left_upper_arm, Vec2::new(-0.4, -0.2));
    let _ = world.create_revolute_joint(left_upper_arm, left_lower_arm, Vec2::new(0.0, -0.4));
    let _ = world.create_revolute_joint(torso, right_upper_arm, Vec2::new(0.4, -0.2));
    let _ = world.create_revolute_joint(right_upper_arm, right_lower_arm, Vec2::new(0.0, -0.4));
    let _ = world.create_revolute_joint(torso, left_upper_leg, Vec2::new(-0.2, -0.6));
    let _ = world.create_revolute_joint(left_upper_leg, left_lower_leg, Vec2::new(0.0, -0.5));
    let _ = world.create_revolute_joint(torso, right_upper_leg, Vec2::new(0.2, -0.6));
    let _ = world.create_revolute_joint(right_upper_leg, right_lower_leg, Vec2::new(0.0, -0.5));
}
