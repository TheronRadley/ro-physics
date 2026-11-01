//! Fuzz the world step: arbitrary bodies, colliders, joints and timesteps.

#![no_main]

use libfuzzer_sys::fuzz_target;

use ro_physics::geometry::Ray;
use ro_physics::math::{Rot2, Transform, Vec2};
use ro_physics::world::World;

/// Read an `f32` from the input, or 0.0 when it has run out
fn take_f32(data: &[u8], cursor: &mut usize) -> f32 {
    if *cursor + 4 > data.len() {
        *cursor = data.len();
        return 0.0;
    }

    let bytes = [data[*cursor], data[*cursor + 1], data[*cursor + 2], data[*cursor + 3]];
    *cursor += 4;
    f32::from_le_bytes(bytes)
}

/// Read a byte from the input, or 0 when it has run out
fn take_u8(data: &[u8], cursor: &mut usize) -> u8 {
    if *cursor >= data.len() {
        return 0;
    }

    let byte = data[*cursor];
    *cursor += 1;
    byte
}

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }

    let mut world = World::default();
    let mut cursor = 0;
    let mut handles = Vec::new();

    while cursor + 8 < data.len() && handles.len() < 32 {
        let x = take_f32(data, &mut cursor);
        let y = take_f32(data, &mut cursor);
        let angle = take_f32(data, &mut cursor);
        let body_type = take_u8(data, &mut cursor) % 3;
        let shape = take_u8(data, &mut cursor) % 3;
        let size = take_f32(data, &mut cursor);
        let density = take_f32(data, &mut cursor);

        let transform = Transform::new(Vec2::new(x, y), Rot2::new(angle));
        let body = match body_type {
            0 => world.create_static_body(transform),
            1 => world.create_kinematic_body(transform),
            _ => world.create_dynamic_body(transform),
        };

        if body.is_null() {
            continue;
        }

        match shape {
            0 => {
                let _ = world.add_circle(body, size, density);
            }
            1 => {
                let _ = world.add_box(body, Vec2::new(size, size), density);
            }
            _ => {
                let vertices = vec![
                    Vec2::zero(),
                    Vec2::new(size, 0.0),
                    Vec2::new(size, size),
                ];
                let _ = world.add_polygon(body, vertices, density);
            }
        }

        let velocity = Vec2::new(
            take_f32(data, &mut cursor),
            take_f32(data, &mut cursor),
        );
        if let Some(body) = world.get_body_mut(body) {
            body.set_linear_velocity(velocity);
        }

        handles.push(body);
    }

    // Join a few pairs so the constraint solver is exercised too
    for pair in handles.windows(2).take(8) {
        let _ = world.create_distance_joint(pair[0], pair[1], Vec2::zero(), Vec2::zero());
    }

    for _ in 0..16 {
        let dt = take_f32(data, &mut cursor);
        world.step(dt);

        for &handle in &handles {
            if let Some(body) = world.get_body(handle) {
                assert!(
                    body.position().is_finite(),
                    "non-finite position after a step: {:?}",
                    body.position()
                );
                assert!(
                    body.linear_velocity().is_finite(),
                    "non-finite velocity after a step: {:?}",
                    body.linear_velocity()
                );
            }
        }
    }

    // Queries must be total functions of the current state as well
    let _ = world.point_query(Vec2::new(take_f32(data, &mut cursor), 0.0));
    let _ = world.ray_cast(
        Ray::new(Vec2::zero(), Vec2::new(1.0, 0.0)),
        take_f32(data, &mut cursor),
    );
});
