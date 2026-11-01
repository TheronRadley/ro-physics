//! Fuzz shape construction and the geometry queries built on top of it.

#![no_main]

use libfuzzer_sys::fuzz_target;

use ro_physics::geometry::utils::{aabb_intersection, aabb_overlaps, point_in_aabb};
use ro_physics::geometry::Aabb;
use ro_physics::math::{Rot2, Transform, Vec2};
use ro_physics::shapes::{Circle, Polygon, Shape};

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }

    // A circle, built through the fallible constructor: `Circle::new` panics
    // on purpose, `try_new` is what the engine itself uses.
    let radius = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    if let Ok(circle) = Circle::try_new(radius) {
        let shape = Shape::Circle(circle);
        let transform = Transform::identity();
        let _ = shape.aabb(transform);
        let _ = shape.area();
        let _ = shape.inertia(1.0);
        let _ = shape.closest_point(Vec2::zero(), transform);
        let _ = shape.support(Vec2::new(1.0, 0.0));
        assert!(shape.is_valid(), "a constructed circle must be valid");
    }

    // A polygon from whatever vertices the input happens to describe
    let count = usize::from(data[4]) % 8;
    let mut vertices = Vec::with_capacity(count);
    for index in 0..count {
        let base = 5 + index * 8;
        if base + 8 > data.len() {
            break;
        }

        let x = f32::from_le_bytes([data[base], data[base + 1], data[base + 2], data[base + 3]]);
        let y = f32::from_le_bytes([data[base + 4], data[base + 5], data[base + 6], data[base + 7]]);
        vertices.push(Vec2::new(x, y));
    }

    if let Ok(polygon) = Polygon::try_new(vertices) {
        let shape = Shape::Polygon(polygon);
        let transform = Transform::new(
            Vec2::new(f32::from(data[5 % data.len()]), 0.0),
            Rot2::new(f32::from(data[6 % data.len()])),
        );
        let aabb = shape.aabb(transform);
        assert!(aabb.min.x <= aabb.max.x, "an AABB must be ordered");
        let _ = shape.area();
        let _ = shape.inertia(1.0);
        let _ = shape.closest_point(Vec2::zero(), transform);
    }

    // AABB helpers
    let a = Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0));
    let b = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0));
    assert!(aabb_overlaps(a, b));
    assert!(aabb_intersection(a, b).is_some());
    assert!(point_in_aabb(Vec2::zero(), a));
});
