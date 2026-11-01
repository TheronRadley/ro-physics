//! Stress tests for Ro-Physics
//!
//! These tests verify the engine behaves correctly under heavy load
//! and edge cases that might cause numerical instability.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use ro_physics::collision::{NarrowPhase, NarrowPhaseConfig};
use ro_physics::prelude::*;

/// Stress test: Many bodies with random properties
fn stress_many_bodies(c: &mut Criterion) {
    let mut group = c.benchmark_group("stress/many_bodies");
    
    for count in [100, 500, 1000].iter() {
        group.throughput(Throughput::Elements(*count as u64));
        
        group.bench_function(format!("bodies_{}", count), |b| {
            let mut world = World::with_gravity(Vec2::new(0.0, -9.81));
            
            // Create ground
            let ground = world.create_static_body(Transform::identity());
            let _ = world.add_box(ground, Vec2::new(100.0, 0.5), 0.0);
            
            // Create many dynamic bodies
            for i in 0..*count {
                let x = (i as f32 % 20.0) * 2.0 - 20.0;
                let y = (i as f32 / 20.0) * 2.0 + 10.0;
                let body = world.create_dynamic_body(Transform::new(Vec2::new(x, y),Rot2::new(0.0)));
                if i % 2 == 0 {
                    let _ = world.add_circle(body, 0.5, 1.0);
                } else {
                    let _ = world.add_box(body, Vec2::new(0.5, 0.5), 1.0);
                }
            }
            
            b.iter(|| {
                world.step(black_box(1.0 / 60.0));
            });
        });
    }
    
    group.finish();
}

/// Stress test: High velocity collisions (CCD test)
fn stress_high_velocity(c: &mut Criterion) {
    let mut group = c.benchmark_group("stress/high_velocity");
    
    for count in [10, 50, 100].iter() {
        group.throughput(Throughput::Elements(*count as u64));
        
        group.bench_function(format!("fast_bodies_{}", count), |b| {
            let mut world = World::new(
                WorldConfig::new()
                    .with_ccd_config(CcdConfig::new().with_enabled(true))
            );
            
            // Create ground
            let ground = world.create_static_body(Transform::identity());
            let _ = world.add_box(ground, Vec2::new(100.0, 0.5), 0.0);
            
            // Create fast-moving bodies
            for i in 0..*count {
                let body = world.create_dynamic_body(Transform::new(
                    Vec2::new(i as f32 * 5.0, 20.0),Rot2::new(0.0)));
                let _ = world.add_circle(body, 0.5, 1.0);
                if let Some(b) = world.get_body_mut(body) {
                    b.set_linear_velocity(Vec2::new(0.0, -100.0));
                }
            }
            
            b.iter(|| {
                world.step(black_box(1.0 / 60.0));
            });
        });
    }
    
    group.finish();
}

/// Stress test: Constrained system (ragdoll-like)
fn stress_constraints(c: &mut Criterion) {
    let mut group = c.benchmark_group("stress/constraints");
    
    for count in [10, 50, 100].iter() {
        group.throughput(Throughput::Elements(*count as u64));
        
        group.bench_function(format!("ragdolls_{}", count), |b| {
            let mut world = World::with_gravity(Vec2::new(0.0, -9.81));
            
            // Create ground
            let ground = world.create_static_body(Transform::identity());
            let _ = world.add_box(ground, Vec2::new(100.0, 0.5), 0.0);
            
            // Create multiple ragdolls
            for r in 0..*count {
                let offset = r as f32 * 5.0;
                
                // Torso
                let torso = world.create_dynamic_body(Transform::new(
                    Vec2::new(offset, 10.0),Rot2::new(0.0)));
                let _ = world.add_box(torso, Vec2::new(0.4, 0.6), 10.0);
                
                // Head
                let head = world.create_dynamic_body(Transform::new(
                    Vec2::new(offset, 10.8),Rot2::new(0.0)));
                let _ = world.add_circle(head, 0.3, 10.0);
                
                // Connect with joint
                world.create_revolute_joint(torso, head, Vec2::new(0.0, -0.3));
            }
            
            b.iter(|| {
                world.step(black_box(1.0 / 60.0));
            });
        });
    }
    
    group.finish();
}

/// Stress test: Complex polygon collisions
fn stress_polygons(c: &mut Criterion) {
    let mut group = c.benchmark_group("stress/polygons");
    
    for count in [10, 50, 100].iter() {
        group.throughput(Throughput::Elements(*count as u64));
        
        group.bench_function(format!("polygons_{}", count), |b| {
            let mut world = World::with_gravity(Vec2::new(0.0, -9.81));
            
            // Create ground
            let ground = world.create_static_body(Transform::identity());
            let _ = world.add_box(ground, Vec2::new(100.0, 0.5), 0.0);
            
            // Create polygons with different vertex counts
            for i in 0..*count {
                let body = world.create_dynamic_body(Transform::new(
                    Vec2::new(i as f32 * 3.0, 10.0),Rot2::new(0.0)));
                
                // Create polygon with 3-8 vertices
                let n = 3 + (i % 6) as usize;
                let mut vertices = Vec::with_capacity(n);
                for j in 0..n {
                    let angle = std::f32::consts::TAU * (j as f32) / (n as f32);
                    vertices.push(Vec2::new(angle.cos() * 0.5, angle.sin() * 0.5));
                }
                world.add_polygon(body, vertices, 1.0);
            }
            
            b.iter(|| {
                world.step(black_box(1.0 / 60.0));
            });
        });
    }
    
    group.finish();
}

/// Benchmark: Query performance
fn bench_queries(c: &mut Criterion) {
    let mut group = c.benchmark_group("queries");
    
    // Setup world with many bodies
    let mut world = World::default();
    let ground = world.create_static_body(Transform::identity());
    let _ = world.add_box(ground, Vec2::new(100.0, 0.5), 0.0);
    
    for i in 0..1000 {
        let x = (i as f32 % 50.0) * 2.0 - 50.0;
        let y = (i as f32 / 50.0) * 2.0 + 5.0;
        let body = world.create_dynamic_body(Transform::new(Vec2::new(x, y),Rot2::new(0.0)));
        let _ = world.add_circle(body, 0.5, 1.0);
    }
    
    group.bench_function("point_query", |b| {
        b.iter(|| {
            black_box(world.point_query(Vec2::new(0.0, 5.0)));
        });
    });
    
    group.bench_function("aabb_query", |b| {
        use ro_physics::geometry::Aabb;
        let aabb = Aabb::new(Vec2::new(-5.0, -5.0), Vec2::new(5.0, 5.0));
        b.iter(|| {
            black_box(world.aabb_query(aabb));
        });
    });
    
    group.bench_function("ray_cast", |b| {
        use ro_physics::geometry::Ray;
        let ray = Ray::new(Vec2::new(-50.0, 5.0), Vec2::new(1.0, 0.0));
        b.iter(|| {
            black_box(world.ray_cast(ray, 100.0));
        });
    });
    
    group.bench_function("ray_cast_all", |b| {
        use ro_physics::geometry::Ray;
        let ray = Ray::new(Vec2::new(-50.0, 5.0), Vec2::new(1.0, 0.0));
        b.iter(|| {
            black_box(world.ray_cast_all(ray, 100.0));
        });
    });
    
    group.finish();
}

/// Benchmark: Collision detection
fn bench_collision(c: &mut Criterion) {
    let mut group = c.benchmark_group("collision");
    
    group.bench_function("circle_circle", |b| {
        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);
        
        b.iter(|| {
            let pos_a = Vec2::new(0.0, 0.0);
            let pos_b = Vec2::new(1.5, 0.0);
            let shape_a = Shape::Circle(circle_a);
            let shape_b = Shape::Circle(circle_b);
            
            let collider_a = Collider::new(
                ColliderHandle::new(1, 0),
                BodyHandle::new(1, 0),
                shape_a
            );
            let collider_b = Collider::new(
                ColliderHandle::new(2, 0),
                BodyHandle::new(2, 0),
                shape_b
            );
            
            let transform_a = Transform::identity();
            let transform_b = Transform::new(pos_b, Rot2::identity());
            
            let narrow_phase = NarrowPhase::new(NarrowPhaseConfig::default());
            black_box(narrow_phase.collide(
                &collider_a, transform_a,
                &collider_b, transform_b
            ));
        });
    });
    
    group.bench_function("circle_polygon", |b| {
        let circle = Circle::new(1.0);
        let vertices = vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(-1.0, 1.0),
        ];
        let polygon = Polygon::new(vertices);
        
        b.iter(|| {
            let pos_a = Vec2::new(0.0, 0.0);
            let pos_b = Vec2::new(1.5, 0.0);
            let shape_a = Shape::Circle(circle);
            let shape_b = Shape::Polygon(polygon.clone());
            
            let collider_a = Collider::new(
                ColliderHandle::new(1, 0),
                BodyHandle::new(1, 0),
                shape_a
            );
            let collider_b = Collider::new(
                ColliderHandle::new(2, 0),
                BodyHandle::new(2, 0),
                shape_b
            );
            
            let transform_a = Transform::identity();
            let transform_b = Transform::new(pos_b, Rot2::identity());
            
            let narrow_phase = NarrowPhase::new(NarrowPhaseConfig::default());
            black_box(narrow_phase.collide(
                &collider_a, transform_a,
                &collider_b, transform_b
            ));
        });
    });
    
    group.bench_function("polygon_polygon", |b| {
        let vertices_a = vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(-1.0, 1.0),
        ];
        let vertices_b = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new(0.5, -0.5),
            Vec2::new(0.5, 0.5),
            Vec2::new(-0.5, 0.5),
        ];
        let polygon_a = Polygon::new(vertices_a);
        let polygon_b = Polygon::new(vertices_b);
        
        b.iter(|| {
            let pos_a = Vec2::new(0.0, 0.0);
            let pos_b = Vec2::new(1.5, 0.0);
            let shape_a = Shape::Polygon(polygon_a.clone());
            let shape_b = Shape::Polygon(polygon_b.clone());
            
            let collider_a = Collider::new(
                ColliderHandle::new(1, 0),
                BodyHandle::new(1, 0),
                shape_a
            );
            let collider_b = Collider::new(
                ColliderHandle::new(2, 0),
                BodyHandle::new(2, 0),
                shape_b
            );
            
            let transform_a = Transform::identity();
            let transform_b = Transform::new(pos_b, Rot2::identity());
            
            let narrow_phase = NarrowPhase::new(NarrowPhaseConfig::default());
            black_box(narrow_phase.collide(
                &collider_a, transform_a,
                &collider_b, transform_b
            ));
        });
    });
    
    group.finish();
}

criterion_group!(
    benches,
    stress_many_bodies,
    stress_high_velocity,
    stress_constraints,
    stress_polygons,
    bench_queries,
    bench_collision,
);

criterion_main!(benches);
