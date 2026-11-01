//! Comprehensive tests for the query module.

use super::*;
use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;
use crate::geometry::Aabb;
use crate::math::utils::approx_eq;
use crate::math::{Rot2, Transform, Vec2};
use crate::world::World;


// ========== BVH Tests ==========

#[test]
fn test_bvh_construction_empty() {
    let bvh = Bvh::new();
    assert!(bvh.is_empty());
    assert_eq!(bvh.node_count(), 0);
    assert_eq!(bvh.collider_count(), 0);
}

#[test]
fn test_bvh_build_single_collider() {
    let aabbs = vec![Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0))];
    let handles = vec![ColliderHandle::new(1, 0)];
    let body_handles = vec![BodyHandle::new(1, 0)];

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    assert!(!bvh.is_empty());
    assert!(bvh.node_count() >= 1);
    assert_eq!(bvh.collider_count(), 1);
}

#[test]
fn test_bvh_build_multiple_colliders() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(-1.0, -1.0)),
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(0.0, 0.0)),
        Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)),
        Aabb::new(Vec2::new(1.0, 1.0), Vec2::new(2.0, 2.0)),
    ];

    let handles: Vec<ColliderHandle> = (0..4)
        .map(|i| ColliderHandle::new(i as u64 + 1, 0))
        .collect();
    let body_handles: Vec<BodyHandle> = (0..4)
        .map(|i| BodyHandle::new(i as u64 + 1, 0))
        .collect();

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    assert!(!bvh.is_empty());
    assert!(bvh.node_count() >= 4);
    assert_eq!(bvh.collider_count(), 4);
}

#[test]
fn test_bvh_point_query_single_hit() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
        Aabb::new(Vec2::new(2.0, 2.0), Vec2::new(3.0, 3.0)),
    ];

    let handles = vec![
        ColliderHandle::new(1, 0),
        ColliderHandle::new(2, 0),
    ];
    let body_handles = vec![
        BodyHandle::new(1, 0),
        BodyHandle::new(2, 0),
    ];

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    // Query point inside first AABB
    let results = bvh.point_query(Vec2::new(0.0, 0.0));
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].0, BodyHandle::new(1, 0));
    assert_eq!(results[0].1, ColliderHandle::new(1, 0));

    // Query point inside second AABB
    let results = bvh.point_query(Vec2::new(2.5, 2.5));
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].0, BodyHandle::new(2, 0));
    assert_eq!(results[0].1, ColliderHandle::new(2, 0));
}

#[test]
fn test_bvh_point_query_multiple_hits() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(0.0, 0.0)),
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
        Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0)),
    ];

    let handles: Vec<ColliderHandle> = (0..3)
        .map(|i| ColliderHandle::new(i as u64 + 1, 0))
        .collect();
    let body_handles: Vec<BodyHandle> = (0..3)
        .map(|i| BodyHandle::new(i as u64 + 1, 0))
        .collect();

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    // Query point at origin - should hit all 3 AABBs
    let results = bvh.point_query(Vec2::new(0.0, 0.0));
    assert_eq!(results.len(), 3);
}

#[test]
fn test_bvh_point_query_no_hit() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
    ];

    let handles = vec![ColliderHandle::new(1, 0)];
    let body_handles = vec![BodyHandle::new(1, 0)];

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    // Query point outside AABB
    let results = bvh.point_query(Vec2::new(10.0, 10.0));
    assert_eq!(results.len(), 0);
}

#[test]
fn test_bvh_aabb_query_single_overlap() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
        Aabb::new(Vec2::new(2.0, 2.0), Vec2::new(3.0, 3.0)),
    ];

    let handles = vec![
        ColliderHandle::new(1, 0),
        ColliderHandle::new(2, 0),
    ];
    let body_handles = vec![
        BodyHandle::new(1, 0),
        BodyHandle::new(2, 0),
    ];

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    // Query AABB that overlaps first AABB
    let query_aabb = Aabb::new(Vec2::new(-0.5, -0.5), Vec2::new(0.5, 0.5));
    let results = bvh.aabb_query(query_aabb);
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].0, BodyHandle::new(1, 0));
    assert_eq!(results[0].1, ColliderHandle::new(1, 0));
}

#[test]
fn test_bvh_aabb_query_multiple_overlaps() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(0.0, 0.0)),
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
        Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0)),
    ];

    let handles: Vec<ColliderHandle> = (0..3)
        .map(|i| ColliderHandle::new(i as u64 + 1, 0))
        .collect();
    let body_handles: Vec<BodyHandle> = (0..3)
        .map(|i| BodyHandle::new(i as u64 + 1, 0))
        .collect();

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    // Query AABB that overlaps all 3
    let query_aabb = Aabb::new(Vec2::new(-1.5, -1.5), Vec2::new(1.5, 1.5));
    let results = bvh.aabb_query(query_aabb);
    assert_eq!(results.len(), 3);
}

#[test]
fn test_bvh_aabb_query_no_overlap() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
    ];

    let handles = vec![ColliderHandle::new(1, 0)];
    let body_handles = vec![BodyHandle::new(1, 0)];

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    // Query AABB far away
    let query_aabb = Aabb::new(Vec2::new(10.0, 10.0), Vec2::new(11.0, 11.0));
    let results = bvh.aabb_query(query_aabb);
    assert_eq!(results.len(), 0);
}

#[test]
fn test_bvh_ray_cast_simple() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
        Aabb::new(Vec2::new(2.0, 2.0), Vec2::new(3.0, 3.0)),
    ];

    let handles = vec![
        ColliderHandle::new(1, 0),
        ColliderHandle::new(2, 0),
    ];
    let body_handles = vec![
        BodyHandle::new(1, 0),
        BodyHandle::new(2, 0),
    ];

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);

    // Ray from left through first AABB
    let results = bvh.ray_cast_all(Vec2::new(-5.0, 0.0), Vec2::new(1.0, 0.0), 100.0);
    assert!(!results.is_empty());
    assert_eq!(results[0].0, BodyHandle::new(1, 0));
}

#[test]
fn test_bvh_clear() {
    let aabbs = vec![Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0))];
    let handles = vec![ColliderHandle::new(1, 0)];
    let body_handles = vec![BodyHandle::new(1, 0)];

    let mut bvh = Bvh::new();
    bvh.build(aabbs, handles, body_handles);
    assert!(!bvh.is_empty());

    bvh.clear();
    assert!(bvh.is_empty());
    assert_eq!(bvh.node_count(), 0);
    assert_eq!(bvh.collider_count(), 0);
}

#[test]
fn test_bvh_update_and_rebuild() {
    let aabbs = vec![
        Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
        Aabb::new(Vec2::new(2.0, 2.0), Vec2::new(3.0, 3.0)),
    ];

    let handles = vec![
        ColliderHandle::new(1, 0),
        ColliderHandle::new(2, 0),
    ];
    let body_handles = vec![
        BodyHandle::new(1, 0),
        BodyHandle::new(2, 0),
    ];

    let mut bvh = Bvh::new();
    bvh.build(aabbs.clone(), handles.clone(), body_handles.clone());

    // Update first collider
    bvh.update_collider(0, Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(-1.0, -1.0)));

    // Query should still work
    let results = bvh.point_query(Vec2::new(-1.5, -1.5));
    assert!(!results.is_empty());
}

// ========== Point Query Edge Cases ==========

#[test]
fn test_point_query_on_edge() {
    let mut world = World::default();

    // Create a box at origin with size 2x2
    let body = world.create_static_body(Transform::identity());
    let _ = world.add_box(body, Vec2::new(1.0, 1.0), 1.0);

    // Query point on the edge of the box
    let results = world.point_query(Vec2::new(1.0, 0.0));
    // Point on edge should be considered inside or on boundary
    assert!(!results.is_empty());
}

#[test]
fn test_point_query_on_vertex() {
    let mut world = World::default();

    // Create a box at origin with size 2x2
    let body = world.create_static_body(Transform::identity());
    let _ = world.add_box(body, Vec2::new(1.0, 1.0), 1.0);

    // Query point on the vertex of the box
    let results = world.point_query(Vec2::new(1.0, 1.0));
    // Point on vertex should be considered inside or on boundary
    assert!(!results.is_empty());
}

#[test]
fn test_point_query_empty_world() {
    let world = World::default();

    let results = world.point_query(Vec2::new(0.0, 0.0));
    assert_eq!(results.len(), 0);
}

#[test]
fn test_point_query_multiple_bodies() {
    let mut world = World::default();

    // Create multiple bodies
    let body1 = world.create_static_body(Transform::identity());
    let _ = world.add_circle(body1, 2.0, 1.0);

    let body2 = world.create_static_body(Transform::new(Vec2::new(5.0, 0.0), Rot2::identity()));
    let _ = world.add_circle(body2, 2.0, 1.0);

    // Query point at origin - should only hit first body
    let results = world.point_query(Vec2::new(0.0, 0.0));
    assert!(!results.is_empty());

    // Verify we got the right body
    let has_body1 = results.iter().any(|r| r.body_handle == body1);
    assert!(has_body1);
}

// ========== AABB Query Edge Cases ==========

#[test]
fn test_aabb_query_touching() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_box(body, Vec2::new(1.0, 1.0), 1.0);

    // Query AABB that touches but doesn't overlap
    let query_aabb = Aabb::new(Vec2::new(1.0, -1.0), Vec2::new(2.0, 1.0));
    let results = world.aabb_query(query_aabb);

    // Touching AABBs should be considered overlapping
    assert!(!results.is_empty());
}

#[test]
fn test_aabb_query_contains() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_box(body, Vec2::new(1.0, 1.0), 1.0);

    // Query AABB that contains the body's AABB
    let query_aabb = Aabb::new(Vec2::new(-10.0, -10.0), Vec2::new(10.0, 10.0));
    let results = world.aabb_query(query_aabb);

    assert!(!results.is_empty());
}

#[test]
fn test_aabb_query_contained() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_box(body, Vec2::new(10.0, 10.0), 1.0);

    // Query AABB that is inside the body's AABB
    let query_aabb = Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0));
    let results = world.aabb_query(query_aabb);

    assert!(!results.is_empty());
}

// ========== Ray Cast Edge Cases ==========

#[test]
fn test_ray_cast_tangent() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_circle(body, 2.0, 1.0);

    // Ray along the tangent line of the circle, starting beside it
    let ray = Ray::new(Vec2::new(-5.0, 2.0), Vec2::new(1.0, 0.0));
    let result = world.ray_cast(ray, 100.0);

    // Tangent ray should graze the circle at x = 0
    assert!(result.is_some());
    let hit = result.unwrap();
    assert!(hit.distance > 0.0);
    assert!(approx_eq(hit.distance, 5.0));
}

#[test]
fn test_ray_cast_from_inside() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_circle(body, 5.0, 1.0);

    // Ray starting inside the circle
    let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
    let result = world.ray_cast(ray, 100.0);

    // Should hit the circle boundary
    assert!(result.is_some());
    let hit = result.unwrap();
    assert!(hit.distance > 0.0);
}

#[test]
fn test_ray_cast_origin_at_center() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_circle(body, 2.0, 1.0);

    // Ray starting at circle center
    let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
    let result = world.ray_cast(ray, 100.0);

    assert!(result.is_some());
    let hit = result.unwrap();
    // Distance should be approximately the radius
    assert!((hit.distance - 2.0).abs() < 0.1);
}

#[test]
fn test_ray_cast_opposite_direction() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_circle(body, 2.0, 1.0);

    // Ray pointing away from circle
    let ray = Ray::new(Vec2::new(5.0, 0.0), Vec2::new(1.0, 0.0));
    let result = world.ray_cast(ray, 100.0);

    // Should not hit
    assert!(result.is_none());
}

#[test]
fn test_ray_cast_zero_length() {
    let mut world = World::default();

    let body = world.create_static_body(Transform::identity());
    let _ = world.add_circle(body, 2.0, 1.0);

    // Ray with zero length
    let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
    let result = world.ray_cast(ray, 0.0);

    // Should not hit with zero max distance
    assert!(result.is_none());
}

#[test]
fn test_ray_cast_all_empty() {
    let world = World::default();

    let ray = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0));
    let results = world.ray_cast_all(ray, 100.0);

    assert_eq!(results.len(), 0);
}

#[test]
fn test_ray_cast_all_multiple() {
    let mut world = World::default();

    // Create multiple bodies in a line
    for i in 0..5 {
        let body = world.create_static_body(Transform::new(
            Vec2::new(i as f32 * 3.0, 0.0),
            Rot2::identity(),
        ));
        let _ = world.add_circle(body, 1.0, 1.0);
    }

    // Ray along the line
    let ray = Ray::new(Vec2::new(-5.0, 0.0), Vec2::new(1.0, 0.0));
    let results = world.ray_cast_all(ray, 100.0);

    // Should hit all bodies
    assert!(results.len() >= 5);

    // Results should be sorted by distance
    for i in 1..results.len() {
        assert!(results[i - 1].distance <= results[i].distance);
    }
}

// ========== Polygon Query Edge Cases ==========

#[test]
fn test_point_query_polygon() {
    let mut world = World::default();

    // Create a triangle
    let vertices = vec![
        Vec2::new(-1.0, -1.0),
        Vec2::new(1.0, -1.0),
        Vec2::new(0.0, 1.0),
    ];
    let body = world.create_static_body(Transform::identity());
    let _ = world.add_polygon(body, vertices, 1.0);

    // Query point inside triangle
    let results = world.point_query(Vec2::new(0.0, 0.0));
    assert!(!results.is_empty());

    // Query point outside triangle
    let results = world.point_query(Vec2::new(2.0, 2.0));
    assert_eq!(results.len(), 0);
}

#[test]
fn test_ray_cast_polygon() {
    let mut world = World::default();

    // Create a square polygon
    let vertices = vec![
        Vec2::new(-1.0, -1.0),
        Vec2::new(1.0, -1.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(-1.0, 1.0),
    ];
    let body = world.create_static_body(Transform::identity());
    let _ = world.add_polygon(body, vertices, 1.0);

    // Ray through the square
    let ray = Ray::new(Vec2::new(-5.0, 0.0), Vec2::new(1.0, 0.0));
    let result = world.ray_cast(ray, 100.0);

    assert!(result.is_some());
    let hit = result.unwrap();
    assert!(hit.distance > 0.0);
}

#[test]
fn test_aabb_query_polygon() {
    let mut world = World::default();

    // Create a rotated square
    let vertices = vec![
        Vec2::new(-1.0, -1.0),
        Vec2::new(1.0, -1.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(-1.0, 1.0),
    ];
    let body = world.create_static_body(Transform::new(Vec2::zero(), Rot2::new(std::f32::consts::PI / 4.0)));
    let _ = world.add_polygon(body, vertices, 1.0);

    // Query AABB that overlaps
    let query_aabb = Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(2.0, 2.0));
    let results = world.aabb_query(query_aabb);

    assert!(!results.is_empty());
}

// ========== Query Options Tests ==========

#[test]
fn test_ray_cast_options_default() {
    let options = RayCastOptions::default();
    assert!(!options.ignore_sensors);
    assert!(!options.ignore_static);
    assert!(!options.ignore_dynamic);
    assert!(!options.ignore_kinematic);
    assert_eq!(options.mask, 0xFFFF_FFFF);
}

#[test]
fn test_ray_cast_options_builder() {
    let options = RayCastOptions::new()
        .with_ignore_sensors(true)
        .with_ignore_static(true)
        .with_ignore_dynamic(false)
        .with_ignore_kinematic(true)
        .with_mask(0xFF);

    assert!(options.ignore_sensors);
    assert!(options.ignore_static);
    assert!(!options.ignore_dynamic);
    assert!(options.ignore_kinematic);
    assert_eq!(options.mask, 0xFF);
}

// Moved here from `query/mod.rs`, which cannot declare two `tests` modules.

#[test]
fn test_point_query_result() {
    let result = PointQueryResult::new(
        BodyHandle::new(1, 0),
        ColliderHandle::new(1, 0),
        true,
        0.0,
    );

    assert_eq!(result.body_handle, BodyHandle::new(1, 0));
    assert_eq!(result.collider_handle, ColliderHandle::new(1, 0));
    assert!(result.is_inside);
}

#[test]
fn test_aabb_query_result() {
    let result = AabbQueryResult::new(
        BodyHandle::new(1, 0),
        ColliderHandle::new(1, 0),
    );

    assert_eq!(result.body_handle, BodyHandle::new(1, 0));
    assert_eq!(result.collider_handle, ColliderHandle::new(1, 0));
}

#[test]
fn test_ray_cast_result() {
    let result = RayCastResult::new(
        BodyHandle::new(1, 0),
        ColliderHandle::new(1, 0),
        Vec2::new(1.0, 2.0),
        Vec2::new(0.0, 1.0),
        5.0,
        0.5,
    );

    assert_eq!(result.body_handle, BodyHandle::new(1, 0));
    assert_eq!(result.hit_point, Vec2::new(1.0, 2.0));
    assert_eq!(result.distance, 5.0);
    assert_eq!(result.fraction, 0.5);
}

