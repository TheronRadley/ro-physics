//! Comprehensive tests for CCD module.

use super::*;
use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;
use crate::math::Rot2;
use crate::shapes::{Circle, Polygon, Shape};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::EPSILON;

    // ========== TOI Edge Cases ==========

    #[test]
    fn test_toi_circle_circle_identical_positions() {
        let toi = TimeOfImpact;

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        // Both circles at the same position
        let position_a = Vec2::zero();
        let velocity_a = Vec2::zero();
        let position_b = Vec2::zero();
        let velocity_b = Vec2::zero();

        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );

        // Already overlapping
        assert!(result.is_colliding());
        assert!(result.toi == 0.0);
    }

    #[test]
    fn test_toi_circle_circle_exact_contact() {
        let toi = TimeOfImpact;

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        // Circles exactly touching (distance = sum of radii)
        let position_a = Vec2::zero();
        let velocity_a = Vec2::zero();
        let position_b = Vec2::new(2.0, 0.0);
        let velocity_b = Vec2::zero();

        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );

        // Already touching - TOI should be 0
        assert!(result.is_colliding());
        assert!(result.toi == 0.0);
    }

    #[test]
    fn test_toi_circle_circle_parallel_motion() {
        let toi = TimeOfImpact;

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        // Circles moving parallel to each other (never collide)
        let position_a = Vec2::zero();
        let velocity_a = Vec2::new(1.0, 0.0);
        let position_b = Vec2::new(0.0, 3.0);
        let velocity_b = Vec2::new(1.0, 0.0);

        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );

        assert!(!result.is_colliding());
    }

    #[test]
    fn test_toi_circle_circle_perpendicular_motion() {
        let toi = TimeOfImpact;

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        // Circle A moving right, Circle B moving down
        let position_a = Vec2::zero();
        let velocity_a = Vec2::new(5.0, 0.0);
        let position_b = Vec2::new(0.0, 5.0);
        let velocity_b = Vec2::new(0.0, -5.0);

        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );

        assert!(result.is_colliding());
        assert!(result.toi > 0.0);
        assert!(result.toi < 1.0);
    }

    #[test]
    fn test_toi_circle_circle_grazing() {
        let toi = TimeOfImpact;

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        // Circles barely grazing each other
        let position_a = Vec2::zero();
        let velocity_a = Vec2::new(1.0, 0.0);
        let position_b = Vec2::new(1.99, 0.001);
        let velocity_b = Vec2::new(-1.0, 0.0);

        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );

        // Should detect the grazing collision
        assert!(result.is_colliding());
    }

    #[test]
    fn test_toi_circle_circle_different_radii() {
        let toi = TimeOfImpact;

        let circle_a = Circle::new(0.5);
        let circle_b = Circle::new(2.0);

        // Small circle inside large circle's radius
        let position_a = Vec2::zero();
        let velocity_a = Vec2::new(5.0, 0.0);
        let position_b = Vec2::new(3.0, 0.0);
        let velocity_b = Vec2::new(-5.0, 0.0);

        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );

        assert!(result.is_colliding());
        assert!(result.toi > 0.0);
    }

    #[test]
    fn test_toi_circle_circle_at_max_distance() {
        let toi = TimeOfImpact;

        let circle_a = Circle::new(1.0);
        let circle_b = Circle::new(1.0);

        // Circles very far apart moving toward each other
        let position_a = Vec2::zero();
        let velocity_a = Vec2::new(10.0, 0.0);
        let position_b = Vec2::new(100.0, 0.0);
        let velocity_b = Vec2::new(-10.0, 0.0);

        let result = toi.circle_circle(
            &circle_a, position_a, velocity_a,
            &circle_b, position_b, velocity_b,
            20, 1e-4
        );

        assert!(result.is_colliding());
        // TOI should be close to 5.0 (distance / relative speed = 100 / 20 = 5)
        assert!((result.toi - 5.0).abs() < 0.1);
    }

    // ========== Swept Collision Edge Cases ==========

    #[test]
    fn test_swept_collision_static_bodies() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let body_a = Body::new_static(handle_a);

        let handle_b = BodyHandle::new(2, 0);
        let body_b = Body::new_static(handle_b);

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        // Static bodies shouldn't collide in CCD
        let result = detector.check_swept_collision(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // No collision expected since both are static
        assert!(result.is_none());
    }

    #[test]
    fn test_swept_collision_one_static() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(10.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let body_b = Body::new_static(handle_b);

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // Should detect collision with static body
        assert!(result.is_some());
    }

    #[test]
    fn test_swept_collision_already_overlapping() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(1.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(0.5, 0.0));
        body_b.set_linear_velocity(Vec2::new(-1.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // Already overlapping - TOI should be 0
        assert!(result.is_some());
        assert!(result.unwrap().toi == 0.0);
    }

    #[test]
    fn test_swept_collision_exact_toi() {
        let detector = SweptCollisionDetector::new(
            CcdConfig::new().with_toi_tolerance(1e-6)
        );

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(10.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(20.0, 0.0));
        body_b.set_linear_velocity(Vec2::new(-10.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // Should collide at TOI = 0.9 (distance 18, relative speed 20, need to cover 18-2=16, 16/20=0.8)
        // Actually: distance between centers = 20, sum of radii = 2, so need to cover 18
        // Relative speed = 20, so TOI = 18/20 = 0.9
        assert!(result.is_some());
        let toi_result = result.unwrap();
        assert!((toi_result.toi - 0.9).abs() < 0.01);
    }

    // ========== Tunneling Detection Edge Cases ==========

    #[test]
    fn test_will_tunnel_slow_movement() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(1.0, 0.0)); // Slow

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(5.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let will_tunnel = detector.will_tunnel(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // Slow movement shouldn't tunnel
        assert!(!will_tunnel);
    }

    #[test]
    fn test_will_tunnel_very_fast() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(1000.0, 0.0)); // Very fast

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(5.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let will_tunnel = detector.will_tunnel(
            &body_a, &collider_a, &body_b, &collider_b, 1.0 / 60.0
        );

        // Very fast movement will tunnel
        assert!(will_tunnel);
    }

    #[test]
    fn test_will_tunnel_both_moving() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(50.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(10.0, 0.0));
        body_b.set_linear_velocity(Vec2::new(-50.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let will_tunnel = detector.will_tunnel(
            &body_a, &collider_a, &body_b, &collider_b, 1.0 / 60.0
        );

        // Both moving fast toward each other will tunnel
        assert!(will_tunnel);
    }

    #[test]
    fn test_will_tunnel_small_colliders() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(10.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(5.0, 0.0));

        // Very small colliders
        let shape_a = Shape::Circle(Circle::new(0.01));
        let shape_b = Shape::Circle(Circle::new(0.01));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let will_tunnel = detector.will_tunnel(
            &body_a, &collider_a, &body_b, &collider_b, 1.0 / 60.0
        );

        // Small colliders with moderate speed will tunnel
        assert!(will_tunnel);
    }

    // ========== CCD with Polygons ==========

    #[test]
    fn test_swept_collision_circle_polygon() {
        let detector = SweptCollisionDetector::new(
            CcdConfig::new().with_enabled(true)
        );

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(5.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(4.0, 0.0));

        let shape_a = Shape::Circle(Circle::new(1.0));
        
        // Create a square polygon
        let vertices = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new(0.5, -0.5),
            Vec2::new(0.5, 0.5),
            Vec2::new(-0.5, 0.5),
        ];
        let shape_b = Shape::Polygon(Polygon::new(vertices));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // Should detect collision
        assert!(result.is_some());
    }

    #[test]
    fn test_swept_collision_polygon_polygon() {
        let detector = SweptCollisionDetector::new(
            CcdConfig::new().with_enabled(true)
        );

        let handle_a = BodyHandle::new(1, 0);
        let mut body_a = Body::new_dynamic(handle_a);
        body_a.set_position(Vec2::zero());
        body_a.set_linear_velocity(Vec2::new(2.0, 0.0));

        let handle_b = BodyHandle::new(2, 0);
        let mut body_b = Body::new_dynamic(handle_b);
        body_b.set_position(Vec2::new(3.0, 0.0));
        body_b.set_linear_velocity(Vec2::new(-2.0, 0.0));

        // Create square polygons
        let vertices_a = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new(0.5, -0.5),
            Vec2::new(0.5, 0.5),
            Vec2::new(-0.5, 0.5),
        ];
        let shape_a = Shape::Polygon(Polygon::new(vertices_a));

        let vertices_b = vec![
            Vec2::new(-0.5, -0.5),
            Vec2::new(0.5, -0.5),
            Vec2::new(0.5, 0.5),
            Vec2::new(-0.5, 0.5),
        ];
        let shape_b = Shape::Polygon(Polygon::new(vertices_b));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // Should detect collision
        assert!(result.is_some());
    }

    // ========== CCD Configuration Tests ==========

    #[test]
    fn test_ccd_config_builder() {
        let config = CcdConfig::new()
            .with_enabled(true)
            .with_max_toi_iterations(50)
            .with_toi_tolerance(1e-6)
            .with_max_toi(0.8)
            .with_conservative_advancement(true);

        assert!(config.enabled);
        assert_eq!(config.max_toi_iterations, 50);
        assert!((config.toi_tolerance - 1e-6).abs() < EPSILON);
        assert!((config.max_toi - 0.8).abs() < EPSILON);
        assert!(config.conservative_advancement);
    }

    #[test]
    fn test_ccd_config_default() {
        let config = CcdConfig::default();
        assert!(config.enabled);
        assert_eq!(config.max_toi_iterations, 20);
        assert!((config.toi_tolerance - 1e-4).abs() < EPSILON);
        assert!((config.max_toi - 1.0).abs() < EPSILON);
        assert!(config.conservative_advancement);
    }

    // ========== CCD Result Tests ==========

    #[test]
    fn test_ccd_result_construction() {
        let result = CcdResult::new(true, 0.5, true, false);
        assert!(result.ccd_performed);
        assert!((result.toi - 0.5).abs() < EPSILON);
        assert!(result.will_collide);
        assert!(!result.would_tunnel);
    }

    #[test]
    fn test_ccd_result_no_ccd() {
        let result = CcdResult::no_ccd();
        assert!(!result.ccd_performed);
        assert!(!result.will_collide);
        assert!(!result.would_tunnel);
    }

    // ========== Swept Collision Detector Tests ==========

    #[test]
    fn test_swept_detector_construction() {
        let detector = SweptCollisionDetector::new(CcdConfig::default());
        assert!(detector.config().enabled);
    }

    #[test]
    fn test_swept_detector_disabled() {
        let detector = SweptCollisionDetector::new(
            CcdConfig::new().with_enabled(false)
        );

        let handle_a = BodyHandle::new(1, 0);
        let body_a = Body::new_dynamic(handle_a);

        let handle_b = BodyHandle::new(2, 0);
        let body_b = Body::new_dynamic(handle_b);

        let shape_a = Shape::Circle(Circle::new(1.0));
        let shape_b = Shape::Circle(Circle::new(1.0));

        let collider_a = Collider::new(ColliderHandle::new(1, 0), handle_a, shape_a);
        let collider_b = Collider::new(ColliderHandle::new(2, 0), handle_b, shape_b);

        let result = detector.check_swept_collision(
            &body_a, &collider_a, &body_b, &collider_b, 1.0
        );

        // Should return None when CCD is disabled
        assert!(result.is_none());
    }

    // ========== TOI Result Tests ==========

    #[test]
    fn test_toi_result_construction() {
        let result = ToiResult::new(0.5, true, 10);
        assert!((result.toi - 0.5).abs() < EPSILON);
        assert!(result.is_colliding());
        assert_eq!(result.iterations, 10);
    }

    #[test]
    fn test_toi_result_default() {
        let result = ToiResult::default();
        assert!((result.toi - 1.0).abs() < EPSILON);
        assert!(!result.is_colliding());
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn test_toi_result_clamped() {
        let result = ToiResult::new(1.5, true, 10);
        assert!((result.clamped_toi() - 1.0).abs() < EPSILON);

        let result2 = ToiResult::new(-0.5, true, 10);
        assert!((result2.clamped_toi() - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_toi_result_display() {
        let result = ToiResult::new(0.123456, true, 5);
        let result_str = format!("{}", result);
        assert!(result_str.contains("ToiResult"));
        assert!(result_str.contains("0.123456"));
        assert!(result_str.contains("true"));
        assert!(result_str.contains("5"));
    }
}
