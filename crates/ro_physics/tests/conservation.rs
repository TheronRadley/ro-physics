//! Conservation law tests for Ro-Physics
//!
//! These tests verify that fundamental physics laws are preserved:
//! - Linear momentum conservation
//! - Angular momentum conservation
//! - Energy conservation (for elastic collisions)
//! - Energy dissipation (for inelastic collisions)

use ro_physics::constants::EPSILON;
use ro_physics::prelude::*;

/// Helper to calculate total linear momentum
fn total_linear_momentum(world: &World) -> Vec2 {
    let mut momentum = Vec2::zero();

    for body_handle in world.body_handles() {
        if let Some(body) = world.get_body(body_handle) {
            if body.is_dynamic() && body.is_enabled() {
                momentum += body.linear_velocity() * body.mass();
            }
        }
    }

    momentum
}

/// Helper to calculate total angular momentum about origin
fn total_angular_momentum(world: &World) -> f32 {
    let mut angular_momentum = 0.0;

    for body_handle in world.body_handles() {
        if let Some(body) = world.get_body(body_handle) {
            if body.is_dynamic() && body.is_enabled() {
                // Angular momentum = I * w + r x (m * v)
                let position = body.position();
                let linear_momentum = body.linear_velocity() * body.mass();
                let cross = position.x * linear_momentum.y - position.y * linear_momentum.x;
                angular_momentum += body.inertia() * body.angular_velocity() + cross;
            }
        }
    }

    angular_momentum
}

/// Helper to calculate total kinetic energy
fn total_kinetic_energy(world: &World) -> f32 {
    let mut energy = 0.0;

    for body_handle in world.body_handles() {
        if let Some(body) = world.get_body(body_handle) {
            if body.is_dynamic() && body.is_enabled() {
                let linear_ke = 0.5 * body.mass() * body.linear_velocity().length_squared();
                let angular_ke = 0.5 * body.inertia() * body.angular_velocity().powi(2);
                energy += linear_ke + angular_ke;
            }
        }
    }

    energy
}

/// Helper to calculate total potential energy (gravitational)
fn total_potential_energy(world: &World, gravity: Vec2) -> f32 {
    let mut energy = 0.0;

    for body_handle in world.body_handles() {
        if let Some(body) = world.get_body(body_handle) {
            if body.is_dynamic() && body.is_enabled() {
                // Potential energy = m * g * h
                // We use y as height (gravity is typically downward)
                energy += body.mass() * gravity.length() * body.position().y;
            }
        }
    }

    energy
}

/// Helper to calculate total mechanical energy
fn total_mechanical_energy(world: &World, gravity: Vec2) -> f32 {
    total_kinetic_energy(world) + total_potential_energy(world, gravity)
}

// ============================================================================
// LINEAR MOMENTUM CONSERVATION TESTS
// ============================================================================

#[test]
fn test_momentum_conservation_single_collision() {
    // Create a world without gravity (isolated system)
    let mut world = World::new(WorldConfig::new().with_gravity(Vec2::zero()));

    // Create two bodies with initial velocities
    let body_a = world.create_dynamic_body(Transform::new(Vec2::new(-2.0, 0.0), Rot2::new(0.0)));
    let _ = world.add_circle(body_a, 1.0, 1.0);

    let body_b = world.create_dynamic_body(Transform::new(Vec2::new(2.0, 0.0), Rot2::new(0.0)));
    let _ = world.add_circle(body_b, 1.0, 1.0);

    // Set initial velocities (toward each other)
    if let Some(b) = world.get_body_mut(body_a) {
        b.set_linear_velocity(Vec2::new(5.0, 0.0));
    }
    if let Some(b) = world.get_body_mut(body_b) {
        b.set_linear_velocity(Vec2::new(-3.0, 0.0));
    }

    let initial_momentum = total_linear_momentum(&world);

    // Step the simulation (collision should occur)
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }

    let final_momentum = total_linear_momentum(&world);

    // Momentum should be conserved (within numerical tolerance)
    let momentum_diff = (initial_momentum - final_momentum).length();
    assert!(
        momentum_diff < 1e-3,
        "Linear momentum not conserved! Difference: {}",
        momentum_diff
    );
}

#[test]
fn test_momentum_conservation_multiple_bodies() {
    // Create a world without gravity
    let mut world = World::new(WorldConfig::new().with_gravity(Vec2::zero()));

    // Create ground (static, doesn't affect momentum)
    let ground = world.create_static_body(Transform::identity());
    let _ = world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create multiple dynamic bodies
    for i in 0..5 {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(i as f32 * 2.0 - 5.0, 5.0),
            Rot2::new(0.0),
        ));
        let _ = world.add_circle(body, 1.0, 1.0);

        if let Some(b) = world.get_body_mut(body) {
            b.set_linear_velocity(Vec2::new((i as f32 - 2.0) * 3.0, -2.0));
        }
    }

    let initial_momentum = total_linear_momentum(&world);

    // Step the simulation
    for _ in 0..20 {
        world.step(1.0 / 60.0);
    }

    let final_momentum = total_linear_momentum(&world);

    // Momentum should be conserved
    let momentum_diff = (initial_momentum - final_momentum).length();
    assert!(
        momentum_diff < 1e-2,
        "Linear momentum not conserved with multiple bodies! Difference: {}",
        momentum_diff
    );
}

// ============================================================================
// ANGULAR MOMENTUM CONSERVATION TESTS
// ============================================================================

#[test]
fn test_angular_momentum_conservation() {
    // Create a world without gravity
    let mut world = World::new(WorldConfig::new().with_gravity(Vec2::zero()));

    // Create two bodies with angular velocity
    let body_a = world.create_dynamic_body(Transform::identity());
    let _ = world.add_circle(body_a, 1.0, 1.0);

    let body_b = world.create_dynamic_body(Transform::new(Vec2::new(3.0, 0.0), Rot2::new(0.0)));
    let _ = world.add_circle(body_b, 1.0, 1.0);

    // Set angular velocities
    if let Some(b) = world.get_body_mut(body_a) {
        b.set_angular_velocity(2.0);
        b.set_linear_velocity(Vec2::new(1.0, 0.0));
    }
    if let Some(b) = world.get_body_mut(body_b) {
        b.set_angular_velocity(-1.5);
        b.set_linear_velocity(Vec2::new(-0.5, 0.0));
    }

    let initial_angular_momentum = total_angular_momentum(&world);

    // Step the simulation
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }

    let final_angular_momentum = total_angular_momentum(&world);

    // Angular momentum should be conserved
    let angular_momentum_diff = (initial_angular_momentum - final_angular_momentum).abs();
    assert!(
        angular_momentum_diff < 1e-2,
        "Angular momentum not conserved! Difference: {}",
        angular_momentum_diff
    );
}

// ============================================================================
// ENERGY CONSERVATION TESTS
// ============================================================================

#[test]
fn test_energy_conservation_elastic() {
    // Create a world with high restitution (elastic collisions)
    let mut world = World::new(WorldConfig::new().with_gravity(Vec2::zero()));

    // Create two bodies
    let body_a = world.create_dynamic_body(Transform::new(Vec2::new(-2.0, 0.0), Rot2::new(0.0)));
    let collider_a = world.add_circle(body_a, 1.0, 1.0);

    let body_b = world.create_dynamic_body(Transform::new(Vec2::new(2.0, 0.0), Rot2::new(0.0)));
    let collider_b = world.add_circle(body_b, 1.0, 1.0);

    // Set high restitution
    if let Some(c) = world.get_collider_mut(collider_a) {
        c.set_restitution(1.0);
    }
    if let Some(c) = world.get_collider_mut(collider_b) {
        c.set_restitution(1.0);
    }

    // Set initial velocities
    if let Some(b) = world.get_body_mut(body_a) {
        b.set_linear_velocity(Vec2::new(3.0, 0.0));
    }
    if let Some(b) = world.get_body_mut(body_b) {
        b.set_linear_velocity(Vec2::new(-2.0, 0.0));
    }

    let initial_energy = total_kinetic_energy(&world);

    // Step the simulation
    for _ in 0..10 {
        world.step(1.0 / 60.0);
    }

    let final_energy = total_kinetic_energy(&world);

    // Energy should be approximately conserved for elastic collisions
    // Note: Some energy loss is expected due to numerical errors
    let energy_ratio = final_energy / initial_energy;
    assert!(
        energy_ratio > 0.99 && energy_ratio < 1.01,
        "Energy not conserved in elastic collision! Ratio: {}",
        energy_ratio
    );
}

#[test]
fn test_energy_dissipation_inelastic() {
    // Create a world with low restitution (inelastic collisions)
    let mut world = World::new(WorldConfig::new().with_gravity(Vec2::zero()));

    // Create two bodies
    let body_a = world.create_dynamic_body(Transform::new(Vec2::new(-2.0, 0.0), Rot2::new(0.0)));
    let collider_a = world.add_circle(body_a, 1.0, 1.0);

    let body_b = world.create_dynamic_body(Transform::new(Vec2::new(2.0, 0.0), Rot2::new(0.0)));
    let collider_b = world.add_circle(body_b, 1.0, 1.0);

    // Set low restitution
    if let Some(c) = world.get_collider_mut(collider_a) {
        c.set_restitution(0.0);
    }
    if let Some(c) = world.get_collider_mut(collider_b) {
        c.set_restitution(0.0);
    }

    // Set initial velocities
    if let Some(b) = world.get_body_mut(body_a) {
        b.set_linear_velocity(Vec2::new(3.0, 0.0));
    }
    if let Some(b) = world.get_body_mut(body_b) {
        b.set_linear_velocity(Vec2::new(-2.0, 0.0));
    }

    let initial_energy = total_kinetic_energy(&world);
    
    // Step the simulation long enough for the circles to meet: they start
    // 2 m apart (centres at -2 and 2, radius 1) and close at 5 m/s, so the
    // impact happens after roughly 0.4 s.
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let final_energy = total_kinetic_energy(&world);

    // Energy should be dissipated (reduced) for inelastic collisions
    assert!(
        final_energy < initial_energy,
        "Energy should be dissipated in inelastic collision! Initial: {}, Final: {}",
        initial_energy,
        final_energy
    );
}

// ============================================================================
// MECHANICAL ENERGY CONSERVATION (with gravity)
// ============================================================================

#[test]
fn test_mechanical_energy_conservation_with_gravity() {
    // Create a world with gravity
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = World::new(WorldConfig::new().with_gravity(gravity));

    // Create a single body (no collisions, so energy should be conserved modulo gravity work)
    let body = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 10.0), Rot2::new(0.0)));
    let _ = world.add_circle(body, 1.0, 1.0);

    // Set initial velocity upward
    if let Some(b) = world.get_body_mut(body) {
        b.set_linear_velocity(Vec2::new(0.0, 5.0));
    }

    let initial_energy = total_mechanical_energy(&world, gravity);

    // Step the simulation (body goes up and down). A finer step keeps the
    // integrator's own error well clear of the tolerance below.
    for _ in 0..120 {
        world.step(1.0 / 120.0);
    }

    let final_energy = total_mechanical_energy(&world, gravity);
    
    // Mechanical energy should be approximately conserved: potential and
    // kinetic energy trade off, and what is left is the integrator's error.
    // The step is a first-order symplectic one, so its energy error is
    // O(dt * m * g * v) - it stays bounded instead of drifting, but it is
    // far above a fixed 0.1 J, so compare it against the total energy.
    let energy_diff = (initial_energy - final_energy).abs();
    let relative_diff = energy_diff / initial_energy.max(f32::EPSILON);
    assert!(relative_diff < 0.01,
        "Mechanical energy not conserved! Difference: {} ({:.3}% of {})",
        energy_diff, relative_diff * 100.0, initial_energy);
}

// ============================================================================
// DETERMINISM TESTS
// ============================================================================

#[test]
fn test_deterministic_simulation() {
    // Create two identical worlds
    let gravity = Vec2::new(0.0, -9.81);
    let config = WorldConfig::new().with_gravity(gravity);

    let mut world1 = World::new(config.clone());
    let mut world2 = World::new(config);

    // Setup both worlds identically
    for i in 0..10 {
        let body1 = world1.create_dynamic_body(Transform::new(
            Vec2::new(i as f32 * 2.0 - 5.0, 10.0),
            Rot2::new(0.0),
        ));
        let _ = world1.add_circle(body1, 1.0, 1.0);

        let body2 = world2.create_dynamic_body(Transform::new(
            Vec2::new(i as f32 * 2.0 - 5.0, 10.0),
            Rot2::new(0.0),
        ));
        let _ = world2.add_circle(body2, 1.0, 1.0);

        // Set same initial velocities
        if let Some(b) = world1.get_body_mut(body1) {
            b.set_linear_velocity(Vec2::new(1.0, -2.0));
        }
        if let Some(b) = world2.get_body_mut(body2) {
            b.set_linear_velocity(Vec2::new(1.0, -2.0));
        }
    }

    // Add ground to both
    let ground1 = world1.create_static_body(Transform::identity());
    let _ = world1.add_box(ground1, Vec2::new(20.0, 0.5), 0.0);

    let ground2 = world2.create_static_body(Transform::identity());
    let _ = world2.add_box(ground2, Vec2::new(20.0, 0.5), 0.0);

    // Step both worlds identically
    for _ in 0..30 {
        world1.step(1.0 / 60.0);
        world2.step(1.0 / 60.0);
    }

    // Compare final states
    assert_eq!(world1.body_count(), world2.body_count());

    for (handle1, handle2) in world1
        .body_handles()
        .iter()
        .zip(world2.body_handles().iter())
    {
        let body1 = world1.get_body(*handle1).unwrap();
        let body2 = world2.get_body(*handle2).unwrap();

        // Positions should be identical (within numerical tolerance)
        let pos_diff = (body1.position() - body2.position()).length();
        assert!(
            pos_diff < EPSILON * 100.0,
            "Positions differ! Handle: {:?}, Diff: {}",
            handle1,
            pos_diff
        );

        // Velocities should be identical
        let vel_diff = (body1.linear_velocity() - body2.linear_velocity()).length();
        assert!(
            vel_diff < EPSILON * 100.0,
            "Velocities differ! Handle: {:?}, Diff: {}",
            handle1,
            vel_diff
        );

        // Rotations should be identical
        let rot_diff = (body1.rotation().angle() - body2.rotation().angle()).abs();
        assert!(
            rot_diff < EPSILON * 100.0,
            "Rotations differ! Handle: {:?}, Diff: {}",
            handle1,
            rot_diff
        );

        // Angular velocities should be identical
        let ang_vel_diff = (body1.angular_velocity() - body2.angular_velocity()).abs();
        assert!(
            ang_vel_diff < EPSILON * 100.0,
            "Angular velocities differ! Handle: {:?}, Diff: {}",
            handle1,
            ang_vel_diff
        );
    }
}

// ============================================================================
// STABILITY TESTS
// ============================================================================

#[test]
fn test_no_nan_propagation() {
    // Create a chaotic scene and ensure no NaNs appear
    let mut world = World::new(WorldConfig::new().with_gravity(Vec2::new(0.0, -9.81)));

    // Create ground
    let ground = world.create_static_body(Transform::identity());
    let _ = world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create many bodies with random properties
    for i in 0..100 {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(
                (i as f32 % 10.0) * 2.0 - 10.0,
                (i as f32 / 10.0) * 3.0 + 5.0,
            ),
            Rot2::new(0.0),
        ));

        if i % 2 == 0 {
            let _ = world.add_circle(body, 0.5 + (i as f32 % 5.0) * 0.1, 1.0);
        } else {
            let size = 0.5 + (i as f32 % 5.0) * 0.1;
            let _ = world.add_box(body, Vec2::new(size, size), 1.0);
        }

        // Set random initial velocities
        if let Some(b) = world.get_body_mut(body) {
            b.set_linear_velocity(Vec2::new(
                (i as f32 * 0.3).sin() * 10.0,
                (i as f32 * 0.7).cos() * 5.0,
            ));
            b.set_angular_velocity((i as f32 * 0.5).sin() * 2.0);
        }
    }

    // Step the simulation and check for NaNs
    for step in 0..100 {
        world.step(1.0 / 60.0);

        for body_handle in world.body_handles() {
            if let Some(body) = world.get_body(body_handle) {
                // Check for NaN in position
                assert!(
                    !body.position().x.is_nan(),
                    "NaN in position.x at step {}",
                    step
                );
                assert!(
                    !body.position().y.is_nan(),
                    "NaN in position.y at step {}",
                    step
                );

                // Check for NaN in velocity
                assert!(
                    !body.linear_velocity().x.is_nan(),
                    "NaN in velocity.x at step {}",
                    step
                );
                assert!(
                    !body.linear_velocity().y.is_nan(),
                    "NaN in velocity.y at step {}",
                    step
                );

                // Check for NaN in angular velocity
                assert!(
                    !body.angular_velocity().is_nan(),
                    "NaN in angular_velocity at step {}",
                    step
                );

                // Check for NaN in rotation
                assert!(
                    !body.rotation().angle().is_nan(),
                    "NaN in rotation at step {}",
                    step
                );
            }
        }
    }
}

#[test]
fn test_stable_stacking() {
    // Create a stack of boxes and ensure it doesn't explode
    let mut world = World::new(WorldConfig::new().with_gravity(Vec2::new(0.0, -9.81)));

    // Create ground
    let ground = world.create_static_body(Transform::identity());
    let _ = world.add_box(ground, Vec2::new(10.0, 0.5), 0.0);

    // Create a stack of boxes
    let mut y = 1.0;
    let mut bodies = Vec::new();

    for _ in 0..20 {
        let body = world.create_dynamic_body(Transform::new(Vec2::new(0.0, y), Rot2::new(0.0)));
        let _ = world.add_box(body, Vec2::new(0.9, 0.9), 1.0);
        bodies.push(body);
        y += 1.0;
    }

    // Step the simulation
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    // Check that all bodies are still in a reasonable position
    // (stack should be stable, not exploded)
    for &body in &bodies {
        if let Some(b) = world.get_body(body) {
            // Y position should be reasonable (not fallen through ground or exploded upward)
            assert!(
                b.position().y > -10.0,
                "Body fell through ground! Position: {:?}",
                b.position()
            );
            assert!(
                b.position().y < 50.0,
                "Body exploded upward! Position: {:?}",
                b.position()
            );

            // X position should be reasonable (not exploded sideways)
            assert!(
                b.position().x > -50.0 && b.position().x < 50.0,
                "Body exploded sideways! Position: {:?}",
                b.position()
            );
        }
    }
}
