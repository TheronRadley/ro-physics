# Ro-Physics - Quick Start Guide

## Get Started in 5 Minutes

### 1. Add to Your Project

```bash
cd your_project
cargo add ro_physics
```

Or add to Cargo.toml:

```toml
[dependencies]
ro_physics = "0.1"
```

### 2. Create a Simple Simulation

```rust
use ro_physics::prelude::*;

fn main() {
    // Create a world with gravity
    let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

    // Create ground
    let ground = world.create_static_body(Transform::identity());
    world.add_box(ground, Vec2::new(10.0, 0.5), 0.0);

    // Create a ball
    let ball = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), 0.0));
    world.add_circle(ball, 0.5, 1.0);

    // Give it a push
    if let Some(body) = world.get_body_mut(ball) {
        body.set_linear_velocity(Vec2::new(5.0, 0.0));
    }

    // Step the simulation
    for _ in 0..60 {
        world.step(1.0 / 60.0);
        if let Some(body) = world.get_body(ball) {
            println!("Ball position: {:?}", body.position());
        }
    }
}
```

### 3. Run It

```bash
cargo run
```

---

## Try the Demo

```bash
cd crates/ro_demo
cargo run
```

### Demo Controls

| Key | Action |
|-----|--------|
| Space | Pause/Resume |
| R | Reload Scene |
| T | Single Step |
| 1-7 | Switch Scene |
| D | Toggle AABB Display |
| C | Toggle Collider Display |
| V | Toggle Velocity Display |
| S | Toggle Sleeping Display |
| + or = | Zoom In |
| - | Zoom Out |
| Up/Down/Left/Right Arrow | Pan Camera |

### Demo Scenes

1. Gravity - Falling objects
2. Stacking - Stacked boxes
3. Dominoes - Chain reaction
4. Bouncy - High restitution
5. Polygons - Various shapes
6. Ragdoll - Articulated body
7. Stress Lab - 150+ bodies stress test

---

## Learn More

### Core Concepts

```rust
// Create bodies
let static_body = world.create_static_body(transform);
let dynamic_body = world.create_dynamic_body(transform);
let kinematic_body = world.create_kinematic_body(transform);

// Add shapes
world.add_circle(body, radius, density);
world.add_box(body, half_extents, density);
world.add_polygon(body, vertices, density);

// Create joints
world.create_distance_joint(body_a, body_b, anchor_a, anchor_b);
world.create_revolute_joint(body_a, body_b, anchor);
world.create_motor_joint(body_a, body_b);

// Step simulation
world.step(dt);

// Query world
world.point_query(point);
world.aabb_query(aabb);
world.ray_cast(ray, max_distance);
```

### Configuration

```rust
let config = WorldConfig::new()
    .with_gravity(Vec2::new(0.0, -9.81))
    .with_solver_config(SolverConfig::new()
        .with_velocity_iterations(8)
        .with_position_iterations(3)
        .with_baumgarte_factor(0.2)
    )
    .with_ccd_config(CcdConfig::new()
        .with_enabled(true)
        .with_max_toi_iterations(20)
    );

let world = World::new(config);
```

---

## Need Help?

- Check the [README](README.md) for detailed documentation
- Look at the demo code in crates/ro_demo/
- Run tests with cargo test --all
- Check clippy with cargo clippy --all-targets -- -D warnings

---

## Next Steps

1. Try the demo
2. Read the README
3. Check the examples
4. Integrate into your project
5. Contribute back!

---

Enjoy building with Ro-Physics!
