# Ro-Physics

A 2D rigid-body physics engine built from scratch in Rust.

[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE-MIT)

---

> Interactive WGPU demo available in `crates/ro_demo`.

*Stress Lab: Pyramid of 105 boxes, circles, polygons, ragdoll, and CCD projectiles*

---

## Features

Rust - Rigid Bodies - SAT - CCD - Constraints - BVH

[x] Rigid bodies - Static, dynamic, and kinematic
[x] Convex polygons - Full SAT collision detection
[x] Circles - Optimized collision algorithms
[x] Constraints - Distance, revolute, and motor joints
[x] Friction and restitution - Coulomb friction model
[x] Sleeping - Automatic body sleeping for performance
[x] Continuous collision detection - Tunneling prevention
[x] BVH spatial queries - Hierarchical spatial acceleration for point, AABB, and ray queries
[x] Fixed timestep simulation - Deterministic execution for identical inputs/configuration
[x] Debug visualization - Live rendering with WGPU

---

## Quick Example

```rust
use ro_physics::prelude::*;

// Create a world with gravity
let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

// Create a static ground body
let ground = world.create_static_body(Transform::identity());
world.add_box(ground, Vec2::new(10.0, 0.5), 0.0);

// Create a dynamic circle
let ball = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), 0.0));
world.add_circle(ball, 0.5, 1.0);

// Apply an initial velocity
if let Some(body) = world.get_body_mut(ball) {
    body.set_linear_velocity(Vec2::new(5.0, 0.0));
}

// Step the simulation
world.step(1.0 / 60.0);
```

---

## Demo

Run the interactive demo:

```bash
git clone https://github.com/YOUR_USERNAME/ro-physics
cd crates/ro_demo
cargo run
```

### Scenes

| Key | Scene | Description |
|-----|-------|-------------|
| 1 | Gravity | Falling objects under gravity |
| 2 | Stacking | Stacked boxes demonstrating stability |
| 3 | Dominoes | Chain reaction with dominoes |
| 4 | Bouncy | High restitution balls bouncing |
| 5 | Polygons | Various polygon shapes |
| 6 | Ragdoll | Articulated body with joints |
| 7 | Stress Lab | 150+ bodies stress test |

### Controls

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

---

## Performance

Criterion benchmarks are included for:

- Large body counts
- High-velocity CCD
- Constraint-heavy scenes
- Polygon collisions
- Spatial queries

Run them locally with:

```bash
cd crates/ro_physics
cargo bench
```

Performance results are hardware-dependent and will be published once benchmarks have been measured on a reproducible environment.

---

## Physics Validation

[x] Linear momentum conservation - Verified for isolated systems
[x] Angular momentum conservation - Verified about origin
[x] Energy conservation - Verified for elastic collisions
[x] Energy dissipation - Verified for inelastic collisions
[x] Determinism - Verified numerical determinism for identical inputs/configuration
[x] Stress tests - 100+ bodies, no NaNs, stable

---

## Installation

```toml
[dependencies]
ro_physics = "0.1"
```

Or from source:

```toml
[dependencies]
ro_physics = { path = "crates/ro_physics" }
```

---

## Capabilities

See [CAPABILITIES.md](CAPABILITIES.md) for detailed feature status and limitations.

---

## License

Dual-licensed under:
- [MIT License](LICENSE-MIT)
- [Apache License 2.0](LICENSE-APACHE)

---

Built from scratch. Validated mathematically. Stress-tested. Visualized in real time.
