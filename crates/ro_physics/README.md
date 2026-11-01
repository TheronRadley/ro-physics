# ro_physics

A 2D rigid-body physics engine in Rust: circle and convex-polygon collision,
contact and joint constraints, continuous collision detection, sleeping, and
BVH-accelerated spatial queries.

```toml
[dependencies]
ro_physics = "0.1"
```

## Quick example

```rust
use ro_physics::prelude::*;

let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

// A static floor
let ground = world.create_static_body(Transform::identity());
world.add_box(ground, Vec2::new(10.0, 0.5), 0.0);

// A dynamic ball five metres up
let ball = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), Rot2::identity()));
world.add_circle(ball, 0.5, 1.0);

if let Some(body) = world.get_body_mut(ball) {
    body.set_linear_velocity(Vec2::new(1.0, 0.0));
}

// Fixed timestep, 60 Hz
for _ in 0..120 {
    world.step(1.0 / 60.0);
}

let body = world.get_body(ball).expect("the ball is still there");
assert!(body.position().y < 5.0);
```

## Conventions

| Topic          | Convention                                                                                   |
| -------------- | -------------------------------------------------------------------------------------------- |
| Coordinates    | Right-handed, **+Y is up**, gravity therefore points down: `Vec2::new(0.0, -9.81)`.           |
| Units          | SI: metres, kilograms, seconds. Densities are kg/m².                                          |
| Angles         | Radians, **counter-clockwise**. `Rot2` wraps its angle; `Transform` is `(Vec2, Rot2)`.         |
| Timestep       | `World::step(dt)` takes a fixed `dt` in seconds. Values below `MIN_TIMESTEP` (1e-5 s) and non-finite values are ignored; values above `MAX_TIMESTEP` (1/15 s) are clamped. |
| Tolerances     | `constants::EPSILON` is 1e-6 and is the engine's default comparison tolerance.                 |
| Ownership      | Bodies, colliders and constraints are owned by the `World` and referenced by handles.          |
| Handles        | `BodyHandle`, `ColliderHandle` and `ConstraintHandle` are `(id, generation)` pairs. A handle to a destroyed object is stale, not dangling: lookups return `None`. The null handle is `(0, 0)`. |
| Failure        | Constructors that can fail hand back a null handle or an `Err`; they do not panic.             |
| Determinism    | The same inputs, configuration and call order produce the same simulation, frame for frame.    |

## Defaults

| Config              | Default                                                                    |
| ------------------- | -------------------------------------------------------------------------- |
| Gravity             | `(0.0, -9.81)` m/s²                                                         |
| Solver              | 8 velocity iterations, 3 position iterations, Baumgarte 0.2, warm starting   |
| Sleeping            | enabled below 0.01 m/s and 0.01 rad/s for 0.5 s                             |
| CCD                 | enabled, 20 TOI iterations, 1e-4 tolerance                                  |
| Capacity limits     | 1024 bodies, 4096 colliders, 8192 contacts, 512 constraints                  |

## Features

| Feature         | Effect                                                                        |
| --------------- | ----------------------------------------------------------------------------- |
| `serialization` | Adds `serde` support plus JSON, RON and bincode helpers in `ro_physics::serialization`. Off by default. |

## Minimum supported Rust version

1.70.

## License

MIT OR Apache-2.0.
