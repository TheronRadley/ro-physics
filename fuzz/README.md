# Fuzzing `ro_physics`

The fuzz targets feed arbitrary bytes to the public API and look for panics,
overflow and non-finite values leaking into the simulation. They are kept out
of the workspace (see `exclude` in the root `Cargo.toml`) so the normal build
never needs a nightly toolchain.

## Running

```sh
cargo install cargo-fuzz
cargo +nightly fuzz run step          # random worlds, random steps
cargo +nightly fuzz run shapes        # random shapes and geometry queries
cargo +nightly fuzz run serialization # arbitrary bytes into the (de)serializers
```

## What each target covers

| Target          | Invariant under test                                              |
| --------------- | ----------------------------------------------------------------- |
| `step`          | Stepping a world never panics, never panics on a stale handle, and never produces a non-finite position/velocity. |
| `shapes`        | Shape construction, `aabb`, `closest_point` and mass properties are total functions of their inputs. |
| `serialization` | Deserializing arbitrary bytes returns `Result` instead of panicking. |

Invalid input is expected to be *refused* (a null handle, an `Err`, or the
previous value kept) rather than to panic or to poison the world with NaN.
