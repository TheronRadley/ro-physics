# Ro-Physics - Capabilities

## Feature Status

| System | Status | Notes |
|--------|--------|-------|
| 2D Math | [x] Complete | Vec2, Rot2, Transform |
| Rigid Bodies | [x] Complete | Static, Dynamic, Kinematic |
| Circle Shape | [x] Complete | Collision, mass properties |
| Polygon Shape | [x] Complete | Convex, SAT collision |
| Contact Manifolds | [x] Complete | Multiple contacts, feature IDs |
| Broad-Phase Collision | [x] Complete | AABB tree |
| Narrow-Phase Collision | [x] Complete | SAT, closest point |
| Circle-Circle Collision | [x] Complete | Analytical solution |
| Circle-Polygon Collision | [x] Complete | Closest point algorithm |
| Polygon-Polygon Collision | [x] Complete | Full SAT |
| Impulse Solver | [x] Complete | Sequential impulse |
| Friction | [x] Complete | Coulomb model |
| Restitution | [x] Complete | Configurable per-collider |
| Constraints | [x] Complete | Distance, Revolute, Motor joints |
| Sleeping | [x] Complete | Automatic deactivation |
| CCD | [x] Complete | Tunneling prevention |
| Spatial Queries | [x] Complete | Point, AABB, Ray |
| BVH Acceleration | [x] Complete | Hierarchical spatial acceleration |
| Debug Renderer | [x] Complete | WGPU-based |
| Determinism | [x] Verified | Numerical determinism for identical inputs/configuration |
| Conservation Laws | [x] Verified | Momentum, energy |

## Performance

| Operation | Complexity | Notes |
|-----------|------------|-------|
| Broad-phase collision | O(n log n) | AABB tree |
| Narrow-phase collision | O(1) average | Shape pair specific |
| Query with BVH | Hierarchical spatial acceleration | Point, AABB, Ray |
| Solver | Iterative contact/constraint solving | For n contacts |
| Sleeping | O(1) per body | Automatic activation |

## Limitations

### Not Supported
- Concave polygons
- Compound shapes
- 3D
- Fluids
- Soft bodies
- Particles

### Current Constraints
- Convex polygons only
- Single shape per collider
- No sub-stepping for CCD
- No multithreading
- Basic sleeping system
- Cross-platform bitwise determinism is not currently guaranteed

## Validation

[x] Linear momentum conservation
[x] Angular momentum conservation
[x] Energy conservation (elastic)
[x] Energy dissipation (inelastic)
[x] Determinism
[x] Stress tests (1000+ bodies, no NaNs)
[x] Stability tests (stacking, high velocity)
