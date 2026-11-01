//! World implementation for the physics engine.
//!
//! This module provides the [`World`] type: the container that owns bodies,
//! colliders and constraints, and drives the simulation forward.
//!
//! # Step pipeline
//!
//! Each call to [`World::step`] runs, in order:
//!
//! 1. timestep validation (non-finite, zero, negative or huge steps are
//!    rejected or clamped),
//! 2. broad-phase refresh from the current transforms,
//! 3. narrow-phase contact generation, with manifold persistence (warm
//!    starting) matched by [`ContactId`](crate::collision::ContactId),
//! 4. collision event generation (`Started` / `Persisted` / `Ended`),
//! 5. force integration (gravity, user forces, damping) into velocities,
//! 6. contact solving (sequential impulses) and joint solving,
//! 7. position integration, optionally split at the earliest time of impact
//!    when continuous collision detection is enabled,
//! 8. sleeping bookkeeping and event dispatch.

use std::collections::{HashMap, HashSet};
use std::fmt;

use super::WorldConfig;
use crate::bodies::{Body, BodyHandle, BodyType};
use crate::ccd::SweptCollisionDetector;
use crate::collision::{
    BroadPhase, BroadPhaseConfig, Collider, ColliderHandle, CollisionEvent, CollisionEventHandler,
    CollisionEventType, ContactManifold, NarrowPhase, NarrowPhaseConfig,
};
use crate::constants::{EPSILON, MAX_TIMESTEP, MIN_TIMESTEP};
use crate::constraints::{Constraint, ConstraintHandle, DistanceJoint, MotorJoint, RevoluteJoint};
use crate::geometry::{Aabb, Ray};
use crate::math::{Rot2, Transform, Vec2};
use crate::query::{AabbQueryResult, PointQueryResult, RayCastResult};
use crate::shapes::{Circle, Polygon, Shape};
use crate::solver::ContactSolver;

/// Maximum number of CCD sub-steps performed in a single simulation step.
const MAX_CCD_SUBSTEPS: usize = 4;

/// The physics world.
///
/// The world is the main container for bodies, colliders, and the simulation
/// state. It manages the physics simulation and provides the public API.
///
/// # Coordinate system
///
/// Positions and sizes are in metres, velocities in metres per second, angles
/// in radians and masses in kilograms. Gravity defaults to `(0, -9.81)`, i.e.
/// +Y is up.
pub struct World {
    /// Configuration
    config: WorldConfig,
    /// Broad-phase collision detection
    broad_phase: BroadPhase,
    /// Narrow-phase collision detection
    narrow_phase: NarrowPhase,
    /// Contact solver
    contact_solver: ContactSolver,
    /// Swept collision detector (CCD)
    swept_detector: SweptCollisionDetector,
    /// List of all bodies
    bodies: Vec<Option<Body>>,
    /// List of all colliders
    colliders: Vec<Option<Collider>>,
    /// List of all constraints
    constraints: Vec<Option<Box<dyn Constraint>>>,
    /// Map from body handles to body indices
    body_map: HashMap<BodyHandle, usize>,
    /// Map from collider handles to collider indices
    collider_map: HashMap<ColliderHandle, usize>,
    /// Map from constraint handles to constraint indices
    constraint_map: HashMap<ConstraintHandle, usize>,
    /// Body slots that can be reused
    free_body_slots: Vec<usize>,
    /// Collider slots that can be reused
    free_collider_slots: Vec<usize>,
    /// Constraint slots that can be reused
    free_constraint_slots: Vec<usize>,
    /// Colliders attached to each body
    body_colliders: HashMap<BodyHandle, Vec<ColliderHandle>>,
    /// Density used when each collider was created
    collider_densities: HashMap<ColliderHandle, f32>,
    /// Body handle generator
    body_handle_generator: crate::bodies::BodyHandleGenerator,
    /// Collider handle generator
    collider_handle_generator: crate::collision::ColliderHandleGenerator,
    /// Constraint handle generator
    constraint_handle_generator: crate::constraints::ConstraintHandleGenerator,
    /// Event handlers
    event_handlers: Vec<Box<dyn CollisionEventHandler>>,
    /// Simulation time
    time: f32,
    /// Frame counter
    frame_count: u64,
    /// Contact manifolds from the current step
    manifolds: Vec<ContactManifold>,
    /// Collider pairs that were touching after the previous step
    touching_pairs: HashSet<(ColliderHandle, ColliderHandle)>,
    /// Collision events produced by the current step
    events: Vec<CollisionEvent>,
}

impl World {
    /// Create a new physics world with the given configuration
    #[inline]
    #[must_use]
    pub fn new(config: WorldConfig) -> Self {
        Self {
            broad_phase: BroadPhase::new(BroadPhaseConfig::new(config.max_colliders)),
            narrow_phase: NarrowPhase::new(NarrowPhaseConfig::default()),
            contact_solver: ContactSolver::new(config.solver_config),
            swept_detector: SweptCollisionDetector::new(config.ccd_config),
            bodies: Vec::with_capacity(config.max_bodies),
            colliders: Vec::with_capacity(config.max_colliders),
            constraints: Vec::with_capacity(config.max_constraints),
            body_map: HashMap::new(),
            collider_map: HashMap::new(),
            constraint_map: HashMap::new(),
            free_body_slots: Vec::new(),
            free_collider_slots: Vec::new(),
            free_constraint_slots: Vec::new(),
            body_colliders: HashMap::new(),
            collider_densities: HashMap::new(),
            body_handle_generator: crate::bodies::BodyHandleGenerator::new(),
            collider_handle_generator: crate::collision::ColliderHandleGenerator::new(),
            constraint_handle_generator: crate::constraints::ConstraintHandleGenerator::new(),
            event_handlers: Vec::new(),
            time: 0.0,
            frame_count: 0,
            manifolds: Vec::with_capacity(config.max_contacts),
            touching_pairs: HashSet::new(),
            events: Vec::new(),
            config,
        }
    }

    /// Create a new physics world with custom gravity
    #[inline]
    #[must_use]
    pub fn with_gravity(gravity: Vec2) -> Self {
        Self::new(WorldConfig::default().with_gravity(gravity))
    }

    /// Get the world configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &WorldConfig {
        &self.config
    }

    /// Get the gravity vector
    #[inline]
    #[must_use]
    pub fn gravity(&self) -> Vec2 {
        self.config.gravity
    }

    /// Set the gravity vector
    #[inline]
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.config.gravity = gravity;
    }

    /// Get the current simulation time
    #[inline]
    #[must_use]
    pub fn time(&self) -> f32 {
        self.time
    }

    /// Get the current frame count
    #[inline]
    #[must_use]
    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Get the contact manifolds generated by the last step
    #[inline]
    #[must_use]
    pub fn manifolds(&self) -> &[ContactManifold] {
        &self.manifolds
    }

    /// Get the collision events generated by the last step
    #[inline]
    #[must_use]
    pub fn events(&self) -> &[CollisionEvent] {
        &self.events
    }

    // ------------------------------------------------------------------
    // Bodies
    // ------------------------------------------------------------------

    /// Create a new static body
    #[inline]
    #[must_use]
    pub fn create_static_body(&mut self, transform: Transform) -> BodyHandle {
        self.create_body(BodyType::Static, transform)
    }

    /// Create a new dynamic body
    #[inline]
    #[must_use]
    pub fn create_dynamic_body(&mut self, transform: Transform) -> BodyHandle {
        self.create_body(BodyType::Dynamic, transform)
    }

    /// Create a new kinematic body
    #[inline]
    #[must_use]
    pub fn create_kinematic_body(&mut self, transform: Transform) -> BodyHandle {
        self.create_body(BodyType::Kinematic, transform)
    }

    /// Create a new body with the given type and transform
    ///
    /// Returns a null handle when the world is at capacity.
    #[inline]
    #[must_use]
    pub fn create_body(&mut self, body_type: BodyType, transform: Transform) -> BodyHandle {
        let index = match self.free_body_slots.pop() {
            Some(index) => index,
            None => {
                if self.bodies.len() >= self.config.max_bodies {
                    return BodyHandle::null();
                }
                self.bodies.push(None);
                self.bodies.len() - 1
            }
        };

        let handle = self.body_handle_generator.generate();

        let mut body = match body_type {
            BodyType::Static => Body::new_static(handle),
            BodyType::Dynamic => Body::new_dynamic(handle),
            BodyType::Kinematic => Body::new_kinematic(handle),
        };
        body.set_transform(transform);

        self.bodies[index] = Some(body);
        self.body_map.insert(handle, index);
        self.body_colliders.insert(handle, Vec::new());

        handle
    }

    /// Destroy a body and everything attached to it
    ///
    /// Colliders owned by the body and constraints referencing it are removed
    /// as well, so no dangling handles are left behind.
    #[inline]
    pub fn destroy_body(&mut self, handle: BodyHandle) -> bool {
        let Some(index) = self.body_map.remove(&handle) else {
            return false;
        };

        // Destroy the colliders attached to this body first
        let colliders = self.body_colliders.remove(&handle).unwrap_or_default();
        for collider in colliders {
            self.destroy_collider(collider);
        }

        // Destroy every constraint that references the body
        let stale: Vec<ConstraintHandle> = self
            .constraint_map
            .iter()
            .filter(|(_, &i)| {
                self.constraints
                    .get(i)
                    .and_then(|c| c.as_ref())
                    .is_some_and(|c| c.body_a() == handle || c.body_b() == handle)
            })
            .map(|(handle, _)| *handle)
            .collect();

        for constraint in stale {
            self.destroy_constraint(constraint);
        }

        if let Some(slot) = self.bodies.get_mut(index) {
            *slot = None;
        }
        self.free_body_slots.push(index);

        true
    }

    /// Get a body by handle
    #[inline]
    #[must_use]
    pub fn get_body(&self, handle: BodyHandle) -> Option<&Body> {
        let index = *self.body_map.get(&handle)?;
        self.bodies.get(index)?.as_ref()
    }

    /// Get a body by handle mutably
    #[inline]
    #[must_use]
    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Option<&mut Body> {
        let index = *self.body_map.get(&handle)?;
        self.bodies.get_mut(index)?.as_mut()
    }

    /// Check if a body exists
    #[inline]
    #[must_use]
    pub fn has_body(&self, handle: BodyHandle) -> bool {
        self.body_map.contains_key(&handle)
    }

    /// Get the number of bodies
    #[inline]
    #[must_use]
    pub fn body_count(&self) -> usize {
        self.body_map.len()
    }

    /// Get all body handles
    ///
    /// Handles come back in body-slot order, which is stable across runs and
    /// across worlds built the same way: iterating the handle map directly
    /// would leak its (randomly seeded) iteration order into everything that
    /// walks the list, and that is enough to make two identical simulations
    /// diverge.
    #[inline]
    #[must_use]
    pub fn body_handles(&self) -> Vec<BodyHandle> {
        self.bodies
            .iter()
            .flatten()
            .map(Body::handle)
            .collect()
    }

    // ------------------------------------------------------------------
    // Colliders
    // ------------------------------------------------------------------

    /// Create a new collider attached to a body
    #[inline]
    #[must_use]
    pub fn create_collider(&mut self, body_handle: BodyHandle, shape: Shape) -> ColliderHandle {
        if !self.has_body(body_handle) || !shape.is_valid() {
            return ColliderHandle::null();
        }

        let index = match self.free_collider_slots.pop() {
            Some(index) => index,
            None => {
                if self.colliders.len() >= self.config.max_colliders {
                    return ColliderHandle::null();
                }
                self.colliders.push(None);
                self.colliders.len() - 1
            }
        };

        let handle = self.collider_handle_generator.generate();
        let collider = Collider::new(handle, body_handle, shape);

        self.colliders[index] = Some(collider);
        self.collider_map.insert(handle, index);
        self.body_colliders
            .entry(body_handle)
            .or_default()
            .push(handle);

        self.refresh_collider_in_broad_phase(handle);

        handle
    }

    /// Destroy a collider
    #[inline]
    pub fn destroy_collider(&mut self, handle: ColliderHandle) -> bool {
        let Some(index) = self.collider_map.remove(&handle) else {
            return false;
        };

        let body_handle = self.colliders[index].as_ref().map(Collider::body_handle);

        self.broad_phase.remove_collider(handle);
        self.colliders[index] = None;
        self.collider_densities.remove(&handle);
        self.free_collider_slots.push(index);

        if let Some(body_handle) = body_handle {
            if let Some(colliders) = self.body_colliders.get_mut(&body_handle) {
                colliders.retain(|existing| *existing != handle);
            }
        }

        true
    }

    /// Get a collider by handle
    #[inline]
    #[must_use]
    pub fn get_collider(&self, handle: ColliderHandle) -> Option<&Collider> {
        let index = *self.collider_map.get(&handle)?;
        self.colliders.get(index)?.as_ref()
    }

    /// Get a collider by handle mutably
    ///
    /// The broad-phase entry of every collider is refreshed at the start of
    /// each [`World::step`], so shapes and local transforms edited through this
    /// accessor are picked up on the next step. Call
    /// [`World::refresh_collider_in_broad_phase`] when a query needs to see the
    /// change immediately.
    #[inline]
    #[must_use]
    pub fn get_collider_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        // Make sure the caller sees a valid index before handing out the borrow
        let index = *self.collider_map.get(&handle)?;
        let collider = self.colliders.get_mut(index)?.as_mut()?;
        Some(collider)
    }

    /// Check if a collider exists
    #[inline]
    #[must_use]
    pub fn has_collider(&self, handle: ColliderHandle) -> bool {
        self.collider_map.contains_key(&handle)
    }

    /// Get the number of colliders
    #[inline]
    #[must_use]
    pub fn collider_count(&self) -> usize {
        self.collider_map.len()
    }

    /// Get all collider handles
    ///
    /// Like [`World::body_handles`], the order is the stable slot order.
    #[inline]
    #[must_use]
    pub fn collider_handles(&self) -> Vec<ColliderHandle> {
        self.colliders
            .iter()
            .flatten()
            .map(Collider::handle)
            .collect()
    }

    /// Refresh the broad-phase entry of a collider
    ///
    /// Must be called after the collider's shape, local transform or owning
    /// body transform changes, otherwise the spatial index goes stale.
    #[inline]
    pub fn refresh_collider_in_broad_phase(&mut self, handle: ColliderHandle) {
        let Some(aabb) = self.collider_aabb(handle) else {
            return;
        };

        if self.broad_phase.get_aabb(handle).is_some() {
            self.broad_phase.update_collider(handle, aabb);
        } else {
            let index = self.collider_map.get(&handle).copied().unwrap_or(0);
            self.broad_phase.add_collider(handle, aabb, index);
        }
    }

    /// Add a circle shape to a body
    ///
    /// `density` is in kg/m²; a non-positive density leaves the body's mass
    /// properties untouched, which is what static bodies want.
    #[inline]
    #[must_use]
    pub fn add_circle(
        &mut self,
        body_handle: BodyHandle,
        radius: f32,
        density: f32,
    ) -> ColliderHandle {
        let Ok(circle) = Circle::try_new(radius) else {
            return ColliderHandle::null();
        };

        self.add_shape(body_handle, Shape::Circle(circle), density)
    }

    /// Add a box shape to a body from its half extents
    #[inline]
    #[must_use]
    pub fn add_box(
        &mut self,
        body_handle: BodyHandle,
        half_extents: Vec2,
        density: f32,
    ) -> ColliderHandle {
        let Ok(polygon) = Polygon::try_new(box_vertices(half_extents)) else {
            return ColliderHandle::null();
        };

        self.add_shape(body_handle, Shape::Polygon(polygon), density)
    }

    /// Add a polygon shape to a body
    #[inline]
    #[must_use]
    pub fn add_polygon(
        &mut self,
        body_handle: BodyHandle,
        vertices: Vec<Vec2>,
        density: f32,
    ) -> ColliderHandle {
        let Ok(polygon) = Polygon::try_new(vertices) else {
            return ColliderHandle::null();
        };

        self.add_shape(body_handle, Shape::Polygon(polygon), density)
    }

    /// Attach a shape to a body and update its mass properties
    #[inline]
    #[must_use]
    fn add_shape(&mut self, body_handle: BodyHandle, shape: Shape, density: f32) -> ColliderHandle {
        let handle = self.create_collider(body_handle, shape);
        if handle.is_null() {
            return handle;
        }

        self.collider_densities.insert(handle, density);
        self.update_body_mass_properties(body_handle);

        handle
    }

    /// Recompute a body's mass, inertia and centre of mass from its colliders
    ///
    /// Mass and inertia are summed over all colliders with a positive density
    /// and inertia is shifted with the parallel-axis theorem, so a body with
    /// several colliders behaves like a single rigid assembly. Static and
    /// kinematic bodies always keep zero mass.
    fn update_body_mass_properties(&mut self, body_handle: BodyHandle) {
        let Some(body) = self.get_body(body_handle) else {
            return;
        };

        if !body.is_dynamic() {
            return;
        }

        let colliders = self
            .body_colliders
            .get(&body_handle)
            .cloned()
            .unwrap_or_default();

        let mut mass = 0.0_f32;
        let mut inertia = 0.0_f32;
        let mut center = Vec2::zero();

        for handle in &colliders {
            let Some(collider) = self.get_collider(*handle) else {
                continue;
            };
            let density = self.collider_densities.get(handle).copied().unwrap_or(0.0);
            if density <= 0.0 {
                continue;
            }

            let properties = collider.shape().mass_properties(density);
            if properties.mass <= 0.0 {
                continue;
            }

            mass += properties.mass;
            center += properties.center_of_mass * properties.mass;
        }

        if mass <= 0.0 {
            return;
        }
        center /= mass;

        for handle in &colliders {
            let Some(collider) = self.get_collider(*handle) else {
                continue;
            };
            let density = self.collider_densities.get(handle).copied().unwrap_or(0.0);
            if density <= 0.0 {
                continue;
            }

            let properties = collider.shape().mass_properties(density);
            if properties.mass <= 0.0 {
                continue;
            }

            let offset = properties.center_of_mass - center;
            inertia += properties.inertia + properties.mass * offset.length_squared();
        }

        if let Some(body) = self.get_body_mut(body_handle) {
            body.set_mass_properties(mass, inertia.max(EPSILON));
        }
    }

    // ------------------------------------------------------------------
    // Constraints
    // ------------------------------------------------------------------

    /// Create a distance joint between two bodies
    #[inline]
    #[must_use]
    pub fn create_distance_joint(
        &mut self,
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor_a: Vec2,
        anchor_b: Vec2,
    ) -> ConstraintHandle {
        if !self.has_body(body_a) || !self.has_body(body_b) || body_a == body_b {
            return ConstraintHandle::null();
        }

        let Some(index) = self.allocate_constraint_slot() else {
            return ConstraintHandle::null();
        };

        let handle = self.constraint_handle_generator.generate();
        let joint = DistanceJoint::new(handle, body_a, body_b, anchor_a, anchor_b);

        self.constraints[index] = Some(Box::new(joint));
        self.constraint_map.insert(handle, index);

        handle
    }

    /// Create a revolute joint between two bodies
    #[inline]
    #[must_use]
    pub fn create_revolute_joint(
        &mut self,
        body_a: BodyHandle,
        body_b: BodyHandle,
        anchor: Vec2,
    ) -> ConstraintHandle {
        if !self.has_body(body_a) || !self.has_body(body_b) || body_a == body_b {
            return ConstraintHandle::null();
        }

        let Some(index) = self.allocate_constraint_slot() else {
            return ConstraintHandle::null();
        };

        let handle = self.constraint_handle_generator.generate();
        let joint = RevoluteJoint::new(handle, body_a, body_b, anchor);

        self.constraints[index] = Some(Box::new(joint));
        self.constraint_map.insert(handle, index);

        handle
    }

    /// Create a motor joint between two bodies
    #[inline]
    #[must_use]
    pub fn create_motor_joint(
        &mut self,
        body_a: BodyHandle,
        body_b: BodyHandle,
    ) -> ConstraintHandle {
        if !self.has_body(body_a) || !self.has_body(body_b) || body_a == body_b {
            return ConstraintHandle::null();
        }

        let Some(index) = self.allocate_constraint_slot() else {
            return ConstraintHandle::null();
        };

        let handle = self.constraint_handle_generator.generate();
        let joint = MotorJoint::new(handle, body_a, body_b);

        self.constraints[index] = Some(Box::new(joint));
        self.constraint_map.insert(handle, index);

        handle
    }

    /// Reserve a slot for a new constraint
    fn allocate_constraint_slot(&mut self) -> Option<usize> {
        if let Some(index) = self.free_constraint_slots.pop() {
            return Some(index);
        }

        if self.constraints.len() >= self.config.max_constraints {
            return None;
        }

        self.constraints.push(None);
        Some(self.constraints.len() - 1)
    }

    /// Destroy a constraint
    #[inline]
    pub fn destroy_constraint(&mut self, handle: ConstraintHandle) -> bool {
        let Some(index) = self.constraint_map.remove(&handle) else {
            return false;
        };

        self.constraints[index] = None;
        self.free_constraint_slots.push(index);

        true
    }

    /// Get a constraint by handle
    #[inline]
    #[must_use]
    pub fn get_constraint(&self, handle: ConstraintHandle) -> Option<&dyn Constraint> {
        let index = *self.constraint_map.get(&handle)?;
        self.constraints.get(index)?.as_ref().map(|c| &**c)
    }

    /// Get a constraint by handle mutably
    #[inline]
    #[must_use]
    pub fn get_constraint_mut(&mut self, handle: ConstraintHandle) -> Option<&mut dyn Constraint> {
        let index = *self.constraint_map.get(&handle)?;
        let constraint = self.constraints.get_mut(index)?.as_mut()?;

        Some(&mut **constraint)
    }

    /// Check if a constraint exists
    #[inline]
    #[must_use]
    pub fn has_constraint(&self, handle: ConstraintHandle) -> bool {
        self.constraint_map.contains_key(&handle)
    }

    /// Get the number of constraints
    #[inline]
    #[must_use]
    pub fn constraint_count(&self) -> usize {
        self.constraint_map.len()
    }

    /// Get all constraint handles
    ///
    /// Like [`World::body_handles`], the order is the stable slot order.
    #[inline]
    #[must_use]
    pub fn constraint_handles(&self) -> Vec<ConstraintHandle> {
        self.constraints
            .iter()
            .flatten()
            .map(|constraint| constraint.handle())
            .collect()
    }

    // ------------------------------------------------------------------
    // Queries
    // ------------------------------------------------------------------

    /// Perform a point query to find colliders containing a point
    #[inline]
    #[must_use]
    pub fn point_query(&self, point: Vec2) -> Vec<PointQueryResult> {
        if !point.is_finite() {
            return Vec::new();
        }

        let mut results = Vec::new();

        for handle in self.broad_phase.query_point(point) {
            let Some(collider) = self.get_collider(handle) else {
                continue;
            };
            let Some(body) = self.get_body(collider.body_handle()) else {
                continue;
            };

            let transform = collider.world_transform(body.transform());
            let shape = collider.shape();
            let inside = shape.contains_point(point, transform);
            let distance = if inside {
                0.0
            } else {
                (point - shape.closest_point(point, transform)).length()
            };

            results.push(PointQueryResult::new(
                body.handle(),
                handle,
                inside,
                distance,
            ));
        }

        results
    }

    /// Perform an AABB query to find colliders overlapping an AABB
    #[inline]
    #[must_use]
    pub fn aabb_query(&self, aabb: Aabb) -> Vec<AabbQueryResult> {
        if !aabb.is_valid() {
            return Vec::new();
        }

        let mut results = Vec::new();

        for handle in self.broad_phase.query_aabb(aabb) {
            let Some(collider) = self.get_collider(handle) else {
                continue;
            };
            let Some(body) = self.get_body(collider.body_handle()) else {
                continue;
            };

            if collider.aabb(body.transform()).overlaps(aabb) {
                results.push(AabbQueryResult::new(body.handle(), handle));
            }
        }

        results
    }

    /// Perform a ray cast and return the closest hit
    ///
    /// A degenerate (zero length or non-finite) ray, or a non-positive maximum
    /// distance, yields no hit instead of panicking.
    #[inline]
    #[must_use]
    pub fn ray_cast(&self, ray: Ray, max_distance: f32) -> Option<RayCastResult> {
        if !ray.is_finite() || ray.direction.length_squared() <= 0.0 || max_distance <= 0.0 {
            return None;
        }

        let mut closest: Option<RayCastResult> = None;

        for hit in self.ray_cast_all(ray, max_distance) {
            let better = closest
                .as_ref()
                .map_or(true, |best| hit.distance < best.distance);
            if better {
                closest = Some(hit);
            }
        }

        closest
    }

    /// Perform a ray cast and return all hits sorted by distance
    #[inline]
    #[must_use]
    pub fn ray_cast_all(&self, ray: Ray, max_distance: f32) -> Vec<RayCastResult> {
        if !ray.is_finite() || ray.direction.length_squared() <= 0.0 || max_distance <= 0.0 {
            return Vec::new();
        }

        let mut hits = Vec::new();

        // Ask the broad-phase for candidates instead of scanning every collider
        let candidates = self
            .broad_phase
            .query_ray(ray.origin, ray.direction, max_distance);

        for (handle, _) in candidates {
            let Some(collider) = self.get_collider(handle) else {
                continue;
            };
            let Some(body) = self.get_body(collider.body_handle()) else {
                continue;
            };

            let transform = collider.world_transform(body.transform());

            if let Some(hit) = ray_cast_shape(
                ray,
                max_distance,
                collider.shape(),
                transform,
                handle,
                body.handle(),
            ) {
                hits.push(hit);
            }
        }

        // `total_cmp` never panics, unlike `partial_cmp(..).unwrap()`
        hits.sort_by(|a, b| a.distance.total_cmp(&b.distance));
        hits
    }

    // ------------------------------------------------------------------
    // Simulation
    // ------------------------------------------------------------------

    /// Step the simulation forward by `dt` seconds
    ///
    /// The timestep is validated first: non-finite, zero and negative values
    /// are ignored, and very large steps are clamped so the solver cannot
    /// explode. Values below [`MIN_TIMESTEP`] are also ignored.
    #[inline]
    pub fn step(&mut self, dt: f32) {
        let dt = self.validate_timestep(dt);
        if dt <= 0.0 {
            return;
        }

        self.events.clear();

        // 1. Keep the spatial index in sync with the current transforms
        self.update_broad_phase();

        // 2. Wake sleeping bodies that are touched by moving ones
        if self.config.enable_sleeping {
            self.wake_islands();
        }

        // 3. Contact generation (with warm starting from the previous step)
        let manifolds = self.detect_contacts(true);

        // 4. Forces -> velocities
        self.integrate_velocities(dt);

        // 5. Solve contacts and joints
        self.manifolds = manifolds;
        self.solve_contacts(dt);
        self.solve_constraints(dt);

        // 6. Positions (splitting the step when CCD finds an impact)
        if self.config.ccd_config.enabled {
            self.integrate_positions_with_ccd(dt);
        } else {
            self.integrate_positions(dt);
        }

        // 7. Sleeping
        if self.config.enable_sleeping {
            self.update_sleeping(dt);
        }

        self.time += dt;
        self.frame_count += 1;

        // 8. Dispatch events
        self.notify_event_handlers();
    }

    /// Validate and clamp a timestep, returning 0 when the step must be skipped
    #[inline]
    fn validate_timestep(&self, dt: f32) -> f32 {
        // Non-finite timesteps (NaN, infinity) are ignored outright: they would
        // propagate into every body and destroy the simulation.
        if !dt.is_finite() || dt < MIN_TIMESTEP {
            return 0.0;
        }

        dt.min(MAX_TIMESTEP)
    }

    /// Refresh every collider's entry in the broad-phase
    fn update_broad_phase(&mut self) {
        let handles: Vec<ColliderHandle> = self.collider_map.keys().copied().collect();

        for handle in handles {
            if let Some(aabb) = self.collider_aabb(handle) {
                self.broad_phase.update_collider(handle, aabb);
            }
        }
    }

    /// The world-space AABB of a collider, if it can be computed
    #[inline]
    fn collider_aabb(&self, handle: ColliderHandle) -> Option<Aabb> {
        let collider = self.get_collider(handle)?;
        let body = self.get_body(collider.body_handle())?;

        let transform = collider.world_transform(body.transform());
        let aabb = collider.shape().aabb(transform);

        aabb.is_valid().then_some(aabb)
    }

    /// Run broad-phase + narrow-phase and return the new manifolds
    ///
    /// Manifold impulses are carried over from the previous step by matching
    /// contact ids, which is what makes warm starting effective. Events are
    /// emitted when `emit_events` is set (skipped for CCD sub-steps).
    fn detect_contacts(&mut self, emit_events: bool) -> Vec<ContactManifold> {
        let pairs = self.broad_phase.find_collision_pairs();
        let previous = std::mem::take(&mut self.manifolds);

        let mut manifolds: Vec<ContactManifold> =
            Vec::with_capacity(pairs.len().min(self.config.max_contacts));
        let mut touching: HashSet<(ColliderHandle, ColliderHandle)> =
            HashSet::with_capacity(pairs.len());

        for (handle_a, handle_b) in pairs {
            if manifolds.len() >= self.config.max_contacts {
                break;
            }

            let Some(mut manifold) = self.collide_pair(handle_a, handle_b) else {
                continue;
            };

            // Warm start from the manifold of the previous step
            if let Some(old) = previous.iter().find(|old| {
                old.collider_a() == manifold.collider_a()
                    && old.collider_b() == manifold.collider_b()
            }) {
                manifold.warm_start_from(old);
            }

            touching.insert((handle_a, handle_b));
            if emit_events {
                self.events.push(self.make_event(
                    if self.touching_pairs.contains(&(handle_a, handle_b)) {
                        CollisionEventType::Persisted
                    } else {
                        CollisionEventType::Started
                    },
                    handle_a,
                    handle_b,
                ));
            }

            manifolds.push(manifold);
        }

        if emit_events {
            // Pairs that disappeared since the last step have ended
            let mut ended: Vec<(ColliderHandle, ColliderHandle)> = self
                .touching_pairs
                .iter()
                .filter(|pair| !touching.contains(pair))
                .copied()
                .collect();

            // `touching_pairs` is a set, so sort before reporting: the event
            // order should not change from run to run.
            ended.sort_unstable_by_key(|(a, b)| (a.id, a.generation, b.id, b.generation));

            for (handle_a, handle_b) in ended {
                self.events
                    .push(self.make_event(CollisionEventType::Ended, handle_a, handle_b));
            }

            self.touching_pairs = touching;
        }

        manifolds
    }

    /// Run the narrow-phase for a single pair of colliders
    fn collide_pair(
        &self,
        handle_a: ColliderHandle,
        handle_b: ColliderHandle,
    ) -> Option<ContactManifold> {
        let collider_a = self.get_collider(handle_a)?;
        let collider_b = self.get_collider(handle_b)?;

        if !collider_a.is_enabled() || !collider_b.is_enabled() {
            return None;
        }

        if !collider_a.filter().can_collide_with(&collider_b.filter()) {
            return None;
        }

        // Colliders on the same body (or on bodies joined by a constraint that
        // disables collision) never generate contacts.
        if collider_a.body_handle() == collider_b.body_handle() {
            return None;
        }

        let body_a = self.get_body(collider_a.body_handle())?;
        let body_b = self.get_body(collider_b.body_handle())?;

        if !body_a.is_enabled() || !body_b.is_enabled() {
            return None;
        }

        // Two non-dynamic bodies cannot resolve anything, but sensors still
        // need their overlap reported.
        if !body_a.is_dynamic()
            && !body_b.is_dynamic()
            && !collider_a.is_sensor()
            && !collider_b.is_sensor()
        {
            return None;
        }

        if body_a.is_sleeping() && body_b.is_sleeping() {
            return None;
        }

        let transform_a = collider_a.world_transform(body_a.transform());
        let transform_b = collider_b.world_transform(body_b.transform());

        self.narrow_phase
            .collide(collider_a, transform_a, collider_b, transform_b)
    }

    /// Build a collision event for a pair of colliders
    fn make_event(
        &self,
        event_type: CollisionEventType,
        handle_a: ColliderHandle,
        handle_b: ColliderHandle,
    ) -> CollisionEvent {
        let (body_a, sensor_a) = self
            .get_collider(handle_a)
            .map_or((BodyHandle::null(), false), |c| {
                (c.body_handle(), c.is_sensor())
            });
        let (body_b, sensor_b) = self
            .get_collider(handle_b)
            .map_or((BodyHandle::null(), false), |c| {
                (c.body_handle(), c.is_sensor())
            });

        CollisionEvent::new(
            event_type, body_a, body_b, handle_a, handle_b, sensor_a, sensor_b,
        )
    }

    /// Apply gravity, user forces and damping to velocities
    fn integrate_velocities(&mut self, dt: f32) {
        let gravity = self.config.gravity;

        for body in self.bodies.iter_mut().flatten() {
            if !body.is_dynamic() || !body.is_enabled() || body.is_sleeping() {
                continue;
            }

            let inv_mass = body.inv_mass();
            let inv_inertia = body.inv_inertia();

            let gravity_velocity = gravity * body.gravity_scale() * dt;
            let force_velocity = body.force() * inv_mass * dt;
            let torque_velocity = body.torque() * inv_inertia * dt;

            let mut linear = body.linear_velocity() + gravity_velocity + force_velocity;
            let mut angular = body.angular_velocity() + torque_velocity;

            // Time-correct damping: v /= (1 + damping * dt), which is stable
            // for any dt and any damping value.
            let linear_damping = body.linear_damping().max(0.0);
            let angular_damping = body.angular_damping().max(0.0);
            linear /= 1.0 + linear_damping * dt;
            angular /= 1.0 + angular_damping * dt;

            body.set_linear_velocity_no_wake(linear);
            body.set_angular_velocity_no_wake(angular);
            body.clear_accumulators();
        }
    }

    /// Integrate positions from velocities
    fn integrate_positions(&mut self, dt: f32) {
        for body in self.bodies.iter_mut().flatten() {
            if body.is_static() || !body.is_enabled() || body.is_sleeping() {
                continue;
            }

            let position = body.position() + body.linear_velocity() * dt;
            let rotation = body.rotation() + Rot2::new(body.angular_velocity() * dt);

            body.set_position_no_wake(position);
            body.set_rotation_no_wake(rotation);
        }
    }

    /// Integrate positions, splitting the step at the earliest impact
    ///
    /// When CCD is enabled the earliest time of impact over all candidate pairs
    /// is computed first. Bodies are advanced to that instant, the contacts are
    /// re-detected and re-solved (so the impact is answered immediately), and
    /// only then is the remaining time integrated. The loop is bounded by
    /// [`MAX_CCD_SUBSTEPS`].
    fn integrate_positions_with_ccd(&mut self, dt: f32) {
        let mut remaining = dt;

        for _ in 0..MAX_CCD_SUBSTEPS {
            let toi = self.earliest_toi(remaining);
            let fraction = if toi.is_finite() && toi > 0.0 && toi < 1.0 {
                toi
            } else {
                1.0
            };

            let slice = remaining * fraction;
            self.integrate_positions(slice);
            remaining -= slice;

            if fraction >= 1.0 || remaining <= EPSILON {
                break;
            }

            // Resolve the impact before continuing with the remaining time
            self.update_broad_phase();
            self.manifolds = self.detect_contacts(false);
            self.solve_contacts(remaining.max(MIN_TIMESTEP));
        }

        if remaining > EPSILON {
            self.integrate_positions(remaining);
        }
    }

    /// The earliest time of impact (as a fraction of `dt`) over all pairs
    fn earliest_toi(&self, dt: f32) -> f32 {
        let mut earliest = 1.0_f32;

        for (handle_a, handle_b) in self.broad_phase.find_collision_pairs() {
            let (Some(collider_a), Some(collider_b)) =
                (self.get_collider(handle_a), self.get_collider(handle_b))
            else {
                continue;
            };
            let (Some(body_a), Some(body_b)) = (
                self.get_body(collider_a.body_handle()),
                self.get_body(collider_b.body_handle()),
            ) else {
                continue;
            };

            if !body_a.is_dynamic() && !body_b.is_dynamic() {
                continue;
            }

            if !self
                .swept_detector
                .will_tunnel(body_a, collider_a, body_b, collider_b, dt)
            {
                continue;
            }

            if let Some(result) = self
                .swept_detector
                .check_swept_collision(body_a, collider_a, body_b, collider_b, dt)
            {
                earliest = earliest.min(result.clamped_toi());
            }
        }

        earliest
    }

    /// Solve the contact manifolds produced by the current step
    fn solve_contacts(&mut self, dt: f32) {
        if self.manifolds.is_empty() {
            return;
        }

        let World {
            bodies,
            manifolds,
            contact_solver,
            body_map,
            ..
        } = self;

        contact_solver.solve(bodies, manifolds, dt, body_map);
    }

    /// Solve all joint constraints
    ///
    /// Each constraint is temporarily taken out of the constraint storage so
    /// that it can be handed two mutable bodies without fighting the borrow
    /// checker; it is put back before the loop continues.
    fn solve_constraints(&mut self, dt: f32) {
        if self.constraint_map.is_empty() {
            return;
        }

        // Solve in slot order so the result does not depend on the iteration
        // order of `constraint_map`.
        let handles: Vec<ConstraintHandle> = self
            .constraints
            .iter()
            .flatten()
            .map(|constraint| constraint.handle())
            .collect();

        // Pre-solve (read-only access to the bodies)
        for handle in &handles {
            let Some(index) = self.constraint_map.get(handle).copied() else {
                continue;
            };
            let Some(mut constraint) = self.constraints[index].take() else {
                continue;
            };

            if constraint.is_enabled() {
                let body_a_handle = constraint.body_a();
                let body_b_handle = constraint.body_b();

                if let (Some(body_a), Some(body_b)) =
                    (self.body_ref(body_a_handle), self.body_ref(body_b_handle))
                {
                    constraint.pre_solve(body_a, body_b, dt);
                }
            }

            self.constraints[index] = Some(constraint);
        }

        // Solve (needs two mutable bodies at a time)
        for _ in 0..self.config.constraint_iterations {
            for handle in &handles {
                let Some(index) = self.constraint_map.get(handle).copied() else {
                    continue;
                };
                let Some(mut constraint) = self.constraints[index].take() else {
                    continue;
                };

                if constraint.is_enabled() {
                    let body_a_handle = constraint.body_a();
                    let body_b_handle = constraint.body_b();

                    if let Some((body_a, body_b)) =
                        self.two_bodies_mut(body_a_handle, body_b_handle)
                    {
                        // Joints must not nudge a sleeping body: it is woken
                        // first (see `wake_islands`) when it really has to move.
                        if !body_a.is_sleeping() && !body_b.is_sleeping() {
                            constraint.solve(body_a, body_b);
                        }
                    }
                }

                self.constraints[index] = Some(constraint);
            }
        }

        // Post-solve
        for index in self.constraint_map.values().copied().collect::<Vec<_>>() {
            if let Some(constraint) = self.constraints[index].as_mut() {
                if constraint.is_enabled() {
                    constraint.post_solve();
                }
            }
        }
    }

    /// Read-only access to a body by handle (usable while other fields are borrowed)
    #[inline]
    fn body_ref(&self, handle: BodyHandle) -> Option<&Body> {
        let index = *self.body_map.get(&handle)?;
        self.bodies.get(index)?.as_ref()
    }

    /// Borrow two distinct bodies mutably at the same time
    #[inline]
    fn two_bodies_mut(
        &mut self,
        handle_a: BodyHandle,
        handle_b: BodyHandle,
    ) -> Option<(&mut Body, &mut Body)> {
        let index_a = *self.body_map.get(&handle_a)?;
        let index_b = *self.body_map.get(&handle_b)?;
        if index_a == index_b {
            return None;
        }

        let (first, second) = if index_a < index_b {
            let (left, right) = self.bodies.split_at_mut(index_b);
            (left.get_mut(index_a)?, right.first_mut()?)
        } else {
            let (left, right) = self.bodies.split_at_mut(index_a);
            (right.first_mut()?, left.get_mut(index_b)?)
        };

        Some((first.as_mut()?, second.as_mut()?))
    }

    /// Wake sleeping bodies that are touching or joined to a moving body
    ///
    /// Wake propagation is what keeps a sleeping stack asleep while still
    /// letting it react the moment something moves into it. Contacts and
    /// joints both transmit motion, so both are treated as edges of the
    /// contact graph.
    fn wake_islands(&mut self) {
        let mut pairs: Vec<(BodyHandle, BodyHandle)> = self
            .manifolds
            .iter()
            .map(|manifold| (manifold.body_a(), manifold.body_b()))
            .collect();

        pairs.extend(
            self.constraints
                .iter()
                .flatten()
                .map(|constraint| (constraint.body_a(), constraint.body_b())),
        );

        for (handle_a, handle_b) in pairs {
            let Some(index_a) = self.body_map.get(&handle_a).copied() else {
                continue;
            };
            let Some(index_b) = self.body_map.get(&handle_b).copied() else {
                continue;
            };

            let a_awake = self.bodies[index_a]
                .as_ref()
                .is_some_and(|body| body.is_dynamic() && !body.is_sleeping());
            let b_awake = self.bodies[index_b]
                .as_ref()
                .is_some_and(|body| body.is_dynamic() && !body.is_sleeping());

            if a_awake && !b_awake {
                if let Some(body) = self.bodies[index_b].as_mut() {
                    body.wake_up();
                }
            } else if b_awake && !a_awake {
                if let Some(body) = self.bodies[index_a].as_mut() {
                    body.wake_up();
                }
            }
        }
    }

    /// Put bodies that have been at rest long enough to sleep, and reset the
    /// timer of those that are still moving
    fn update_sleeping(&mut self, dt: f32) {
        let config = self.config.sleeping_config;

        for body in self.bodies.iter_mut().flatten() {
            if !body.is_dynamic() || !body.is_enabled() {
                continue;
            }

            if body.is_sleeping() {
                continue;
            }

            if body.is_moving(config.linear_threshold, config.angular_threshold) {
                body.reset_sleep_timer();
            } else if body.advance_sleep_timer(dt) >= config.sleep_time {
                body.set_sleeping(true);
            }
        }
    }

    /// Notify event handlers
    fn notify_event_handlers(&mut self) {
        if self.event_handlers.is_empty() {
            // The events stay available through [`World::events`] until the
            // next step clears them; only the handlers consume them.
            return;
        }

        // Copy the events out first: the handlers are borrowed mutably while
        // they are called, which is not possible while `events` is borrowed.
        let events = self.events.clone();

        for event in events {
            for handler in self.event_handlers.iter_mut() {
                handler.on_collision(event);
            }
        }
    }

    /// Add a collision event handler
    #[inline]
    pub fn add_event_handler(&mut self, handler: Box<dyn CollisionEventHandler>) {
        self.event_handlers.push(handler);
    }

    /// Remove all event handlers
    #[inline]
    pub fn clear_event_handlers(&mut self) {
        self.event_handlers.clear();
    }

    /// Clear all bodies, colliders, and constraints
    #[inline]
    pub fn clear(&mut self) {
        self.bodies.clear();
        self.colliders.clear();
        self.constraints.clear();
        self.body_map.clear();
        self.collider_map.clear();
        self.constraint_map.clear();
        self.free_body_slots.clear();
        self.free_collider_slots.clear();
        self.free_constraint_slots.clear();
        self.body_colliders.clear();
        self.collider_densities.clear();
        self.broad_phase.clear();
        self.manifolds.clear();
        self.touching_pairs.clear();
        self.events.clear();
        self.time = 0.0;
        self.frame_count = 0;
    }
}

impl fmt::Debug for World {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("World")
            .field("config", &self.config)
            .field("bodies", &self.body_count())
            .field("colliders", &self.collider_count())
            .field("constraints", &self.constraint_count())
            .field("handlers", &self.event_handlers.len())
            .field("time", &self.time)
            .field("frame_count", &self.frame_count)
            .finish_non_exhaustive()
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new(WorldConfig::default())
    }
}

impl fmt::Display for World {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "World(bodies: {}, colliders: {}, constraints: {}, time: {:.2})",
            self.body_count(),
            self.collider_count(),
            self.constraint_count(),
            self.time
        )
    }
}

/// Vertices of an axis aligned box centred on the origin, in CCW order
#[inline]
fn box_vertices(half_extents: Vec2) -> Vec<Vec2> {
    let x = half_extents.x.abs().max(EPSILON);
    let y = half_extents.y.abs().max(EPSILON);

    vec![
        Vec2::new(-x, -y),
        Vec2::new(x, -y),
        Vec2::new(x, y),
        Vec2::new(-x, y),
    ]
}

/// Cast a ray against a shape, returning the closest hit within `max_distance`
#[inline]
fn ray_cast_shape(
    ray: Ray,
    max_distance: f32,
    shape: &Shape,
    transform: Transform,
    collider_handle: ColliderHandle,
    body_handle: BodyHandle,
) -> Option<RayCastResult> {
    // Move the ray into the shape's local frame: rotations (and translations)
    // preserve distances, so the hit distance stays in world units.
    let local_origin = (ray.origin - transform.position).rotate(-transform.rotation.angle());
    let local_direction = ray.direction.rotate(-transform.rotation.angle());

    let (distance, local_normal) = match shape {
        Shape::Circle(circle) => {
            ray_cast_circle(local_origin, local_direction, max_distance, circle.radius())?
        }
        Shape::Polygon(polygon) => ray_cast_polygon(
            local_origin,
            local_direction,
            max_distance,
            polygon.vertices(),
        )?,
    };

    let hit_local = local_origin + local_direction * distance;
    let hit_point = hit_local.rotate(transform.rotation.angle()) + transform.position;
    let hit_normal = local_normal.rotate(transform.rotation.angle());

    Some(RayCastResult::new(
        body_handle,
        collider_handle,
        hit_point,
        hit_normal,
        distance,
        distance / max_distance,
    ))
}

/// Ray-circle intersection; returns the distance and the surface normal
#[inline]
fn ray_cast_circle(
    origin: Vec2,
    direction: Vec2,
    max_distance: f32,
    radius: f32,
) -> Option<(f32, Vec2)> {
    let a = direction.length_squared();
    if a <= EPSILON {
        return None;
    }

    let b = 2.0 * origin.dot(direction);
    let c = origin.length_squared() - radius * radius;
    let discriminant = b * b - 4.0 * a * c;

    if discriminant < 0.0 {
        return None;
    }

    let root = discriminant.sqrt();
    let candidates = [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)];

    let mut distance = f32::INFINITY;
    for candidate in candidates {
        if candidate >= 0.0 && candidate <= max_distance && candidate < distance {
            distance = candidate;
        }
    }

    if !distance.is_finite() {
        return None;
    }

    // A ray that starts inside the circle reports a zero distance hit
    let point = origin + direction * distance;
    let normal = if point.length_squared() > EPSILON * EPSILON {
        point.normalize()
    } else {
        Vec2::unit_y()
    };

    Some((distance, normal))
}

/// Ray-convex-polygon intersection; returns the distance and the edge normal
#[inline]
fn ray_cast_polygon(
    origin: Vec2,
    direction: Vec2,
    max_distance: f32,
    vertices: &[Vec2],
) -> Option<(f32, Vec2)> {
    if vertices.len() < 3 {
        return None;
    }

    let count = vertices.len();
    let mut best: Option<(f32, Vec2)> = None;

    for (index, &v1) in vertices.iter().enumerate() {
        let v2 = vertices[(index + 1) % count];

        let edge = v2 - v1;
        let denominator = direction.cross(edge);
        if denominator.abs() <= EPSILON {
            continue; // Ray is parallel to this edge
        }

        let to_edge = v1 - origin;
        let t = to_edge.cross(edge) / denominator; // Along the ray
        let u = to_edge.cross(direction) / denominator; // Along the edge

        if !(0.0..=1.0).contains(&u) || t < 0.0 || t > max_distance {
            continue;
        }

        if best.map_or(true, |(distance, _)| t < distance) {
            // Outward normal of the CCW edge (dx, dy) is (dy, -dx)
            let mut normal = Vec2::new(edge.y, -edge.x).normalize();
            if normal.dot(direction) > 0.0 {
                // The ray hits the back face, so flip towards the ray
                normal = -normal;
            }
            best = Some((t, normal));
        }
    }

    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::CollisionEventType;
    use crate::math::utils::approx_eq;

    fn approx_eq_eps(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() <= epsilon
    }

    #[test]
    fn test_construction() {
        let world = World::new(WorldConfig::default());
        assert_eq!(world.body_count(), 0);
        assert_eq!(world.collider_count(), 0);
        assert!(approx_eq(world.time(), 0.0));
        assert_eq!(world.frame_count(), 0);

        let world2 = World::default();
        assert_eq!(world2.body_count(), 0);

        let world3 = World::with_gravity(Vec2::new(0.0, -10.0));
        assert_eq!(world3.gravity(), Vec2::new(0.0, -10.0));
    }

    #[test]
    fn test_create_destroy_body() {
        let mut world = World::default();

        let handle = world.create_dynamic_body(Transform::identity());
        assert!(world.has_body(handle));
        assert_eq!(world.body_count(), 1);

        assert!(world.destroy_body(handle));
        assert!(!world.has_body(handle));
        assert_eq!(world.body_count(), 0);
    }

    #[test]
    fn test_destroy_body_removes_colliders_and_constraints() {
        let mut world = World::default();

        let a = world.create_dynamic_body(Transform::identity());
        let b = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));

        let _ = world.add_circle(a, 0.5, 1.0);
        let _ = world.add_box(b, Vec2::splat(0.5), 1.0);
        let _ = world.create_distance_joint(a, b, Vec2::zero(), Vec2::zero());

        assert_eq!(world.collider_count(), 2);
        assert_eq!(world.constraint_count(), 1);

        assert!(world.destroy_body(a));

        assert_eq!(world.collider_count(), 1);
        assert_eq!(world.constraint_count(), 0);
    }

    #[test]
    fn test_body_slots_are_reused() {
        let mut world = World::default();

        let first = world.create_dynamic_body(Transform::identity());
        world.destroy_body(first);
        let second = world.create_dynamic_body(Transform::identity());

        assert_ne!(
            first, second,
            "handles must not be reused without a new generation"
        );
        assert!(world.has_body(second));
        assert!(!world.has_body(first));
        assert_eq!(world.body_count(), 1);
    }

    #[test]
    fn test_get_body() {
        let mut world = World::default();

        let handle =
            world.create_dynamic_body(Transform::new(Vec2::new(1.0, 2.0), Rot2::identity()));

        let body = world.get_body(handle);
        assert!(body.is_some());
        assert_eq!(body.expect("body").position(), Vec2::new(1.0, 2.0));

        assert!(world.get_body_mut(handle).is_some());
    }

    #[test]
    fn test_create_destroy_collider() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let shape = Shape::Circle(Circle::new(1.0));
        let collider_handle = world.create_collider(body_handle, shape);

        assert!(world.has_collider(collider_handle));
        assert_eq!(world.collider_count(), 1);

        assert!(world.destroy_collider(collider_handle));
        assert!(!world.has_collider(collider_handle));
        assert_eq!(world.collider_count(), 0);
    }

    #[test]
    fn test_add_circle_sets_mass() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let collider_handle = world.add_circle(body_handle, 1.0, 1.0);

        assert!(world.has_collider(collider_handle));

        let body = world.get_body(body_handle).expect("body");
        // Mass = pi * r^2 * density = pi
        assert!(approx_eq_eps(body.mass(), crate::constants::PI, 1e-4));
        assert!(body.inv_mass() > 0.0);
    }

    #[test]
    fn test_add_circle_rejects_invalid_radius() {
        let mut world = World::default();
        let body_handle = world.create_dynamic_body(Transform::identity());

        assert!(world.add_circle(body_handle, -1.0, 1.0).is_null());
        assert!(world.add_circle(body_handle, f32::NAN, 1.0).is_null());
        assert_eq!(world.collider_count(), 0);
    }

    #[test]
    fn test_add_box_sets_mass() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let collider_handle = world.add_box(body_handle, Vec2::new(1.0, 1.0), 1.0);

        assert!(world.has_collider(collider_handle));

        let body = world.get_body(body_handle).expect("body");
        // Area = 2 * 2 = 4 -> mass 4
        assert!(approx_eq_eps(body.mass(), 4.0, 1e-4));
    }

    #[test]
    fn test_mass_is_aggregated_over_colliders() {
        let mut world = World::default();

        let body = world.create_dynamic_body(Transform::identity());
        let _ = world.add_box(body, Vec2::splat(0.5), 1.0); // area 1
        let _ = world.add_box(body, Vec2::splat(0.5), 1.0); // area 1

        let body = world.get_body(body).expect("body");
        assert!(approx_eq_eps(body.mass(), 2.0, 1e-4));
    }

    #[test]
    fn test_static_bodies_have_no_mass() {
        let mut world = World::default();

        let body = world.create_static_body(Transform::identity());
        let _ = world.add_box(body, Vec2::splat(0.5), 1.0);

        let body = world.get_body(body).expect("body");
        assert_eq!(body.mass(), 0.0);
        assert_eq!(body.inv_mass(), 0.0);
    }

    #[test]
    fn test_step_integrates_gravity() {
        let mut world = World::with_gravity(Vec2::new(0.0, -10.0));

        let body_handle = world.create_dynamic_body(Transform::identity());
        let _ = world.add_circle(body_handle, 0.5, 1.0);

        world.step(1.0 / 60.0);

        assert!(world.time() > 0.0);
        assert_eq!(world.frame_count(), 1);

        let body = world.get_body(body_handle).expect("body");
        assert!(body.position().y < 0.0);
        assert!(body.linear_velocity().y < 0.0);
    }

    #[test]
    fn test_step_rejects_invalid_timesteps() {
        let mut world = World::default();
        let body_handle = world.create_dynamic_body(Transform::identity());
        let _ = world.add_circle(body_handle, 0.5, 1.0);

        world.step(0.0);
        world.step(-1.0);
        world.step(f32::NAN);
        world.step(f32::INFINITY);

        assert_eq!(world.frame_count(), 0);
        assert!(approx_eq(world.time(), 0.0));

        let body = world.get_body(body_handle).expect("body");
        assert_eq!(body.position(), Vec2::zero());
    }

    #[test]
    fn test_body_rests_on_static_ground() {
        let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

        let ground = world.create_static_body(Transform::from_translation(Vec2::new(0.0, -1.0)));
        let _ = world.add_box(ground, Vec2::new(10.0, 0.5), 0.0);

        let body = world.create_dynamic_body(Transform::from_translation(Vec2::new(0.0, 3.0)));
        let _ = world.add_box(body, Vec2::splat(0.5), 1.0);

        for _ in 0..240 {
            world.step(1.0 / 60.0);
        }

        let body = world.get_body(body).expect("body");
        // The box rests on the ground: its centre sits at -1 + 0.5 + 0.5
        assert!(
            approx_eq_eps(body.position().y, 0.0, 0.1),
            "body should rest on the ground, y = {}",
            body.position().y
        );
        assert!(body.linear_velocity().length() < 0.5);
    }

    #[test]
    fn test_collision_events_start_and_end() {
        let mut world = World::with_gravity(Vec2::zero());

        let a = world.create_dynamic_body(Transform::from_translation(Vec2::new(-2.0, 0.0)));
        let collider_a = world.add_box(a, Vec2::splat(0.5), 1.0);
        let b = world.create_dynamic_body(Transform::from_translation(Vec2::new(2.0, 0.0)));
        let collider_b = world.add_box(b, Vec2::splat(0.5), 1.0);

        // No bounce: the boxes stay in contact once they meet
        for collider in [collider_a, collider_b] {
            world
                .get_collider_mut(collider)
                .expect("collider")
                .set_restitution(0.0);
        }

        // Far apart: no contacts
        world.step(1.0 / 60.0);
        assert!(world.events().is_empty());

        // Keep pushing them together: one Started event, then persisted ones.
        // The boxes start 3 m apart (centres at -2 and 2, half extents 0.5),
        // so at 5 m/s each they need roughly 18 steps of 1/60 s to meet.
        let mut started = 0;
        let mut persisted = 0;
        for _ in 0..40 {
            world
                .get_body_mut(a)
                .expect("a")
                .set_linear_velocity(Vec2::new(5.0, 0.0));
            world
                .get_body_mut(b)
                .expect("b")
                .set_linear_velocity(Vec2::new(-5.0, 0.0));

            world.step(1.0 / 60.0);

            for event in world.events() {
                match event.event_type {
                    CollisionEventType::Started => started += 1,
                    CollisionEventType::Persisted => persisted += 1,
                    CollisionEventType::Ended => {}
                }
            }
        }

        assert_eq!(started, 1, "exactly one Started event expected");
        assert!(persisted > 0, "the contact should then persist");
        assert!(world
            .events()
            .iter()
            .any(|event| event.event_type == CollisionEventType::Persisted));
    }

    #[test]
    fn test_sensor_reports_events_without_impulses() {
        let mut world = World::with_gravity(Vec2::zero());

        let ground = world.create_static_body(Transform::identity());
        let sensor = world.add_box(ground, Vec2::splat(1.0), 0.0);
        world
            .get_collider_mut(sensor)
            .expect("sensor")
            .set_is_sensor(true);

        let body = world.create_dynamic_body(Transform::from_translation(Vec2::new(0.0, 0.5)));
        let _ = world.add_box(body, Vec2::splat(0.5), 1.0);

        world.step(1.0 / 60.0);

        assert!(world.events().iter().any(CollisionEvent::is_sensor));

        let body = world.get_body(body).expect("body");
        // Gravity is disabled, so the body must keep its (zero) velocity:
        // the sensor must not have pushed it.
        assert_eq!(body.linear_velocity(), Vec2::zero());
    }

    #[test]
    fn test_ray_cast_hits_body() {
        let mut world = World::default();

        let body = world.create_static_body(Transform::identity());
        let _ = world.add_circle(body, 1.0, 0.0);

        let ray = Ray::new(Vec2::new(-5.0, 0.0), Vec2::unit_x());
        let hit = world.ray_cast(ray, 100.0).expect("hit");

        assert_eq!(hit.body_handle, body);
        assert!(approx_eq_eps(hit.distance, 4.0, 1e-3));
        assert!(approx_eq_eps(hit.hit_point.x, -1.0, 1e-3));
    }

    #[test]
    fn test_ray_cast_edge_cases() {
        let mut world = World::default();
        let body = world.create_static_body(Transform::identity());
        let _ = world.add_box(body, Vec2::splat(1.0), 0.0);

        // Zero length direction
        assert!(world
            .ray_cast(Ray::new(Vec2::zero(), Vec2::zero()), 10.0)
            .is_none());
        // Non finite ray
        assert!(world
            .ray_cast(Ray::new(Vec2::new(f32::NAN, 0.0), Vec2::unit_x()), 10.0)
            .is_none());
        // Non positive max distance
        assert!(world
            .ray_cast(Ray::new(Vec2::zero(), Vec2::unit_x()), 0.0)
            .is_none());
        // Ray pointing away from the body
        assert!(world
            .ray_cast(Ray::new(Vec2::new(5.0, 0.0), Vec2::unit_x()), 10.0)
            .is_none());
        // Ray starting inside the body
        let inside = world.ray_cast(Ray::new(Vec2::zero(), Vec2::unit_x()), 10.0);
        assert!(inside.is_some(), "a ray starting inside a box must hit it");
    }

    #[test]
    fn test_point_and_aabb_queries() {
        let mut world = World::default();

        let body = world.create_static_body(Transform::identity());
        let _ = world.add_box(body, Vec2::splat(1.0), 0.0);

        assert_eq!(world.point_query(Vec2::new(0.5, 0.5)).len(), 1);
        assert!(world.point_query(Vec2::new(5.0, 5.0)).is_empty());
        assert!(world.point_query(Vec2::new(f32::NAN, 0.0)).is_empty());

        let results = world.aabb_query(Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(2.0, 2.0)));
        assert_eq!(results.len(), 1);
        assert!(world
            .aabb_query(Aabb::new(Vec2::new(10.0, 10.0), Vec2::new(20.0, 20.0)))
            .is_empty());
    }

    #[test]
    fn test_sleeping_bodies_stop_moving() {
        let mut world = World::with_gravity(Vec2::new(0.0, -9.81));

        let ground = world.create_static_body(Transform::from_translation(Vec2::new(0.0, -1.0)));
        let _ = world.add_box(ground, Vec2::new(10.0, 0.5), 0.0);

        let body = world.create_dynamic_body(Transform::from_translation(Vec2::new(0.0, 0.0)));
        let _ = world.add_box(body, Vec2::splat(0.5), 1.0);

        for _ in 0..600 {
            world.step(1.0 / 60.0);
        }

        let body = world.get_body(body).expect("body");
        assert!(body.is_sleeping(), "a body at rest should fall asleep");
        assert_eq!(body.linear_velocity(), Vec2::zero());
    }

    #[test]
    fn test_event_handlers() {
        let mut world = World::default();

        let handler = Box::new(crate::collision::SimpleCollisionEventHandler::new());
        world.add_event_handler(handler);

        assert_eq!(world.event_handlers.len(), 1);

        world.clear_event_handlers();
        assert_eq!(world.event_handlers.len(), 0);
    }

    #[test]
    fn test_clear() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let _ = world.add_circle(body_handle, 1.0, 1.0);

        assert_eq!(world.body_count(), 1);
        assert_eq!(world.collider_count(), 1);

        world.clear();
        assert_eq!(world.body_count(), 0);
        assert_eq!(world.collider_count(), 0);
        assert!(approx_eq(world.time(), 0.0));
        assert_eq!(world.frame_count(), 0);
    }

    #[test]
    fn test_body_handles() {
        let mut world = World::default();

        let handle1 = world.create_dynamic_body(Transform::identity());
        let handle2 = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));

        let handles = world.body_handles();
        assert_eq!(handles.len(), 2);
        assert!(handles.contains(&handle1));
        assert!(handles.contains(&handle2));
    }

    #[test]
    fn test_collider_handles() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let collider1 = world.add_circle(body_handle, 1.0, 1.0);
        let collider2 = world.add_circle(body_handle, 0.5, 1.0);

        let handles = world.collider_handles();
        assert_eq!(handles.len(), 2);
        assert!(handles.contains(&collider1));
        assert!(handles.contains(&collider2));
    }

    #[test]
    fn test_display() {
        let world = World::default();
        let world_str = format!("{world}");
        assert!(world_str.contains("World"));
    }

    #[test]
    fn test_create_distance_joint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));

        let handle =
            world.create_distance_joint(body_a, body_b, Vec2::new(0.5, 0.0), Vec2::new(-0.5, 0.0));

        assert!(world.has_constraint(handle));
        assert_eq!(world.constraint_count(), 1);
    }

    #[test]
    fn test_create_revolute_joint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));

        let handle = world.create_revolute_joint(body_a, body_b, Vec2::new(0.5, 0.0));

        assert!(world.has_constraint(handle));
        assert_eq!(world.constraint_count(), 1);
    }

    #[test]
    fn test_create_motor_joint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));

        let handle = world.create_motor_joint(body_a, body_b);

        assert!(world.has_constraint(handle));
        assert_eq!(world.constraint_count(), 1);
    }

    #[test]
    fn test_destroy_constraint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));

        let handle = world.create_distance_joint(body_a, body_b, Vec2::zero(), Vec2::zero());
        assert_eq!(world.constraint_count(), 1);

        assert!(world.destroy_constraint(handle));
        assert!(!world.has_constraint(handle));
        assert_eq!(world.constraint_count(), 0);
    }

    #[test]
    fn test_constraint_handles() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));

        let joint1 = world.create_distance_joint(body_a, body_b, Vec2::zero(), Vec2::zero());
        let joint2 = world.create_revolute_joint(body_a, body_b, Vec2::zero());

        let handles = world.constraint_handles();
        assert_eq!(handles.len(), 2);
        assert!(handles.contains(&joint1));
        assert!(handles.contains(&joint2));
    }

    #[test]
    fn test_step_with_constraints() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let _ = world.add_circle(body_a, 1.0, 1.0);

        let body_b = world.create_dynamic_body(Transform::from_translation(Vec2::new(0.0, 2.0)));
        let _ = world.add_circle(body_b, 1.0, 1.0);

        let _ = world.create_distance_joint(body_a, body_b, Vec2::zero(), Vec2::zero());

        world.step(1.0 / 60.0);

        assert!(world.time() > 0.0);
        assert_eq!(world.frame_count(), 1);
    }

    #[test]
    fn test_clear_with_constraints() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::from_translation(Vec2::new(1.0, 0.0)));
        let _ = world.create_distance_joint(body_a, body_b, Vec2::zero(), Vec2::zero());

        assert_eq!(world.constraint_count(), 1);

        world.clear();
        assert_eq!(world.body_count(), 0);
        assert_eq!(world.collider_count(), 0);
        assert_eq!(world.constraint_count(), 0);
    }
}
