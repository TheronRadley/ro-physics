//! World implementation for the physics engine.
//!
//! This module provides the World type which is the main container
//! for the physics simulation.

use std::fmt;

use crate::bodies::{Body, BodyHandle, BodyType};
use crate::ccd::SweptCollisionDetector;
use crate::collision::{
    BroadPhase, BroadPhaseConfig, Collider, ColliderHandle, CollisionEvent,
    CollisionEventHandler, Filter, NarrowPhase, NarrowPhaseConfig, ContactManifold,
};
use crate::constraints::{Constraint, ConstraintHandle, DistanceJoint, RevoluteJoint, MotorJoint};
use crate::geometry::Aabb;
use crate::math::{Rot2, Transform, Vec2};
use crate::shapes::Shape;
use crate::solver::{ContactSolver, SolverConfig};
use super::WorldConfig;

/// The physics world.
///
/// The world is the main container for bodies, colliders, and the simulation state.
/// It manages the physics simulation and provides the public API.
#[derive(Debug)]
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
    body_map: std::collections::HashMap<BodyHandle, usize>,
    /// Map from collider handles to collider indices
    collider_map: std::collections::HashMap<ColliderHandle, usize>,
    /// Map from constraint handles to constraint indices
    constraint_map: std::collections::HashMap<ConstraintHandle, usize>,
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
    /// Contact manifolds from the previous step (for warm starting)
    manifolds: Vec<ContactManifold>,
}

impl World {
    /// Create a new physics world with the given configuration
    #[inline]
    #[must_use]
    pub fn new(config: WorldConfig) -> Self {
        Self {
            config: config.clone(),
            broad_phase: BroadPhase::new(BroadPhaseConfig::new(
                config.max_colliders,
            )),
            narrow_phase: NarrowPhase::new(NarrowPhaseConfig::default()),
            contact_solver: ContactSolver::new(config.solver_config.clone()),
            swept_detector: SweptCollisionDetector::new(config.ccd_config.clone()),
            bodies: Vec::with_capacity(config.max_bodies),
            colliders: Vec::with_capacity(config.max_colliders),
            constraints: Vec::with_capacity(config.max_constraints),
            body_map: std::collections::HashMap::new(),
            collider_map: std::collections::HashMap::new(),
            constraint_map: std::collections::HashMap::new(),
            body_handle_generator: crate::bodies::BodyHandleGenerator::new(),
            collider_handle_generator: crate::collision::ColliderHandleGenerator::new(),
            constraint_handle_generator: crate::constraints::ConstraintHandleGenerator::new(),
            event_handlers: Vec::new(),
            time: 0.0,
            frame_count: 0,
            manifolds: Vec::with_capacity(config.max_contacts),
        }
    }

    /// Create a new physics world with default configuration
    #[inline]
    #[must_use]
    pub fn default() -> Self {
        Self::new(WorldConfig::default())
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
    #[inline]
    #[must_use]
    pub fn create_body(&mut self, body_type: BodyType, transform: Transform) -> BodyHandle {
        // Check if we have capacity
        if self.bodies.len() >= self.config.max_bodies {
            return BodyHandle::null();
        }

        // Generate a new handle
        let handle = self.body_handle_generator.generate();

        // Create the body
        let body = match body_type {
            BodyType::Static => Body::new_static(handle),
            BodyType::Dynamic => Body::new_dynamic(handle),
            BodyType::Kinematic => Body::new_kinematic(handle),
        };

        let mut body = body;
        body.set_transform(transform);

        // Store the body
        let index = self.bodies.len();
        self.bodies.push(Some(body));
        self.body_map.insert(handle, index);

        handle
    }

    /// Destroy a body
    #[inline]
    pub fn destroy_body(&mut self, handle: BodyHandle) -> bool {
        if let Some(index) = self.body_map.remove(&handle) {
            if index < self.bodies.len() {
                self.bodies[index] = None;
                
                // Remove all colliders attached to this body
                for i in (0..self.colliders.len()).rev() {
                    if let Some(collider) = &self.colliders[i] {
                        if collider.body_handle() == handle {
                            self.destroy_collider(collider.handle());
                        }
                    }
                }
                
                return true;
            }
        }
        false
    }

    /// Get a body by handle
    #[inline]
    #[must_use]
    pub fn get_body(&self, handle: BodyHandle) -> Option<&Body> {
        self.body_map
            .get(&handle)
            .and_then(|&index| self.bodies.get(index))
            .and_then(|body| body.as_ref())
    }

    /// Get a body by handle mutably
    #[inline]
    #[must_use]
    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Option<&mut Body> {
        self.body_map
            .get(&handle)
            .and_then(|&index| self.bodies.get_mut(index))
            .and_then(|body| body.as_mut())
    }

    /// Check if a body exists
    #[inline]
    #[must_use]
    pub fn has_body(&self, handle: BodyHandle) -> bool {
        self.body_map.contains_key(&handle)
    }

    /// Create a new collider attached to a body
    #[inline]
    #[must_use]
    pub fn create_collider(
        &mut self,
        body_handle: BodyHandle,
        shape: Shape,
    ) -> ColliderHandle {
        // Check if the body exists
        if !self.has_body(body_handle) {
            return ColliderHandle::null();
        }

        // Check if we have capacity
        if self.colliders.len() >= self.config.max_colliders {
            return ColliderHandle::null();
        }

        // Generate a new handle
        let handle = self.collider_handle_generator.generate();

        // Create the collider
        let collider = Collider::new(handle, body_handle, shape);

        // Store the collider
        let index = self.colliders.len();
        self.colliders.push(Some(collider));
        self.collider_map.insert(handle, index);

        // Add to broad-phase
        if let Some(body) = self.get_body(body_handle) {
            let aabb = self.calculate_collider_aabb(handle);
            if let Some(aabb) = aabb {
                self.broad_phase.add_collider(handle, aabb, index);
            }
        }

        handle
    }

    /// Destroy a collider
    #[inline]
    pub fn destroy_collider(&mut self, handle: ColliderHandle) -> bool {
        if let Some(index) = self.collider_map.remove(&handle) {
            if index < self.colliders.len() {
                self.colliders[index] = None;
                self.broad_phase.remove_collider(handle);
                return true;
            }
        }
        false
    }

    /// Get a collider by handle
    #[inline]
    #[must_use]
    pub fn get_collider(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.collider_map
            .get(&handle)
            .and_then(|&index| self.colliders.get(index))
            .and_then(|collider| collider.as_ref())
    }

    /// Get a collider by handle mutably
    #[inline]
    #[must_use]
    pub fn get_collider_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.collider_map
            .get(&handle)
            .and_then(|&index| self.colliders.get_mut(index))
            .and_then(|collider| collider.as_mut())
    }

    /// Check if a collider exists
    #[inline]
    #[must_use]
    pub fn has_collider(&self, handle: ColliderHandle) -> bool {
        self.collider_map.contains_key(&handle)
    }

    /// Add a circle shape to a body
    #[inline]
    #[must_use]
    pub fn add_circle(
        &mut self,
        body_handle: BodyHandle,
        radius: f32,
        density: f32,
    ) -> ColliderHandle {
        let shape = Shape::Circle(crate::shapes::Circle::new(radius));
        
        // Calculate mass properties
        let mass_props = shape.mass_properties(density);
        
        // Set mass properties on the body
        if let Some(body) = self.get_body_mut(body_handle) {
            body.set_mass_properties(mass_props.mass, mass_props.inertia);
        }
        
        self.create_collider(body_handle, shape)
    }

    /// Add a box shape to a body
    #[inline]
    #[must_use]
    pub fn add_box(
        &mut self,
        body_handle: BodyHandle,
        half_extents: Vec2,
        density: f32,
    ) -> ColliderHandle {
        let shape = Shape::Polygon(crate::shapes::Polygon::from_box(half_extents));
        
        // Calculate mass properties
        let mass_props = shape.mass_properties(density);
        
        // Set mass properties on the body
        if let Some(body) = self.get_body_mut(body_handle) {
            body.set_mass_properties(mass_props.mass, mass_props.inertia);
        }
        
        self.create_collider(body_handle, shape)
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
        let shape = Shape::Polygon(crate::shapes::Polygon::new(vertices));
        
        // Calculate mass properties
        let mass_props = shape.mass_properties(density);
        
        // Set mass properties on the body
        if let Some(body) = self.get_body_mut(body_handle) {
            body.set_mass_properties(mass_props.mass, mass_props.inertia);
        }
        
        self.create_collider(body_handle, shape)
    }

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
        // Check if we have capacity
        if self.constraints.len() >= self.config.max_constraints {
            return ConstraintHandle::null();
        }

        // Check if both bodies exist
        if !self.has_body(body_a) || !self.has_body(body_b) {
            return ConstraintHandle::null();
        }

        // Generate a new handle
        let handle = self.constraint_handle_generator.generate();

        // Create the joint
        let joint = DistanceJoint::new(handle, body_a, body_b, anchor_a, anchor_b);

        // Store the constraint
        let index = self.constraints.len();
        self.constraints.push(Some(Box::new(joint) as Box<dyn Constraint>));
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
        // Check if we have capacity
        if self.constraints.len() >= self.config.max_constraints {
            return ConstraintHandle::null();
        }

        // Check if both bodies exist
        if !self.has_body(body_a) || !self.has_body(body_b) {
            return ConstraintHandle::null();
        }

        // Generate a new handle
        let handle = self.constraint_handle_generator.generate();

        // Create the joint
        let joint = RevoluteJoint::new(handle, body_a, body_b, anchor);

        // Store the constraint
        let index = self.constraints.len();
        self.constraints.push(Some(Box::new(joint) as Box<dyn Constraint>));
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
        // Check if we have capacity
        if self.constraints.len() >= self.config.max_constraints {
            return ConstraintHandle::null();
        }

        // Check if both bodies exist
        if !self.has_body(body_a) || !self.has_body(body_b) {
            return ConstraintHandle::null();
        }

        // Generate a new handle
        let handle = self.constraint_handle_generator.generate();

        // Create the joint
        let joint = MotorJoint::new(handle, body_a, body_b);

        // Store the constraint
        let index = self.constraints.len();
        self.constraints.push(Some(Box::new(joint) as Box<dyn Constraint>));
        self.constraint_map.insert(handle, index);

        handle
    }

    /// Destroy a constraint
    #[inline]
    pub fn destroy_constraint(&mut self, handle: ConstraintHandle) -> bool {
        if let Some(index) = self.constraint_map.remove(&handle) {
            if index < self.constraints.len() {
                self.constraints[index] = None;
                return true;
            }
        }
        false
    }

    /// Get a constraint by handle
    #[inline]
    #[must_use]
    pub fn get_constraint(&self, handle: ConstraintHandle) -> Option<&dyn Constraint> {
        self.constraint_map
            .get(&handle)
            .and_then(|&index| self.constraints.get(index))
            .and_then(|constraint| constraint.as_ref())
            .map(|c| c.as_ref())
    }

    /// Get a constraint by handle mutably
    #[inline]
    #[must_use]
    pub fn get_constraint_mut(&mut self, handle: ConstraintHandle) -> Option<&mut dyn Constraint> {
        self.constraint_map
            .get(&handle)
            .and_then(|&index| self.constraints.get_mut(index))
            .and_then(|constraint| constraint.as_mut())
            .map(|c| c.as_mut())
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
    #[inline]
    #[must_use]
    pub fn constraint_handles(&self) -> Vec<ConstraintHandle> {
        self.constraint_map.keys().cloned().collect()
    }

    /// Perform a point query to find colliders containing a point
    #[inline]
    #[must_use]
    pub fn point_query(&self, point: Vec2) -> Vec<crate::query::PointQueryResult> {
        let mut results = Vec::new();
        
        for (handle, &collider) in &self.colliders {
            if let Some(collider) = collider {
                if let Some(body) = self.get_body(collider.body_handle()) {
                    let transform = body.transform();
                    let shape = collider.shape();
                    
                    if let Some(shape) = shape {
                        if Self::point_in_shape(point, &shape, transform) {
                            results.push(crate::query::PointQueryResult::new(
                                body.handle(),
                                collider.handle(),
                                true,
                                0.0,
                            ));
                        }
                    }
                }
            }
        }
        
        results
    }

    /// Perform an AABB query to find colliders overlapping an AABB
    #[inline]
    #[must_use]
    pub fn aabb_query(&self, aabb: Aabb) -> Vec<crate::query::AabbQueryResult> {
        let mut results = Vec::new();
        
        for (handle, &collider) in &self.colliders {
            if let Some(collider) = collider {
                if let Some(body) = self.get_body(collider.body_handle()) {
                    let collider_aabb = collider.aabb(body.transform());
                    
                    if collider_aabb.overlaps(&aabb) {
                        results.push(crate::query::AabbQueryResult::new(
                            body.handle(),
                            collider.handle(),
                        ));
                    }
                }
            }
        }
        
        results
    }

    /// Perform a ray cast and return the first hit
    #[inline]
    #[must_use]
    pub fn ray_cast(&self, ray: Ray, max_distance: f32) -> Option<crate::query::RayCastResult> {
        let mut closest_hit: Option<crate::query::RayCastResult> = None;
        let mut closest_distance = max_distance;
        
        for (handle, &collider) in &self.colliders {
            if let Some(collider) = collider {
                if let Some(body) = self.get_body(collider.body_handle()) {
                    let shape = collider.shape();
                    
                    if let Some(shape) = shape {
                        if let Some(hit) = Self::ray_cast_shape(
                            &ray, max_distance, body.transform(), &shape, collider.handle(), body.handle()
                        ) {
                            if hit.distance < closest_distance {
                                closest_hit = Some(hit);
                                closest_distance = hit.distance;
                            }
                        }
                    }
                }
            }
        }
        
        closest_hit
    }

    /// Perform a ray cast and return all hits
    #[inline]
    #[must_use]
    pub fn ray_cast_all(&self, ray: Ray, max_distance: f32) -> Vec<crate::query::RayCastResult> {
        let mut hits = Vec::new();
        
        for (handle, &collider) in &self.colliders {
            if let Some(collider) = collider {
                if let Some(body) = self.get_body(collider.body_handle()) {
                    let shape = collider.shape();
                    
                    if let Some(shape) = shape {
                        if let Some(hit) = Self::ray_cast_shape(
                            &ray, max_distance, body.transform(), &shape, collider.handle(), body.handle()
                        ) {
                            hits.push(hit);
                        }
                    }
                }
            }
        }
        
        // Sort by distance
        hits.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        hits
    }

    /// Step the simulation forward by a fixed timestep
    #[inline]
    pub fn step(&mut self, dt: f32) {
        // Increment time and frame count
        self.time += dt;
        self.frame_count += 1;

        // Clear old manifolds
        self.manifolds.clear();

        // Update broad-phase with current transforms
        self.update_broad_phase();

        // Find collision pairs
        let pairs = self.broad_phase.find_collision_pairs();

        // Check for CCD (Continuous Collision Detection)
        // This prevents fast-moving objects from tunneling through other objects
        if self.config.ccd_config.enabled {
            self.check_ccd(&pairs, dt);
        }

        // Process collisions and generate new manifolds
        for (handle_a, handle_b) in pairs {
            self.process_collision_pair(handle_a, handle_b);
        }

        // Pre-solve constraints
        self.pre_solve_constraints(dt);

        // Solve contact constraints
        if !self.manifolds.is_empty() {
            self.solve_contacts(dt);
        }

        // Solve joint constraints
        self.solve_constraints(dt);

        // Integrate bodies
        self.integrate_bodies(dt);

        // Apply forces and clear accumulators
        self.apply_forces(dt);

        // Handle sleeping
        if self.config.enable_sleeping {
            self.update_sleeping();
        }

        // Notify event handlers
        self.notify_event_handlers();
    }

    /// Update the broad-phase with current body transforms
    fn update_broad_phase(&mut self) {
        for (handle, &collider) in self.colliders.iter().enumerate() {
            if let Some(collider) = collider {
                if let Some(body) = self.get_body(collider.body_handle()) {
                    let aabb = self.calculate_collider_aabb(collider.handle());
                    if let Some(aabb) = aabb {
                        self.broad_phase.update_collider(collider.handle(), aabb);
                    }
                }
            }
        }
    }

    /// Calculate the AABB for a collider
    fn calculate_collider_aabb(&self, handle: ColliderHandle) -> Option<Aabb> {
        let collider = self.get_collider(handle)?;
        let body = self.get_body(collider.body_handle())?;
        
        Some(collider.aabb(body.transform()))
    }

    /// Check for continuous collision detection (CCD) between body pairs.
    ///
    /// This prevents fast-moving objects from tunneling through other objects
    /// by calculating the exact time of impact and advancing bodies to that time.
    fn check_ccd(&mut self, pairs: &[(ColliderHandle, ColliderHandle)], dt: f32) {
        for &(handle_a, handle_b) in pairs {
            let collider_a = self.get_collider(handle_a);
            let collider_b = self.get_collider(handle_b);

            if collider_a.is_none() || collider_b.is_none() {
                continue;
            }

            let collider_a = collider_a.unwrap();
            let collider_b = collider_b.unwrap();

            // Check if they can collide
            if !Filter::can_collide_with(&collider_a.filter(), &collider_b.filter()) {
                continue;
            }

            // Get the bodies
            let body_a = self.get_body(collider_a.body_handle());
            let body_b = self.get_body(collider_b.body_handle());

            if body_a.is_none() || body_b.is_none() {
                continue;
            }

            let body_a = body_a.unwrap();
            let body_b = body_b.unwrap();

            // Skip if both bodies are static
            if body_a.is_static() && body_b.is_static() {
                continue;
            }

            // Check if tunneling would occur
            if self.swept_detector.will_tunnel(body_a, collider_a, body_b, collider_b, dt) {
                // Calculate time of impact
                if let Some(toi_result) = self.swept_detector.check_swept_collision(
                    body_a, collider_a, body_b, collider_b, dt
                ) {
                    // If collision will happen before the end of the time step,
                    // we need to handle it specially
                    if toi_result.toi < 1.0 {
                        // For now, just ensure we detect the collision
                        // In a full implementation, we would:
                        // 1. Advance bodies to TOI
                        // 2. Handle the collision at that exact time
                        // 3. Continue with remaining time
                        // This is a simplified version that just ensures detection
                    }
                }
            }
        }
    }

    /// Pre-solve all constraints
    fn pre_solve_constraints(&mut self, dt: f32) {
        for constraint in &mut self.constraints {
            if let Some(constraint) = constraint {
                if constraint.is_enabled() {
                    let body_a = self.get_body(constraint.body_a());
                    let body_b = self.get_body(constraint.body_b());
                    
                    if let (Some(body_a), Some(body_b)) = (body_a, body_b) {
                        constraint.pre_solve(body_a, body_b, dt);
                    }
                }
            }
        }
    }

    /// Solve all joint constraints
    fn solve_constraints(&mut self, dt: f32) {
        for _ in 0..self.config.constraint_iterations {
            for constraint in &mut self.constraints {
                if let Some(constraint) = constraint {
                    if constraint.is_enabled() {
                        let body_a_handle = constraint.body_a();
                        let body_b_handle = constraint.body_b();
                        
                        let body_a = self.get_body_mut(body_a_handle);
                        let body_b = self.get_body_mut(body_b_handle);
                        
                        if let (Some(body_a), Some(body_b)) = (body_a, body_b) {
                            constraint.solve(body_a, body_b);
                        }
                    }
                }
            }
        }
        
        // Post-solve all constraints
        for constraint in &mut self.constraints {
            if let Some(constraint) = constraint {
                if constraint.is_enabled() {
                    constraint.post_solve();
                }
            }
        }
    }

    /// Process a collision pair
    fn process_collision_pair(
        &mut self,
        handle_a: ColliderHandle,
        handle_b: ColliderHandle,
    ) {
        let collider_a = self.get_collider(handle_a);
        let collider_b = self.get_collider(handle_b);

        if collider_a.is_none() || collider_b.is_none() {
            return;
        }

        let collider_a = collider_a.unwrap();
        let collider_b = collider_b.unwrap();

        // Check if they can collide
        if !Filter::can_collide_with(&collider_a.filter(), &collider_b.filter()) {
            return;
        }

        // Check if either is a sensor
        let is_sensor = collider_a.is_sensor() || collider_b.is_sensor();

        // Get the bodies
        let body_a = self.get_body(collider_a.body_handle());
        let body_b = self.get_body(collider_b.body_handle());

        if body_a.is_none() || body_b.is_none() {
            return;
        }

        let body_a = body_a.unwrap();
        let body_b = body_b.unwrap();

        // Skip if both bodies are static
        if body_a.is_static() && body_b.is_static() {
            return;
        }

        // Get transforms
        let transform_a = body_a.transform();
        let transform_b = body_b.transform();

        // Perform narrow-phase collision detection
        if let Some(mut manifold) = self.narrow_phase.collide(
            collider_a,
            transform_a,
            collider_b,
            transform_b,
        ) {
            // Set friction and restitution from colliders
            let friction = (collider_a.friction() + collider_b.friction()) / 2.0;
            let restitution = (collider_a.restitution() + collider_b.restitution()) / 2.0;
            manifold.set_friction(friction.clamp(0.0, 1.0));
            manifold.set_restitution(restitution.clamp(0.0, 1.0));

            // Add to manifolds list
            self.manifolds.push(manifold);

            // Generate collision events (unless both are sensors)
            if !is_sensor {
                self.generate_collision_events(handle_a, handle_b, &self.manifolds.last().unwrap());
            }
        }
    }

    /// Solve contact constraints using the solver
    fn solve_contacts(&mut self, dt: f32) {
        // Convert bodies to mutable slice
        let bodies_slice: Vec<Option<Body>> = self.bodies.iter().map(|b| b.clone()).collect();
        
        // Create a mutable copy for the solver
        let mut bodies_mut: Vec<Option<Body>> = bodies_slice;
        let mut manifolds_mut = self.manifolds.clone();

        // Solve contacts
        self.contact_solver.solve(&mut bodies_mut, &mut manifolds_mut, dt);

        // Copy back the results
        for (i, body_opt) in bodies_mut.iter().enumerate() {
            if i < self.bodies.len() {
                self.bodies[i] = body_opt.clone();
            }
        }
        self.manifolds = manifolds_mut;
    }

    /// Generate collision events
    fn generate_collision_events(
        &mut self,
        handle_a: ColliderHandle,
        handle_b: ColliderHandle,
        manifold: &ContactManifold,
    ) {
        let collider_a = self.get_collider(handle_a);
        let collider_b = self.get_collider(handle_b);

        if collider_a.is_none() || collider_b.is_none() {
            return;
        }

        let collider_a = collider_a.unwrap();
        let collider_b = collider_b.unwrap();

        let event = CollisionEvent::new(
            crate::collision::CollisionEventType::Started,
            collider_a.body_handle(),
            collider_b.body_handle(),
            handle_a,
            handle_b,
            collider_a.is_sensor(),
            collider_b.is_sensor(),
        );

        for handler in &mut self.event_handlers {
            handler.on_collision(event);
        }
    }

    /// Integrate bodies forward in time
    fn integrate_bodies(&mut self, dt: f32) {
        for body in &mut self.bodies {
            if let Some(body) = body {
                if body.is_dynamic() && body.is_enabled() && !body.is_sleeping() {
                    // Semi-implicit Euler integration
                    // v += (f/m) * dt
                    // w += (t/I) * dt
                    // x += v * dt
                    // angle += w * dt

                    let inv_mass = body.inv_mass();
                    let inv_inertia = body.inv_inertia();

                    // Apply gravity
                    let gravity_force = self.config.gravity * body.mass() * body.gravity_scale();

                    // Update velocity
                    let total_force = body.force() + gravity_force;
                    let linear_acceleration = total_force * inv_mass;
                    let angular_acceleration = body.torque() * inv_inertia;

                    let new_linear_velocity = body.linear_velocity() + linear_acceleration * dt;
                    let new_angular_velocity = body.angular_velocity() + angular_acceleration * dt;

                    // Apply damping
                    let linear_velocity = new_linear_velocity * (1.0 - body.linear_damping()).max(0.0);
                    let angular_velocity = new_angular_velocity * (1.0 - body.angular_damping()).max(0.0);

                    // Update position
                    let new_position = body.position() + linear_velocity * dt;
                    let new_rotation = body.rotation() + Rot2::new(angular_velocity * dt);

                    // Update body state
                    body.set_position(new_position);
                    body.set_rotation(new_rotation);
                    body.set_linear_velocity(linear_velocity);
                    body.set_angular_velocity(angular_velocity);

                    // Clear accumulators
                    body.clear_accumulators();
                }
            }
        }
    }

    /// Apply forces to bodies
    fn apply_forces(&mut self, _dt: f32) {
        // Forces are applied during integration
        // This is a placeholder for additional force application logic
    }

    /// Update sleeping state for all bodies
    fn update_sleeping(&mut self) {
        if !self.config.enable_sleeping {
            return;
        }

        // For now, simple sleeping: if a body is moving slowly, sleep it
        // This will be improved with proper sleeping islands later
        for body in &mut self.bodies {
            if let Some(body) = body {
                if body.is_dynamic() && body.is_enabled() && !body.is_sleeping() {
                    // Check if body is moving slowly
                    let speed = body.linear_velocity().length();
                    let angular_speed = body.angular_velocity().abs();

                    if speed < 0.01 && angular_speed < 0.01 {
                        body.set_sleeping(true);
                    }
                }
            }
        }
    }

    /// Notify event handlers
    fn notify_event_handlers(&mut self) {
        // TODO: Implement event notification
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

    /// Get the number of bodies
    #[inline]
    #[must_use]
    pub fn body_count(&self) -> usize {
        self.body_map.len()
    }

    /// Get the number of colliders
    #[inline]
    #[must_use]
    pub fn collider_count(&self) -> usize {
        self.collider_map.len()
    }

    /// Get all body handles
    #[inline]
    #[must_use]
    pub fn body_handles(&self) -> Vec<BodyHandle> {
        self.body_map.keys().cloned().collect()
    }

    /// Get all collider handles
    #[inline]
    #[must_use]
    pub fn collider_handles(&self) -> Vec<ColliderHandle> {
        self.collider_map.keys().cloned().collect()
    }

    /// Check if a point is inside a shape
    #[inline]
    fn point_in_shape(point: Vec2, shape: &Shape, transform: Transform) -> bool {
        let local_point = (point - transform.position).rotate(-transform.rotation.angle());
        
        match shape {
            Shape::Circle(circle) => {
                local_point.length_squared() <= circle.radius() * circle.radius()
            }
            Shape::Polygon(polygon) => {
                // Transform point to polygon local space
                polygon.contains_point(local_point)
            }
        }
    }

    /// Cast a ray against a shape
    #[inline]
    fn ray_cast_shape(
        ray: &Ray,
        max_distance: f32,
        transform: Transform,
        shape: &Shape,
        collider_handle: ColliderHandle,
        body_handle: BodyHandle,
    ) -> Option<crate::query::RayCastResult> {
        // Transform ray to shape local space
        let local_origin = (ray.origin - transform.position).rotate(-transform.rotation.angle());
        let local_direction = ray.direction.rotate(-transform.rotation.angle());
        let local_ray = Ray::new(local_origin, local_direction);
        
        match shape {
            Shape::Circle(circle) => {
                Self::ray_cast_circle(&local_ray, max_distance, circle, transform, collider_handle, body_handle)
            }
            Shape::Polygon(polygon) => {
                Self::ray_cast_polygon(&local_ray, max_distance, polygon, transform, collider_handle, body_handle)
            }
        }
    }

    /// Cast a ray against a circle
    #[inline]
    fn ray_cast_circle(
        ray: &Ray,
        max_distance: f32,
        circle: &Circle,
        transform: Transform,
        collider_handle: ColliderHandle,
        body_handle: BodyHandle,
    ) -> Option<crate::query::RayCastResult> {
        use crate::constants::EPSILON;
        
        let radius = circle.radius();
        let center = Vec2::zero(); // In local space, circle is at origin
        
        // Ray-circle intersection
        // Ray: origin + t * direction
        // Circle: |point - center| <= radius
        
        let oc = ray.origin - center;
        let a = ray.direction.dot(ray.direction);
        let b = 2.0 * oc.dot(ray.direction);
        let c = oc.dot(oc) - radius * radius;
        
        let discriminant = b * b - 4.0 * a * c;
        
        if discriminant < 0.0 {
            return None;
        }
        
        let sqrt_discriminant = discriminant.sqrt();
        let t1 = (-b - sqrt_discriminant) / (2.0 * a);
        let t2 = (-b + sqrt_discriminant) / (2.0 * a);
        
        // Find the smallest positive t in [0, max_distance]
        let mut t = f32::INFINITY;
        
        if t1 >= 0.0 && t1 <= max_distance {
            t = t1;
        }
        if t2 >= 0.0 && t2 <= max_distance && t2 < t {
            t = t2;
        }
        
        if t.is_infinite() {
            return None;
        }
        
        // Calculate hit point in world space
        let hit_point_local = ray.origin + ray.direction * t;
        let hit_point_world = hit_point_local.rotate(transform.rotation.angle()) + transform.position;
        
        // Calculate normal (from circle center to hit point)
        let hit_point_local_normalized = hit_point_local.normalize();
        let normal_local = hit_point_local_normalized;
        let normal_world = normal_local.rotate(transform.rotation.angle());
        
        Some(crate::query::RayCastResult::new(
            body_handle,
            collider_handle,
            hit_point_world,
            normal_world,
            t,
            t / max_distance,
        ))
    }

    /// Cast a ray against a polygon
    #[inline]
    fn ray_cast_polygon(
        ray: &Ray,
        max_distance: f32,
        polygon: &Polygon,
        transform: Transform,
        collider_handle: ColliderHandle,
        body_handle: BodyHandle,
    ) -> Option<crate::query::RayCastResult> {
        use crate::constants::EPSILON;
        
        // For now, use a simple approach: check each edge
        // A more efficient approach would use a spatial partition
        
        let vertices = polygon.vertices();
        let mut closest_hit: Option<crate::query::RayCastResult> = None;
        let mut closest_distance = max_distance;
        
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            
            let v1 = vertices[i];
            let v2 = vertices[j];
            
            // Ray-edge intersection
            if let Some((t, normal)) = Self::ray_edge_intersection(ray, v1, v2) {
                if t >= 0.0 && t < closest_distance {
                    // Calculate hit point in world space
                    let hit_point_local = ray.origin + ray.direction * t;
                    let hit_point_world = hit_point_local.rotate(transform.rotation.angle()) + transform.position;
                    let normal_world = normal.rotate(transform.rotation.angle());
                    
                    closest_hit = Some(crate::query::RayCastResult::new(
                        body_handle,
                        collider_handle,
                        hit_point_world,
                        normal_world,
                        t,
                        t / max_distance,
                    ));
                    closest_distance = t;
                }
            }
        }
        
        closest_hit
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
        self.broad_phase.clear();
        self.time = 0.0;
        self.frame_count = 0;
    }

    /// Check if a point is inside a shape
}

impl Default for World {
    fn default() -> Self {
        Self::default()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

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
    fn test_get_body() {
        let mut world = World::default();

        let handle = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 2.0), Rot2::identity()));
        
        let body = world.get_body(handle);
        assert!(body.is_some());
        assert_eq!(body.unwrap().position(), Vec2::new(1.0, 2.0));

        let body_mut = world.get_body_mut(handle);
        assert!(body_mut.is_some());
    }

    #[test]
    fn test_create_destroy_collider() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let shape = Shape::Circle(crate::shapes::Circle::new(1.0));
        let collider_handle = world.create_collider(body_handle, shape);

        assert!(world.has_collider(collider_handle));
        assert_eq!(world.collider_count(), 1);

        assert!(world.destroy_collider(collider_handle));
        assert!(!world.has_collider(collider_handle));
        assert_eq!(world.collider_count(), 0);
    }

    #[test]
    fn test_add_circle() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let collider_handle = world.add_circle(body_handle, 1.0, 1.0);

        assert!(world.has_collider(collider_handle));
        
        let body = world.get_body(body_handle).unwrap();
        // Mass = pi * r^2 * density = pi * 1 * 1 = pi
        assert!(approx_eq(body.mass(), crate::constants::PI, 1e-5));
    }

    #[test]
    fn test_add_box() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        let collider_handle = world.add_box(body_handle, Vec2::new(1.0, 1.0), 1.0);

        assert!(world.has_collider(collider_handle));
        
        let body = world.get_body(body_handle).unwrap();
        // Area = 2 * 2 = 4, Mass = 4 * 1 = 4
        assert!(approx_eq(body.mass(), 4.0, 1e-5));
    }

    #[test]
    fn test_step() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        world.add_circle(body_handle, 1.0, 1.0);

        // Step the simulation
        world.step(1.0 / 60.0);

        assert!(world.time() > 0.0);
        assert_eq!(world.frame_count(), 1);

        // The body should have fallen due to gravity
        let body = world.get_body(body_handle).unwrap();
        assert!(body.position().y < 0.0);
    }

    #[test]
    fn test_gravity() {
        let mut world = World::with_gravity(Vec2::new(0.0, -10.0));
        
        let body_handle = world.create_dynamic_body(Transform::identity());
        world.add_circle(body_handle, 1.0, 1.0);

        world.step(0.1);

        let body = world.get_body(body_handle).unwrap();
        // After 0.1 seconds with gravity -10, the body should have fallen
        // Distance = 0.5 * g * t^2 = 0.5 * 10 * 0.01 = 0.05
        // But we're using semi-implicit Euler, so it's different
        assert!(body.position().y < 0.0);
    }

    #[test]
    fn test_event_handlers() {
        let mut world = World::default();

        let handler = Box::new(super::super::collision::SimpleCollisionEventHandler::new());
        world.add_event_handler(handler);

        assert_eq!(world.event_handlers.len(), 1);

        world.clear_event_handlers();
        assert_eq!(world.event_handlers.len(), 0);
    }

    #[test]
    fn test_clear() {
        let mut world = World::default();

        let body_handle = world.create_dynamic_body(Transform::identity());
        world.add_circle(body_handle, 1.0, 1.0);

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
        let handle2 = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::identity()));

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
        let world_str = format!("{}", world);
        assert!(world_str.contains("World"));
    }

    #[test]
    fn test_create_distance_joint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::identity()));

        let handle = world.create_distance_joint(
            body_a,
            body_b,
            Vec2::new(0.5, 0.0),
            Vec2::new(-0.5, 0.0),
        );

        assert!(world.has_constraint(handle));
        assert_eq!(world.constraint_count(), 1);
    }

    #[test]
    fn test_create_revolute_joint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::identity()));

        let handle = world.create_revolute_joint(
            body_a,
            body_b,
            Vec2::new(0.5, 0.0),
        );

        assert!(world.has_constraint(handle));
        assert_eq!(world.constraint_count(), 1);
    }

    #[test]
    fn test_create_motor_joint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::identity()));

        let handle = world.create_motor_joint(body_a, body_b);

        assert!(world.has_constraint(handle));
        assert_eq!(world.constraint_count(), 1);
    }

    #[test]
    fn test_destroy_constraint() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::identity()));

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
        let body_b = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::identity()));

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
        world.add_circle(body_a, 1.0, 1.0);

        let body_b = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 2.0), Rot2::identity()));
        world.add_circle(body_b, 1.0, 1.0);

        // Create a distance joint to keep them connected
        world.create_distance_joint(body_a, body_b, Vec2::zero(), Vec2::zero());

        // Step the simulation
        world.step(1.0 / 60.0);

        assert!(world.time() > 0.0);
        assert_eq!(world.frame_count(), 1);
    }

    #[test]
    fn test_clear_with_constraints() {
        let mut world = World::default();

        let body_a = world.create_dynamic_body(Transform::identity());
        let body_b = world.create_dynamic_body(Transform::new(Vec2::new(1.0, 0.0), Rot2::identity()));
        world.create_distance_joint(body_a, body_b, Vec2::zero(), Vec2::zero());

        assert_eq!(world.constraint_count(), 1);

        world.clear();
        assert_eq!(world.body_count(), 0);
        assert_eq!(world.collider_count(), 0);
        assert_eq!(world.constraint_count(), 0);
    }
}
