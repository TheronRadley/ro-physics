//! Ragdoll scene for the physics engine demo.
//!
//! This scene demonstrates the constraint system by creating a ragdoll
//! made of multiple bodies connected with joints.

use std::fmt;

use ro_physics::math::{Rot2, Transform, Vec2};
use ro_physics::bodies::BodyHandle;
use ro_physics::constraints::ConstraintHandle;
use ro_physics::world::World;

/// A ragdoll scene that demonstrates constraint system.
#[derive(Debug)]
pub struct RagdollScene {
    /// Head body
    pub head: BodyHandle,
    /// Torso body
    pub torso: BodyHandle,
    /// Left upper arm
    pub left_upper_arm: BodyHandle,
    /// Left lower arm
    pub left_lower_arm: BodyHandle,
    /// Right upper arm
    pub right_upper_arm: BodyHandle,
    /// Right lower arm
    pub right_lower_arm: BodyHandle,
    /// Left upper leg
    pub left_upper_leg: BodyHandle,
    /// Left lower leg
    pub left_lower_leg: BodyHandle,
    /// Right upper leg
    pub right_upper_leg: BodyHandle,
    /// Right lower leg
    pub right_lower_leg: BodyHandle,
    /// Constraints connecting the bodies
    pub constraints: Vec<ConstraintHandle>,
    /// Whether the ragdoll is assembled
    pub assembled: bool,
}

impl RagdollScene {
    /// Create a new empty ragdoll scene
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            head: BodyHandle::null(),
            torso: BodyHandle::null(),
            left_upper_arm: BodyHandle::null(),
            left_lower_arm: BodyHandle::null(),
            right_upper_arm: BodyHandle::null(),
            right_lower_arm: BodyHandle::null(),
            left_upper_leg: BodyHandle::null(),
            left_lower_leg: BodyHandle::null(),
            right_upper_leg: BodyHandle::null(),
            right_lower_leg: BodyHandle::null(),
            constraints: Vec::new(),
            assembled: false,
        }
    }

    /// Assemble the ragdoll in the given world
    #[inline]
    pub fn assemble(&mut self, world: &mut World) {
        if self.assembled {
            return;
        }

        // Clear any existing bodies
        self.dispose(world);

        // Create all body parts
        
        // Torso (center of the ragdoll)
        self.torso = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 10.0),Rot2::new(0.0)));
        let _ = world.add_box(self.torso, Vec2::new(0.4, 0.6), 10.0);

        // Head
        self.head = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 10.8),Rot2::new(0.0)));
        let _ = world.add_circle(self.head, 0.3, 10.0);

        // Left upper arm
        self.left_upper_arm = world.create_dynamic_body(Transform::new(Vec2::new(-0.5, 9.5),Rot2::new(0.0)));
        let _ = world.add_box(self.left_upper_arm, Vec2::new(0.15, 0.4), 10.0);

        // Left lower arm
        self.left_lower_arm = world.create_dynamic_body(Transform::new(Vec2::new(-0.5, 8.8),Rot2::new(0.0)));
        let _ = world.add_box(self.left_lower_arm, Vec2::new(0.15, 0.4), 10.0);

        // Right upper arm
        self.right_upper_arm = world.create_dynamic_body(Transform::new(Vec2::new(0.5, 9.5),Rot2::new(0.0)));
        let _ = world.add_box(self.right_upper_arm, Vec2::new(0.15, 0.4), 10.0);

        // Right lower arm
        self.right_lower_arm = world.create_dynamic_body(Transform::new(Vec2::new(0.5, 8.8),Rot2::new(0.0)));
        let _ = world.add_box(self.right_lower_arm, Vec2::new(0.15, 0.4), 10.0);

        // Left upper leg
        self.left_upper_leg = world.create_dynamic_body(Transform::new(Vec2::new(-0.2, 9.0),Rot2::new(0.0)));
        let _ = world.add_box(self.left_upper_leg, Vec2::new(0.15, 0.5), 10.0);

        // Left lower leg
        self.left_lower_leg = world.create_dynamic_body(Transform::new(Vec2::new(-0.2, 8.2),Rot2::new(0.0)));
        let _ = world.add_box(self.left_lower_leg, Vec2::new(0.15, 0.5), 10.0);

        // Right upper leg
        self.right_upper_leg = world.create_dynamic_body(Transform::new(Vec2::new(0.2, 9.0),Rot2::new(0.0)));
        let _ = world.add_box(self.right_upper_leg, Vec2::new(0.15, 0.5), 10.0);

        // Right lower leg
        self.right_lower_leg = world.create_dynamic_body(Transform::new(Vec2::new(0.2, 8.2),Rot2::new(0.0)));
        let _ = world.add_box(self.right_lower_leg, Vec2::new(0.15, 0.5), 10.0);

        // Create constraints to connect the ragdoll
        
        // Head to torso (revolute joint at neck)
        let neck_joint = world.create_revolute_joint(
            self.head,
            self.torso,
            Vec2::new(0.0, -0.3), // Anchor on head (bottom center)
        );
        self.constraints.push(neck_joint);

        // Left shoulder (revolute joint)
        let left_shoulder = world.create_revolute_joint(
            self.torso,
            self.left_upper_arm,
            Vec2::new(-0.4, -0.2), // Anchor on torso (left shoulder)
        );
        self.constraints.push(left_shoulder);

        // Left elbow (revolute joint)
        let left_elbow = world.create_revolute_joint(
            self.left_upper_arm,
            self.left_lower_arm,
            Vec2::new(0.0, -0.4), // Anchor on upper arm (bottom center)
        );
        self.constraints.push(left_elbow);

        // Right shoulder (revolute joint)
        let right_shoulder = world.create_revolute_joint(
            self.torso,
            self.right_upper_arm,
            Vec2::new(0.4, -0.2), // Anchor on torso (right shoulder)
        );
        self.constraints.push(right_shoulder);

        // Right elbow (revolute joint)
        let right_elbow = world.create_revolute_joint(
            self.right_upper_arm,
            self.right_lower_arm,
            Vec2::new(0.0, -0.4), // Anchor on upper arm (bottom center)
        );
        self.constraints.push(right_elbow);

        // Left hip (revolute joint)
        let left_hip = world.create_revolute_joint(
            self.torso,
            self.left_upper_leg,
            Vec2::new(-0.2, -0.6), // Anchor on torso (left hip)
        );
        self.constraints.push(left_hip);

        // Left knee (revolute joint)
        let left_knee = world.create_revolute_joint(
            self.left_upper_leg,
            self.left_lower_leg,
            Vec2::new(0.0, -0.5), // Anchor on upper leg (bottom center)
        );
        self.constraints.push(left_knee);

        // Right hip (revolute joint)
        let right_hip = world.create_revolute_joint(
            self.torso,
            self.right_upper_leg,
            Vec2::new(0.2, -0.6), // Anchor on torso (right hip)
        );
        self.constraints.push(right_hip);

        // Right knee (revolute joint)
        let right_knee = world.create_revolute_joint(
            self.right_upper_leg,
            self.right_lower_leg,
            Vec2::new(0.0, -0.5), // Anchor on upper leg (bottom center)
        );
        self.constraints.push(right_knee);

        self.assembled = true;
    }

    /// Dispose of the ragdoll bodies and constraints
    #[inline]
    pub fn dispose(&mut self, world: &mut World) {
        // Destroy constraints first
        for &constraint in &self.constraints {
            world.destroy_constraint(constraint);
        }
        self.constraints.clear();

        // Destroy bodies
        let bodies = [
            self.head,
            self.torso,
            self.left_upper_arm,
            self.left_lower_arm,
            self.right_upper_arm,
            self.right_lower_arm,
            self.left_upper_leg,
            self.left_lower_leg,
            self.right_upper_leg,
            self.right_lower_leg,
        ];

        for body in bodies {
            if !body.is_null() {
                world.destroy_body(body);
            }
        }

        // Reset handles
        self.head = BodyHandle::null();
        self.torso = BodyHandle::null();
        self.left_upper_arm = BodyHandle::null();
        self.left_lower_arm = BodyHandle::null();
        self.right_upper_arm = BodyHandle::null();
        self.right_lower_arm = BodyHandle::null();
        self.left_upper_leg = BodyHandle::null();
        self.left_lower_leg = BodyHandle::null();
        self.right_upper_leg = BodyHandle::null();
        self.right_lower_leg = BodyHandle::null();

        self.assembled = false;
    }

    /// Apply a force to the torso (e.g., from user input)
    #[inline]
    pub fn apply_torso_force(&self, world: &mut World, force: Vec2) {
        if self.assembled && !self.torso.is_null() {
            if let Some(body) = world.get_body_mut(self.torso) {
                body.add_force(force);
            }
        }
    }

    /// Apply an impulse to a specific body part
    #[inline]
    pub fn apply_impulse(&self, world: &mut World, body_part: BodyHandle, impulse: Vec2) {
        if self.assembled && !body_part.is_null() {
            if let Some(body) = world.get_body_mut(body_part) {
                body.add_impulse(impulse);
            }
        }
    }

    /// Get all body handles
    #[inline]
    #[must_use]
    pub fn body_handles(&self) -> Vec<BodyHandle> {
        vec![
            self.head,
            self.torso,
            self.left_upper_arm,
            self.left_lower_arm,
            self.right_upper_arm,
            self.right_lower_arm,
            self.left_upper_leg,
            self.left_lower_leg,
            self.right_upper_leg,
            self.right_lower_leg,
        ]
    }

    /// Check if the ragdoll is assembled
    #[inline]
    #[must_use]
    pub const fn is_assembled(&self) -> bool {
        self.assembled
    }

    /// Get the number of constraints
    #[inline]
    #[must_use]
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }

    /// Get the number of body parts
    #[inline]
    #[must_use]
    pub fn body_count(&self) -> usize {
        let mut count = 0;
        let bodies = self.body_handles();
        for body in bodies {
            if !body.is_null() {
                count += 1;
            }
        }
        count
    }
}

impl Default for RagdollScene {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for RagdollScene {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "RagdollScene(assembled: {}, bodies: {}, constraints: {})",
            self.assembled,
            self.body_count(),
            self.constraint_count()
        )
    }
}

/// Scene trait for the demo framework
pub trait Scene {
    /// Initialize the scene
    fn init(&mut self, world: &mut World);

    /// Update the scene
    fn update(&mut self, world: &mut World, dt: f32);

    /// Get the scene name
    fn name(&self) -> &str;

    /// Get the scene description
    fn description(&self) -> &str;

    /// Clean up the scene
    fn cleanup(&mut self, world: &mut World);
}

impl Scene for RagdollScene {
    fn init(&mut self, world: &mut World) {
        self.assemble(world);
    }

    fn update(&mut self, _world: &mut World, _dt: f32) {
        // The ragdoll is driven entirely by the simulation: joints hold it
        // together and gravity does the rest, so there is nothing to do here.
    }

    fn name(&self) -> &str {
        "Ragdoll"
    }

    fn description(&self) -> &str {
        "A ragdoll made of multiple bodies connected with revolute joints"
    }

    fn cleanup(&mut self, world: &mut World) {
        self.dispose(world);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use World;

    #[test]
    fn test_ragdoll_creation() {
        let scene = RagdollScene::new();
        assert!(!scene.is_assembled());
        assert_eq!(scene.body_count(), 0);
        assert_eq!(scene.constraint_count(), 0);
    }

    #[test]
    fn test_ragdoll_assemble() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        scene.assemble(&mut world);
        
        assert!(scene.is_assembled());
        assert_eq!(scene.body_count(), 10); // 10 body parts
        assert_eq!(scene.constraint_count(), 10); // 10 joints
        assert_eq!(world.body_count(), 10);
        assert_eq!(world.constraint_count(), 10);
    }

    #[test]
    fn test_ragdoll_dispose() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        scene.assemble(&mut world);
        assert!(scene.is_assembled());
        
        scene.dispose(&mut world);
        
        assert!(!scene.is_assembled());
        assert_eq!(scene.body_count(), 0);
        assert_eq!(scene.constraint_count(), 0);
        assert_eq!(world.body_count(), 0);
        assert_eq!(world.constraint_count(), 0);
    }

    #[test]
    fn test_ragdoll_scene_trait() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        // Test Scene trait methods
        scene.init(&mut world);
        assert!(scene.is_assembled());
        
        scene.update(&mut world, 1.0 / 60.0);
        
        assert_eq!(scene.name(), "Ragdoll");
        assert!(scene.description().contains("ragdoll"));
        
        scene.cleanup(&mut world);
        assert!(!scene.is_assembled());
    }

    #[test]
    fn test_ragdoll_display() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        scene.assemble(&mut world);
        
        let scene_str = format!("{}", scene);
        assert!(scene_str.contains("RagdollScene"));
        assert!(scene_str.contains("assembled: true"));
    }

    #[test]
    fn test_ragdoll_body_handles() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        scene.assemble(&mut world);
        
        let handles = scene.body_handles();
        assert_eq!(handles.len(), 10);
        
        // All handles should be valid (not null)
        for handle in handles {
            assert!(!handle.is_null());
        }
    }

    #[test]
    fn test_ragdoll_apply_force() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        scene.assemble(&mut world);
        
        let torso = world.get_body(scene.torso).unwrap();
        let initial_force = torso.force();
        
        scene.apply_torso_force(&mut world, Vec2::new(0.0, 100.0));
        
        let torso_after = world.get_body(scene.torso).unwrap();
        assert_ne!(torso_after.force(), initial_force);
    }

    #[test]
    fn test_ragdoll_apply_impulse() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        scene.assemble(&mut world);
        
        let head = world.get_body(scene.head).unwrap();
        let initial_velocity = head.linear_velocity();
        
        scene.apply_impulse(&mut world, scene.head, Vec2::new(10.0, 0.0));
        
        let head_after = world.get_body(scene.head).unwrap();
        assert_ne!(head_after.linear_velocity(), initial_velocity);
    }

    #[test]
    fn test_ragdoll_simulation() {
        let mut world = World::default();
        let mut scene = RagdollScene::new();
        
        scene.assemble(&mut world);
        
        // Step the simulation
        for _ in 0..10 {
            scene.update(&mut world, 1.0 / 60.0);
            world.step(1.0 / 60.0);
        }
        
        // All bodies should still exist
        assert_eq!(world.body_count(), 10);
        assert_eq!(world.constraint_count(), 10);
        
        // Bodies should have moved (fallen due to gravity)
        let head = world.get_body(scene.head).unwrap();
        assert!(head.position().y < 10.8); // Started at y=10.8
    }
}
