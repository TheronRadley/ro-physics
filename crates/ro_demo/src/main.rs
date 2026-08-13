//! Ro-Physics Demo Application
//!
//! This is a demonstration application for the Ro-Physics Engine.
//! It showcases various physics scenarios with live rendering.

use std::time::{Duration, Instant};

use winit::{
    event::{Event, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

use ro_physics::prelude::*;
use ro_physics::world::World;

mod renderer;
mod scenes;
use renderer::{Camera, DebugFlags, Renderer};
use scenes::{RagdollScene, create_stress_scene};

/// Demo application state
struct DemoApp {
    /// Physics world
    world: World,
    /// Renderer
    renderer: Option<Renderer>,
    /// Last frame time
    last_frame_time: Instant,
    /// Frame counter
    frame_count: u64,
    /// Current FPS
    fps: f32,
    /// Last FPS update time
    last_fps_update: Instant,
    /// Current scene index
    current_scene: usize,
    /// Available scenes
    scenes: Vec<Scene>,
    /// Whether the simulation is paused
    paused: bool,
    /// Whether to single-step
    single_step: bool,
    /// Debug flags
    debug_flags: DebugFlags,
    /// Camera
    camera: Camera,
    /// Camera zoom level
    zoom: f32,
    /// Camera target position
    camera_target: Vec2,
}

impl DemoApp {
    /// Create a new demo application
    fn new() -> Self {
        let world = World::with_gravity(Vec2::new(0.0, -9.81));

        let scenes = vec![
            Scene::new("Gravity", create_gravity_scene),
            Scene::new("Stacking", create_stacking_scene),
            Scene::new("Dominoes", create_dominoes_scene),
            Scene::new("Bouncy", create_bouncy_scene),
            Scene::new("Polygons", create_polygon_scene),
            Scene::new("Ragdoll", create_ragdoll_scene),
            Scene::new("Stress Lab", create_stress_scene),
        ];

        Self {
            world,
            renderer: None,
            last_frame_time: Instant::now(),
            frame_count: 0,
            fps: 0.0,
            last_fps_update: Instant::now(),
            current_scene: 0,
            scenes,
            paused: false,
            single_step: false,
            debug_flags: DebugFlags::default(),
            camera: Camera::default(),
            zoom: 1.0,
            camera_target: Vec2::zero(),
        }
    }

    /// Load the current scene
    fn load_current_scene(&mut self) {
        if self.current_scene < self.scenes.len() {
            self.world.clear();
            (self.scenes[self.current_scene].creator)(&mut self.world);
        }
    }

    /// Step the simulation
    fn step(&mut self) {
        if self.paused && !self.single_step {
            return;
        }

        self.single_step = false;

        // Fixed timestep
        const FIXED_DT: f32 = 1.0 / 60.0;
        self.world.step(FIXED_DT);
    }

    /// Update FPS counter
    fn update_fps(&mut self) {
        self.frame_count += 1;

        let now = Instant::now();
        let elapsed = now.duration_since(self.last_fps_update);

        if elapsed >= Duration::from_secs(1) {
            self.fps = self.frame_count as f32 / elapsed.as_secs_f32();
            self.frame_count = 0;
            self.last_fps_update = now;
        }
    }

    /// Handle keyboard input
    fn handle_keyboard(&mut self, key: winit::event::VirtualKeyCode) {
        match key {
            winit::event::VirtualKeyCode::Space => {
                self.paused = !self.paused;
            }
            winit::event::VirtualKeyCode::R => {
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::T => {
                self.single_step = true;
            }
            winit::event::VirtualKeyCode::Key1 => {
                self.current_scene = 0;
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::Key2 => {
                self.current_scene = 1;
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::Key3 => {
                self.current_scene = 2;
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::Key4 => {
                self.current_scene = 3;
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::Key5 => {
                self.current_scene = 4;
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::Key6 => {
                self.current_scene = 5;
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::Key7 => {
                self.current_scene = 6;
                self.load_current_scene();
            }
            winit::event::VirtualKeyCode::D => {
                self.debug_flags.show_aabbs = !self.debug_flags.show_aabbs;
            }
            winit::event::VirtualKeyCode::C => {
                self.debug_flags.show_colliders = !self.debug_flags.show_colliders;
            }
            winit::event::VirtualKeyCode::V => {
                self.debug_flags.show_velocities = !self.debug_flags.show_velocities;
            }
            winit::event::VirtualKeyCode::S => {
                self.debug_flags.show_sleeping = !self.debug_flags.show_sleeping;
            }
            winit::event::VirtualKeyCode::Plus => {
                self.zoom *= 1.1;
            }
            winit::event::VirtualKeyCode::Minus => {
                self.zoom /= 1.1;
            }
            winit::event::VirtualKeyCode::Up => {
                self.camera_target.y += 0.5 / self.zoom;
            }
            winit::event::VirtualKeyCode::Down => {
                self.camera_target.y -= 0.5 / self.zoom;
            }
            winit::event::VirtualKeyCode::Left => {
                self.camera_target.x -= 0.5 / self.zoom;
            }
            winit::event::VirtualKeyCode::Right => {
                self.camera_target.x += 0.5 / self.zoom;
            }
            winit::event::VirtualKeyCode::Escape => {
                // Quit
            }
            _ => {}
        }
    }

    /// Update camera
    fn update_camera(&mut self) {
        // Smooth camera follow
        let current_center = self.calculate_world_center();
        let target_x = current_center.x.lerp(self.camera_target.x, 0.05);
        let target_y = current_center.y.lerp(self.camera_target.y, 0.05);
        
        self.camera = Camera::new(Vec2::new(target_x, target_y), self.zoom);
    }

    /// Calculate the center of the world
    fn calculate_world_center(&self) -> Vec2 {
        let mut min_x = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        for body_handle in self.world.body_handles() {
            if let Some(body) = self.world.get_body(body_handle) {
                let pos = body.position();
                min_x = min_x.min(pos.x);
                max_x = max_x.max(pos.x);
                min_y = min_y.min(pos.y);
                max_y = max_y.max(pos.y);
            }
        }

        if min_x.is_finite() {
            Vec2::new((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
        } else {
            Vec2::zero()
        }
    }
}

/// A physics scene
struct Scene {
    /// Scene name
    name: String,
    /// Scene creator function
    creator: fn(&mut World),
}

impl Scene {
    /// Create a new scene
    fn new(name: &str, creator: fn(&mut World)) -> Self {
        Self {
            name: name.to_string(),
            creator,
        }
    }
}

/// Create a gravity test scene
fn create_gravity_scene(world: &mut World) {
    // Create ground
    let ground = world.create_static_body(Transform::identity());
    world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create some falling circles
    for i in 0..5 {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(-5.0 + i as f32 * 3.0, 10.0),
            0.0,
        ));
        world.add_circle(body, 1.0, 1.0);
    }

    // Create some falling boxes
    for i in 0..5 {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(-5.0 + i as f32 * 3.0, 15.0),
            0.0,
        ));
        world.add_box(body, Vec2::new(1.0, 1.0), 1.0);
    }
}

/// Create a stacking test scene
fn create_stacking_scene(world: &mut World) {
    // Create ground
    let ground = world.create_static_body(Transform::identity());
    world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create a stack of boxes
    let mut y = 1.0;
    for _i in 0..10 {
        let body = world.create_dynamic_body(Transform::new(Vec2::new(0.0, y), 0.0));
        world.add_box(body, Vec2::new(1.0, 0.5), 1.0);
        y += 1.0;
    }
}

/// Create a dominoes test scene
fn create_dominoes_scene(world: &mut World) {
    // Create ground
    let ground = world.create_static_body(Transform::identity());
    world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create a line of dominoes
    for i in 0..20 {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(-10.0 + i as f32 * 1.2, 0.75),
            0.0,
        ));
        world.add_box(body, Vec2::new(0.2, 1.0), 1.0);
    }

    // Add a ball to knock them over
    let ball = world.create_dynamic_body(Transform::new(Vec2::new(-15.0, 5.0), 0.0));
    world.add_circle(ball, 1.0, 1.0);
    if let Some(body) = world.get_body_mut(ball) {
        body.set_linear_velocity(Vec2::new(10.0, 0.0));
    }
}

/// Create a bouncy test scene
fn create_bouncy_scene(world: &mut World) {
    // Create ground
    let ground = world.create_static_body(Transform::identity());
    world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create walls
    let left_wall = world.create_static_body(Transform::new(Vec2::new(-10.0, 5.0), 0.0));
    world.add_box(left_wall, Vec2::new(0.5, 10.0), 0.0);

    let right_wall = world.create_static_body(Transform::new(Vec2::new(10.0, 5.0), 0.0));
    world.add_box(right_wall, Vec2::new(0.5, 10.0), 0.0);

    // Create bouncy balls
    for i in 0..10 {
        let body = world.create_dynamic_body(Transform::new(
            Vec2::new(-8.0 + i as f32 * 2.0, 15.0),
            0.0,
        ));
        let collider = world.add_circle(body, 1.0, 1.0);
        
        // Set high restitution
        if let Some(collider) = world.get_collider_mut(collider) {
            collider.set_restitution(0.9);
        }
    }
}

/// Create a polygon test scene
fn create_polygon_scene(world: &mut World) {
    // Create ground
    let ground = world.create_static_body(Transform::identity());
    world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create a triangle
    let triangle_vertices = vec![
        Vec2::new(-1.0, -1.0),
        Vec2::new(1.0, -1.0),
        Vec2::new(0.0, 1.0),
    ];
    let triangle = world.create_dynamic_body(Transform::new(Vec2::new(-5.0, 5.0), 0.0));
    world.add_polygon(triangle, triangle_vertices, 1.0);

    // Create a hexagon
    let mut hexagon_vertices = Vec::new();
    for i in 0..6 {
        let angle = std::f32::consts::PI / 3.0 * i as f32;
        hexagon_vertices.push(Vec2::new(angle.cos(), angle.sin()));
    }
    let hexagon = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 5.0), 0.0));
    world.add_polygon(hexagon, hexagon_vertices, 1.0);

    // Create a pentagon
    let mut pentagon_vertices = Vec::new();
    for i in 0..5 {
        let angle = std::f32::consts::PI * 2.0 / 5.0 * i as f32;
        pentagon_vertices.push(Vec2::new(angle.cos() * 1.5, angle.sin() * 1.5));
    }
    let pentagon = world.create_dynamic_body(Transform::new(Vec2::new(5.0, 5.0), 0.0));
    world.add_polygon(pentagon, pentagon_vertices, 1.0);
}

/// Create a ragdoll scene with constraints
fn create_ragdoll_scene(world: &mut World) {
    // Create ground
    let ground = world.create_static_body(Transform::identity());
    world.add_box(ground, Vec2::new(20.0, 0.5), 0.0);

    // Create torso (center of the ragdoll)
    let torso = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 10.0), 0.0));
    world.add_box(torso, Vec2::new(0.4, 0.6), 10.0);

    // Create head
    let head = world.create_dynamic_body(Transform::new(Vec2::new(0.0, 10.8), 0.0));
    world.add_circle(head, 0.3, 10.0);

    // Create left upper arm
    let left_upper_arm = world.create_dynamic_body(Transform::new(Vec2::new(-0.5, 9.5), 0.0));
    world.add_box(left_upper_arm, Vec2::new(0.15, 0.4), 10.0);

    // Create left lower arm
    let left_lower_arm = world.create_dynamic_body(Transform::new(Vec2::new(-0.5, 8.8), 0.0));
    world.add_box(left_lower_arm, Vec2::new(0.15, 0.4), 10.0);

    // Create right upper arm
    let right_upper_arm = world.create_dynamic_body(Transform::new(Vec2::new(0.5, 9.5), 0.0));
    world.add_box(right_upper_arm, Vec2::new(0.15, 0.4), 10.0);

    // Create right lower arm
    let right_lower_arm = world.create_dynamic_body(Transform::new(Vec2::new(0.5, 8.8), 0.0));
    world.add_box(right_lower_arm, Vec2::new(0.15, 0.4), 10.0);

    // Create left upper leg
    let left_upper_leg = world.create_dynamic_body(Transform::new(Vec2::new(-0.2, 9.0), 0.0));
    world.add_box(left_upper_leg, Vec2::new(0.15, 0.5), 10.0);

    // Create left lower leg
    let left_lower_leg = world.create_dynamic_body(Transform::new(Vec2::new(-0.2, 8.2), 0.0));
    world.add_box(left_lower_leg, Vec2::new(0.15, 0.5), 10.0);

    // Create right upper leg
    let right_upper_leg = world.create_dynamic_body(Transform::new(Vec2::new(0.2, 9.0), 0.0));
    world.add_box(right_upper_leg, Vec2::new(0.15, 0.5), 10.0);

    // Create right lower leg
    let right_lower_leg = world.create_dynamic_body(Transform::new(Vec2::new(0.2, 8.2), 0.0));
    world.add_box(right_lower_leg, Vec2::new(0.15, 0.5), 10.0);

    // Create constraints to connect the ragdoll
    
    // Head to torso (revolute joint at neck)
    world.create_revolute_joint(
        head,
        torso,
        Vec2::new(0.0, -0.3), // Anchor on head (bottom center)
    );

    // Left shoulder (revolute joint)
    world.create_revolute_joint(
        torso,
        left_upper_arm,
        Vec2::new(-0.4, -0.2), // Anchor on torso (left shoulder)
    );

    // Left elbow (revolute joint)
    world.create_revolute_joint(
        left_upper_arm,
        left_lower_arm,
        Vec2::new(0.0, -0.4), // Anchor on upper arm (bottom center)
    );

    // Right shoulder (revolute joint)
    world.create_revolute_joint(
        torso,
        right_upper_arm,
        Vec2::new(0.4, -0.2), // Anchor on torso (right shoulder)
    );

    // Right elbow (revolute joint)
    world.create_revolute_joint(
        right_upper_arm,
        right_lower_arm,
        Vec2::new(0.0, -0.4), // Anchor on upper arm (bottom center)
    );

    // Left hip (revolute joint)
    world.create_revolute_joint(
        torso,
        left_upper_leg,
        Vec2::new(-0.2, -0.6), // Anchor on torso (left hip)
    );

    // Left knee (revolute joint)
    world.create_revolute_joint(
        left_upper_leg,
        left_lower_leg,
        Vec2::new(0.0, -0.5), // Anchor on upper leg (bottom center)
    );

    // Right hip (revolute joint)
    world.create_revolute_joint(
        torso,
        right_upper_leg,
        Vec2::new(0.2, -0.6), // Anchor on torso (right hip)
    );

    // Right knee (revolute joint)
    world.create_revolute_joint(
        right_upper_leg,
        right_lower_leg,
        Vec2::new(0.0, -0.5), // Anchor on upper leg (bottom center)
    );

    // Add some initial velocity to the torso to make it interesting
    if let Some(body) = world.get_body_mut(torso) {
        body.set_linear_velocity(Vec2::new(2.0, 0.0));
    }
}

fn main() {
    // Initialize logging
    env_logger::init();

    // Create event loop
    let event_loop = EventLoop::new().unwrap();

    // Create window
    let window = WindowBuilder::new()
        .with_title("Ro-Physics Demo")
        .with_inner_size(winit::dpi::LogicalSize::new(1280.0, 720.0))
        .build(&event_loop)
        .unwrap();

    // Create demo app
    let mut app = DemoApp::new();
    app.load_current_scene();

    // Initialize renderer (async)
    let mut app = pollster::block_on(async {
        let renderer = Renderer::new(&window).await;
        app.renderer = Some(renderer);
        app
    });

    // Main loop
    event_loop.run(move |event, elwt| {
        match event {
            Event::WindowEvent { event, .. } => {
                match event {
                    WindowEvent::CloseRequested => {
                        elwt.exit();
                    }
                    WindowEvent::KeyboardInput { input, .. } => {
                        if let Some(key) = input.virtual_keycode {
                            app.handle_keyboard(key);
                        }
                    }
                    WindowEvent::Resized(size) => {
                        if let Some(renderer) = &mut app.renderer {
                            renderer.resize(size);
                        }
                    }
                    _ => {}
                }
            }
            Event::MainEventsCleared => {
                // Update FPS
                app.update_fps();

                // Update camera
                app.update_camera();

                // Step simulation
                app.step();

                // Prepare world for rendering
                if let Some(renderer) = &mut app.renderer {
                    // Update renderer debug flags
                    renderer.set_debug_flags(app.debug_flags.clone());
                    renderer.set_camera(app.camera.clone());
                    
                    // Prepare the world
                    renderer.prepare_world(&app.world);
                }

                // Request redraw
                window.request_redraw();
            }
            Event::RedrawRequested(_) => {
                // Render
                if let Some(renderer) = &mut app.renderer {
                    match renderer.render() {
                        Ok(_) => {}
                        Err(e) => {
                            eprintln!("Render error: {:?}", e);
                            elwt.exit();
                        }
                    }
                }

                // Print stats every 60 frames
                if app.frame_count % 60 == 0 {
                    println!(
                        "FPS: {:.1}, Bodies: {}, Colliders: {}, Time: {:.2}, Scene: {}",
                        app.fps,
                        app.world.body_count(),
                        app.world.collider_count(),
                        app.world.time(),
                        app.scenes[app.current_scene].name
                    );
                }
            }
            _ => {}
        }
    }).unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scene_creation() {
        let mut world = World::with_gravity(Vec2::new(0.0, -9.81));
        
        create_gravity_scene(&mut world);
        assert!(world.body_count() > 0);

        world.clear();
        create_stacking_scene(&mut world);
        assert!(world.body_count() > 0);

        world.clear();
        create_dominoes_scene(&mut world);
        assert!(world.body_count() > 0);

        world.clear();
        create_bouncy_scene(&mut world);
        assert!(world.body_count() > 0);

        world.clear();
        create_polygon_scene(&mut world);
        assert!(world.body_count() > 0);
    }

    #[test]
    fn test_camera() {
        let camera = Camera::new(Vec2::new(1.0, 2.0), 1.5);
        assert_eq!(camera.position, Vec2::new(1.0, 2.0));
        assert_eq!(camera.scale, 1.5);

        let matrix = camera.get_view_projection(800.0, 600.0);
        assert!(!matrix[0][0].is_nan());
    }

    #[test]
    fn test_debug_flags() {
        let flags = DebugFlags::default();
        assert!(!flags.show_aabbs);
        assert!(flags.show_colliders);
    }
}
