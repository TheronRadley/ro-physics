//! Renderer for the Ro-Physics Demo.
//!
//! This module provides rendering functionality for visualizing the physics simulation.

use std::sync::Arc;

use wgpu::util::DeviceExt;
use winit::window::Window;

use ro_physics::geometry::Aabb;
use ro_physics::prelude::*;
use ro_physics::shapes::Shape;

/// Vertex for rendering
#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 3],
}

impl Vertex {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                // Position
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                // Color
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

/// Renderer state
pub struct Renderer {
    // The surface owns a clone of the window's `Arc`, so it stays valid
    // for as long as the renderer does.
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    size: winit::dpi::PhysicalSize<u32>,

    // Render pipeline
    render_pipeline: wgpu::RenderPipeline,

    // Vertex buffer
    vertex_buffer: wgpu::Buffer,
    vertices: Vec<Vertex>,

    // Index buffer
    index_buffer: wgpu::Buffer,
    indices: Vec<u16>,

    // Camera
    camera: Camera,

    // Debug flags
    debug_flags: DebugFlags,
}

/// Camera for 2D rendering
#[derive(Debug, Clone)]
pub struct Camera {
    pub position: Vec2,
    pub scale: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            position: Vec2::zero(),
            scale: 1.0,
        }
    }
}

impl Camera {
    /// Create a new camera
    #[must_use]
    pub fn new(position: Vec2, scale: f32) -> Self {
        Self { position, scale }
    }

    /// Get the view projection matrix
    ///
    /// `scale` is the half-height of the view in world units, so zooming in
    /// means making it smaller. This matches what the renderer does to its
    /// vertices on the CPU.
    #[must_use]
    pub fn get_view_projection(&self, width: f32, height: f32) -> [[f32; 4]; 4] {
        let scale = self.scale.abs().max(f32::EPSILON);
        let aspect = (width / height).abs().max(f32::EPSILON);

        // 2D orthographic projection
        let scale_x = 1.0 / (scale * aspect);
        let scale_y = 1.0 / scale;

        let translate_x = -self.position.x * scale_x;
        let translate_y = -self.position.y * scale_y;

        [
            [scale_x, 0.0, 0.0, 0.0],
            [0.0, scale_y, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [translate_x, translate_y, 0.0, 1.0],
        ]
    }
}

/// Debug flags for rendering
#[derive(Debug, Clone)]
pub struct DebugFlags {
    pub show_aabbs: bool,
    pub show_contacts: bool,
    pub show_velocities: bool,
    pub show_sleeping: bool,
    pub show_colliders: bool,
}

impl Default for DebugFlags {
    fn default() -> Self {
        Self {
            show_aabbs: false,
            show_contacts: false,
            show_velocities: false,
            show_sleeping: false,
            show_colliders: true,
        }
    }
}

impl Renderer {
    /// Create a new renderer
    pub async fn new(window: Arc<Window>) -> Self {
        // Instance
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        // Surface. Passing an `Arc<Window>` keeps the window alive for as
        // long as the surface exists, which is what `Surface<'static>` needs.
        let surface = instance
            .create_surface(window.clone())
            .expect("failed to create a surface for the window");

        // Adapter
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();

        // Device and queue
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                },
                None,
            )
            .await
            .unwrap();

        // Surface configuration
        let size = window.inner_size();
        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(surface_caps.formats[0]);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: surface_caps.present_modes[0],
            desired_maximum_frame_latency: 2,
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
        };

        surface.configure(&device, &config);

        // Shader module
        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        // Render pipeline
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[],
                push_constant_ranges: &[],
            });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader_module,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader_module,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        // Initial vertex and index buffers
        let vertices = Vec::new();
        let indices = Vec::new();

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Index Buffer"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        Self {
            surface,
            device,
            queue,
            config,
            size,
            render_pipeline,
            vertex_buffer,
            vertices,
            index_buffer,
            indices,
            camera: Camera::default(),
            debug_flags: DebugFlags::default(),
        }
    }

    /// Push a vertex in world space, applying the camera
    ///
    /// The pipeline has no uniform buffer yet, so the view transform is
    /// folded into the vertices on the CPU: `scale` is the half-height of
    /// the view in world units, and the width follows from the aspect
    /// ratio.
    fn push_vertex(&mut self, world_x: f32, world_y: f32, color: [f32; 3]) {
        let width = self.size.width.max(1) as f32;
        let height = self.size.height.max(1) as f32;
        let aspect = width / height;

        let scale_x = 1.0 / (self.camera.scale.abs().max(f32::EPSILON) * aspect);
        let scale_y = 1.0 / self.camera.scale.abs().max(f32::EPSILON);

        self.vertices.push(Vertex {
            position: [
                (world_x - self.camera.position.x) * scale_x,
                (world_y - self.camera.position.y) * scale_y,
                0.0,
            ],
            color,
        });
    }

    /// Push a line segment in world space (drawn as two triangles)
    fn push_line(&mut self, from: Vec2, to: Vec2, color: [f32; 3], thickness: f32) {
        let direction = to - from;
        let length = direction.length();
        if length <= f32::EPSILON {
            return;
        }

        let along = direction / length;
        let across = Vec2::new(-along.y, along.x) * (thickness * 0.5);

        let start_idx = self.vertices.len() as u16;
        for corner in [from + across, from - across, to - across, to + across] {
            self.push_vertex(corner.x, corner.y, color);
        }

        self.indices.push(start_idx);
        self.indices.push(start_idx + 1);
        self.indices.push(start_idx + 2);

        self.indices.push(start_idx);
        self.indices.push(start_idx + 2);
        self.indices.push(start_idx + 3);
    }

    /// Resize the renderer
    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
        }
    }

    /// Update the camera
    pub fn set_camera(&mut self, camera: Camera) {
        self.camera = camera;
    }

    /// Update debug flags
    pub fn set_debug_flags(&mut self, flags: DebugFlags) {
        self.debug_flags = flags;
    }

    /// Prepare the world for rendering
    pub fn prepare_world(&mut self, world: &World) {
        self.vertices.clear();
        self.indices.clear();

        // Add shapes from all colliders
        for collider_handle in world.collider_handles() {
            if let Some(collider) = world.get_collider(collider_handle) {
                if let Some(body) = world.get_body(collider.body_handle()) {
                    let transform = body.transform();
                    let shape = collider.shape();

                    self.add_shape(shape, transform, body, collider);
                }
            }
        }

        // Add debug visualizations
        if self.debug_flags.show_aabbs {
            for collider_handle in world.collider_handles() {
                if let Some(collider) = world.get_collider(collider_handle) {
                    if let Some(body) = world.get_body(collider.body_handle()) {
                        let aabb = collider.aabb(body.transform());
                        self.add_aabb(aabb, [1.0, 0.0, 0.0]); // Red for AABBs
                    }
                }
            }
        }

        // Draw a short arrow for every moving body
        if self.debug_flags.show_velocities {
            let thickness = 0.02 * self.camera.scale;
            for handle in world.body_handles() {
                let Some(body) = world.get_body(handle) else {
                    continue;
                };

                let velocity = body.linear_velocity();
                if velocity.length_squared() <= 1.0e-6 {
                    continue;
                }

                let from = body.position();
                self.push_line(from, from + velocity * 0.25, [1.0, 1.0, 0.0], thickness);
            }
        }

        // Mark every contact point with a small cross
        if self.debug_flags.show_contacts {
            let thickness = 0.02 * self.camera.scale;
            let arm = 0.05 * self.camera.scale;
            for manifold in world.manifolds() {
                for contact in manifold.contacts() {
                    let point = contact.position();
                    let colour = [1.0, 0.0, 1.0];
                    self.push_line(
                        point - Vec2::new(arm, arm),
                        point + Vec2::new(arm, arm),
                        colour,
                        thickness,
                    );
                    self.push_line(
                        point - Vec2::new(arm, -arm),
                        point + Vec2::new(arm, -arm),
                        colour,
                        thickness,
                    );
                }
            }
        }

        // Upload to GPU
        if !self.vertices.is_empty() {
            self.vertex_buffer =
                self.device
                    .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("Vertex Buffer"),
                        contents: bytemuck::cast_slice(&self.vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    });
        }

        if !self.indices.is_empty() {
            self.index_buffer = self
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Index Buffer"),
                    contents: bytemuck::cast_slice(&self.indices),
                    usage: wgpu::BufferUsages::INDEX,
                });
        }
    }

    /// Add a shape to the render list
    fn add_shape(
        &mut self,
        shape: &Shape,
        transform: Transform,
        body: &Body,
        _collider: &Collider,
    ) {
        if !self.debug_flags.show_colliders {
            return;
        }

        let color = if body.is_sleeping() && self.debug_flags.show_sleeping {
            [0.5, 0.5, 0.5] // Gray for sleeping bodies
        } else if body.is_static() {
            [0.5, 0.5, 1.0] // Light blue for static bodies
        } else if body.is_kinematic() {
            [0.5, 1.0, 0.5] // Light green for kinematic bodies
        } else {
            [1.0, 0.5, 0.5] // Light red for dynamic bodies
        };

        match shape {
            Shape::Circle(circle) => {
                self.add_circle(circle, transform, color);
            }
            Shape::Polygon(polygon) => {
                self.add_polygon(polygon, transform, color);
            }
        }
    }

    /// Add a circle to the render list
    fn add_circle(&mut self, circle: &Circle, transform: Transform, color: [f32; 3]) {
        let radius = circle.radius();
        let segments = 32;

        let start_idx = self.vertices.len() as u16;

        // Center vertex
        self.push_vertex(transform.position.x, transform.position.y, color);

        // Circle vertices
        for i in 0..=segments {
            let angle = 2.0 * std::f32::consts::PI * (i as f32) / (segments as f32);
            let x = radius * angle.cos();
            let y = radius * angle.sin();

            // Rotate by body rotation
            let rotated_x = x * transform.rotation.cos() - y * transform.rotation.sin();
            let rotated_y = x * transform.rotation.sin() + y * transform.rotation.cos();

            self.push_vertex(
                transform.position.x + rotated_x,
                transform.position.y + rotated_y,
                color,
            );
        }

        // Triangles (fan from center)
        for i in 1..=segments {
            self.indices.push(start_idx);
            self.indices.push(start_idx + i);
            self.indices.push(start_idx + ((i + 1) % (segments + 1)));
        }
    }

    /// Add a polygon to the render list
    fn add_polygon(&mut self, polygon: &Polygon, transform: Transform, color: [f32; 3]) {
        let vertices = polygon.vertices();
        let start_idx = self.vertices.len() as u16;

        // Add vertices
        for vertex in vertices {
            // Rotate vertex
            let rotated_x =
                vertex.x * transform.rotation.cos() - vertex.y * transform.rotation.sin();
            let rotated_y =
                vertex.x * transform.rotation.sin() + vertex.y * transform.rotation.cos();

            self.push_vertex(
                transform.position.x + rotated_x,
                transform.position.y + rotated_y,
                color,
            );
        }

        // Add indices for triangle fan
        for i in 1..(vertices.len() - 1) {
            self.indices.push(start_idx);
            self.indices.push(start_idx + i as u16);
            self.indices.push(start_idx + (i + 1) as u16);
        }
    }

    /// Add an AABB to the render list
    fn add_aabb(&mut self, aabb: Aabb, color: [f32; 3]) {
        let start_idx = self.vertices.len() as u16;

        // AABB vertices (4 corners)
        self.push_vertex(aabb.min.x, aabb.min.y, color);
        self.push_vertex(aabb.max.x, aabb.min.y, color);
        self.push_vertex(aabb.max.x, aabb.max.y, color);
        self.push_vertex(aabb.min.x, aabb.max.y, color);

        // Two triangles for the AABB
        self.indices.push(start_idx);
        self.indices.push(start_idx + 1);
        self.indices.push(start_idx + 2);

        self.indices.push(start_idx);
        self.indices.push(start_idx + 2);
        self.indices.push(start_idx + 3);
    }

    /// Render the scene
    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.1,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            // Set pipeline
            render_pass.set_pipeline(&self.render_pipeline);

            // Set vertex buffer
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));

            // Set index buffer
            render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);

            // Draw
            if !self.indices.is_empty() {
                render_pass.draw_indexed(0..self.indices.len() as u32, 0, 0..1);
            }
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}
