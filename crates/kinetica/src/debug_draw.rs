//! Renderer-agnostic debug drawing of a [`World`].
//!
//! Implement [`DebugDraw`] for your renderer, then call [`World::debug_draw`] every frame.

use crate::body::BodyType;
use crate::joint::JointKind;
use crate::math::{Rot, Transform, Vec2};
use crate::shape::{MAX_POLYGON_VERTICES, Shape};
use crate::world::World;

/// An RGBA color with components in `[0, 1]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color {
    /// Red component.
    pub r: f32,
    /// Green component.
    pub g: f32,
    /// Blue component.
    pub b: f32,
    /// Opacity.
    pub a: f32,
}

impl Color {
    /// Creates an opaque color.
    #[must_use]
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Returns the same color with another opacity.
    #[must_use]
    pub const fn with_alpha(self, a: f32) -> Self {
        Self { a, ..self }
    }
}

/// Drawing primitives a renderer must provide. Everything is in world coordinates (meters).
pub trait DebugDraw {
    /// Draws a convex polygon given in counter-clockwise order.
    fn draw_polygon(&mut self, vertices: &[Vec2], color: Color);
    /// Draws a disc; `rotation` tells which way the circle is facing, to make spinning visible.
    fn draw_circle(&mut self, center: Vec2, radius: f32, rotation: Rot, color: Color);
    /// Draws a line segment.
    fn draw_segment(&mut self, start: Vec2, end: Vec2, color: Color);
    /// Draws a small marker at a point.
    fn draw_point(&mut self, point: Vec2, color: Color);
}

/// Selects what [`World::debug_draw`] shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools, reason = "independent on/off switches")]
pub struct DebugDrawOptions {
    /// Collider shapes, colored by body type.
    pub shapes: bool,
    /// Collider bounding boxes.
    pub aabbs: bool,
    /// Body centers of mass.
    pub centers_of_mass: bool,
    /// Contact points and normals.
    pub contacts: bool,
    /// Joint anchors and links.
    pub joints: bool,
}

impl Default for DebugDrawOptions {
    fn default() -> Self {
        Self { shapes: true, aabbs: false, centers_of_mass: false, contacts: false, joints: true }
    }
}

const FIXED_COLOR: Color = Color::rgb(0.55, 0.6, 0.55);
const KINEMATIC_COLOR: Color = Color::rgb(0.45, 0.6, 0.95);
const DYNAMIC_COLOR: Color = Color::rgb(0.95, 0.65, 0.3);
const AABB_COLOR: Color = Color::rgb(0.9, 0.3, 0.9);
const CENTER_OF_MASS_COLOR: Color = Color::rgb(0.95, 0.95, 0.95);
const PENETRATING_POINT_COLOR: Color = Color::rgb(0.95, 0.25, 0.25);
const SPECULATIVE_POINT_COLOR: Color = Color::rgb(0.95, 0.85, 0.25);
const CONTACT_NORMAL_COLOR: Color = Color::rgb(0.35, 0.85, 0.45);
const JOINT_COLOR: Color = Color::rgb(0.4, 0.85, 0.85);
const MOUSE_JOINT_COLOR: Color = Color::rgb(0.95, 0.95, 0.4);
/// Length of the drawn contact normals, in meters.
const CONTACT_NORMAL_LENGTH: f32 = 0.3;

impl World {
    /// Draws the world with the given renderer.
    pub fn debug_draw(&self, drawer: &mut impl DebugDraw, options: &DebugDrawOptions) {
        for (_, collider) in self.colliders() {
            let Some(body) = self.body(collider.body()) else { continue };

            if options.shapes {
                let color = match body.body_type() {
                    BodyType::Fixed => FIXED_COLOR,
                    BodyType::Kinematic => KINEMATIC_COLOR,
                    BodyType::Dynamic => DYNAMIC_COLOR,
                };
                draw_shape(drawer, collider.shape(), body.transform(), color);
            }

            if options.aabbs {
                let aabb = collider.aabb();
                let corners =
                    [aabb.min, Vec2::new(aabb.max.x, aabb.min.y), aabb.max, Vec2::new(aabb.min.x, aabb.max.y)];
                for (i, &corner) in corners.iter().enumerate() {
                    drawer.draw_segment(corner, corners[(i + 1) % corners.len()], AABB_COLOR);
                }
            }
        }

        if options.centers_of_mass {
            for (_, body) in self.bodies() {
                drawer.draw_point(body.center_of_mass(), CENTER_OF_MASS_COLOR);
            }
        }

        if options.joints {
            self.draw_joints(drawer);
        }

        if options.contacts {
            for contact in self.contacts() {
                let normal = contact.manifold().normal();
                for point in contact.manifold().points() {
                    let position = point.position();
                    drawer.draw_segment(position, position + CONTACT_NORMAL_LENGTH * normal, CONTACT_NORMAL_COLOR);
                    let color =
                        if point.separation() < 0.0 { PENETRATING_POINT_COLOR } else { SPECULATIVE_POINT_COLOR };
                    drawer.draw_point(position, color);
                }
            }
        }
    }
}

impl World {
    fn draw_joints(&self, drawer: &mut impl DebugDraw) {
        for (_, joint) in self.joints() {
            let (Some(a), Some(b)) = (self.body(joint.body_a()), self.body(joint.body_b())) else { continue };
            let (local_anchor_a, local_anchor_b) = joint.local_anchors();
            let anchor_b = b.world_point(local_anchor_b);
            match joint.kind() {
                JointKind::Mouse(mouse) => {
                    drawer.draw_segment(mouse.target(), anchor_b, MOUSE_JOINT_COLOR);
                    drawer.draw_point(mouse.target(), MOUSE_JOINT_COLOR);
                }
                JointKind::Distance(_) => {
                    drawer.draw_segment(a.world_point(local_anchor_a), anchor_b, JOINT_COLOR);
                }
                _ => {
                    // Each body's center linked to its anchor, like a skeleton.
                    let anchor_a = a.world_point(local_anchor_a);
                    drawer.draw_segment(a.center_of_mass(), anchor_a, JOINT_COLOR);
                    drawer.draw_segment(anchor_a, anchor_b, JOINT_COLOR);
                    drawer.draw_segment(b.center_of_mass(), anchor_b, JOINT_COLOR);
                    drawer.draw_point(anchor_b, JOINT_COLOR);
                }
            }
        }
    }
}

fn draw_shape(drawer: &mut impl DebugDraw, shape: &Shape, transform: &Transform, color: Color) {
    match shape {
        Shape::Circle(circle) => {
            drawer.draw_circle(transform.transform_point(circle.center()), circle.radius(), transform.rotation, color);
        }
        Shape::Polygon(polygon) => {
            let mut world_vertices = [Vec2::ZERO; MAX_POLYGON_VERTICES];
            let count = polygon.vertices().len();
            for (world, &local) in world_vertices.iter_mut().zip(polygon.vertices()) {
                *world = transform.transform_point(local);
            }
            drawer.draw_polygon(&world_vertices[..count], color);
        }
    }
}
