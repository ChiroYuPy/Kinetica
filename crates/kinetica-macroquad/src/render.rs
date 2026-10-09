//! Draws Kinetica worlds with macroquad shapes.

use kinetica::math::{Rot, Vec2};
use kinetica::{BodyHandle, ColliderHandle, DebugDraw, DebugDrawOptions, World};
use macroquad::color::Color;
use macroquad::shapes;

use crate::WorldCamera;

/// Opacity of shape interiors, relative to their outline.
const FILL_OPACITY: f32 = 0.3;
const CIRCLE_SIDES: u8 = 40;
const LINE_WIDTH_PIXELS: f32 = 1.5;

/// Draws with macroquad in world coordinates; set the camera first with
/// [`WorldCamera::apply`].
///
/// Shapes are filled with a see-through color and outlined, lines keep the same width on
/// screen at every zoom level. Its methods take macroquad colors; it also implements
/// [`DebugDraw`], which [`draw_world`](Self::draw_world) uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DebugRenderer {
    /// Line width in meters.
    line_width: f32,
}

impl DebugRenderer {
    /// A renderer for the zoom level of `camera`.
    #[must_use]
    pub fn new(camera: &WorldCamera) -> Self {
        Self { line_width: LINE_WIDTH_PIXELS / camera.pixels_per_meter() }
    }

    /// Draws the whole world, colored by body type.
    pub fn draw_world(&mut self, world: &World, options: &DebugDrawOptions) {
        world.debug_draw(self, options);
    }

    /// Draws every collider of a body.
    ///
    /// # Panics
    ///
    /// Panics if the body was removed.
    pub fn draw_body(&mut self, world: &World, body: BodyHandle, color: Color) {
        for &collider in world[body].colliders() {
            self.draw_collider(world, collider, color);
        }
    }

    /// Draws a collider.
    ///
    /// # Panics
    ///
    /// Panics if the collider was removed.
    pub fn draw_collider(&mut self, world: &World, collider: ColliderHandle, color: Color) {
        let collider = &world[collider];
        let transform = world[collider.body()].transform();
        DebugDraw::draw_shape(self, collider.shape(), transform, physics_color(color));
    }

    /// Draws a convex polygon given in counter-clockwise order.
    pub fn draw_polygon(&mut self, vertices: &[Vec2], color: Color) {
        let fill = fill_color(color);
        for pair in vertices[1..].windows(2) {
            shapes::draw_triangle(vertices[0], pair[0], pair[1], fill);
        }
        for (i, &start) in vertices.iter().enumerate() {
            self.draw_segment(start, vertices[(i + 1) % vertices.len()], color);
        }
    }

    /// Draws a disc, with a radius showing which way it is facing.
    pub fn draw_circle(&mut self, center: Vec2, radius: f32, rotation: Rot, color: Color) {
        shapes::draw_poly(center.x, center.y, CIRCLE_SIDES, radius, 0.0, fill_color(color));
        shapes::draw_poly_lines(center.x, center.y, CIRCLE_SIDES, radius, 0.0, self.line_width, color);
        self.draw_segment(center, center + rotation.rotate(Vec2::new(radius, 0.0)), color);
    }

    /// Draws a capsule: every point within `radius` of the segment from `start` to `end`.
    pub fn draw_capsule(&mut self, start: Vec2, end: Vec2, radius: f32, color: Color) {
        DebugDraw::draw_capsule(self, start, end, radius, physics_color(color));
    }

    /// Draws a line segment.
    pub fn draw_segment(&mut self, start: Vec2, end: Vec2, color: Color) {
        shapes::draw_line(start.x, start.y, end.x, end.y, self.line_width, color);
    }

    /// Draws a small marker at a point.
    pub fn draw_point(&mut self, point: Vec2, color: Color) {
        shapes::draw_circle(point.x, point.y, 2.5 * self.line_width, color);
    }
}

fn macroquad_color(color: kinetica::Color) -> Color {
    Color::new(color.r, color.g, color.b, color.a)
}

fn physics_color(color: Color) -> kinetica::Color {
    kinetica::Color::rgb(color.r, color.g, color.b).with_alpha(color.a)
}

fn fill_color(color: Color) -> Color {
    Color { a: color.a * FILL_OPACITY, ..color }
}

impl DebugDraw for DebugRenderer {
    fn draw_polygon(&mut self, vertices: &[Vec2], color: kinetica::Color) {
        Self::draw_polygon(self, vertices, macroquad_color(color));
    }

    fn draw_circle(&mut self, center: Vec2, radius: f32, rotation: Rot, color: kinetica::Color) {
        Self::draw_circle(self, center, radius, rotation, macroquad_color(color));
    }

    fn draw_segment(&mut self, start: Vec2, end: Vec2, color: kinetica::Color) {
        Self::draw_segment(self, start, end, macroquad_color(color));
    }

    fn draw_point(&mut self, point: Vec2, color: kinetica::Color) {
        Self::draw_point(self, point, macroquad_color(color));
    }
}
