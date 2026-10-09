//! Draws Kinetica worlds with macroquad shapes.

use kinetica::math::{Rot, Vec2};
use kinetica::shape::Shape;
use kinetica::{BodyHandle, ColliderHandle, Color, DebugDraw, DebugDrawOptions, World};
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
/// screen at every zoom level.
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

    /// Draws the whole world.
    pub fn draw_world(&mut self, world: &World, options: &DebugDrawOptions) {
        world.debug_draw(self, options);
    }

    /// Draws a collider in a custom color.
    pub fn draw_collider(&mut self, world: &World, collider: ColliderHandle, color: Color) {
        let Some(collider) = world.collider(collider) else { return };
        let Some(body) = world.body(collider.body()) else { return };
        let transform = body.transform();
        match collider.shape() {
            Shape::Circle(circle) => {
                let center = transform.transform_point(circle.center());
                self.draw_circle(center, circle.radius(), transform.rotation, color);
            }
            Shape::Capsule(capsule) => {
                let (start, end) =
                    (transform.transform_point(capsule.start()), transform.transform_point(capsule.end()));
                self.draw_capsule(start, end, capsule.radius(), color);
            }
            Shape::Polygon(polygon) => {
                let vertices: Vec<Vec2> = polygon.vertices().iter().map(|&v| transform.transform_point(v)).collect();
                self.draw_polygon(&vertices, color);
            }
            Shape::Segment(segment) => {
                let (start, end) =
                    (transform.transform_point(segment.start()), transform.transform_point(segment.end()));
                self.draw_segment(start, end, color);
            }
            Shape::ChainSegment(chain) => {
                let segment = chain.segment();
                let (start, end) =
                    (transform.transform_point(segment.start()), transform.transform_point(segment.end()));
                self.draw_segment(start, end, color);
            }
        }
    }

    /// Draws every collider of a body in a custom color.
    pub fn draw_body(&mut self, world: &World, body: BodyHandle, color: Color) {
        for &collider in world.body(body).map_or(&[][..], kinetica::Body::colliders) {
            self.draw_collider(world, collider, color);
        }
    }
}

/// Converts a Kinetica color to a macroquad color.
#[must_use]
pub fn to_macroquad_color(color: Color) -> macroquad::color::Color {
    macroquad::color::Color::new(color.r, color.g, color.b, color.a)
}

fn fill_color(color: Color) -> macroquad::color::Color {
    to_macroquad_color(color.with_alpha(color.a * FILL_OPACITY))
}

impl DebugDraw for DebugRenderer {
    fn draw_polygon(&mut self, vertices: &[Vec2], color: Color) {
        let fill = fill_color(color);
        for pair in vertices[1..].windows(2) {
            shapes::draw_triangle(vertices[0], pair[0], pair[1], fill);
        }
        for (i, &start) in vertices.iter().enumerate() {
            self.draw_segment(start, vertices[(i + 1) % vertices.len()], color);
        }
    }

    fn draw_circle(&mut self, center: Vec2, radius: f32, rotation: Rot, color: Color) {
        shapes::draw_poly(center.x, center.y, CIRCLE_SIDES, radius, 0.0, fill_color(color));
        let outline = to_macroquad_color(color);
        shapes::draw_poly_lines(center.x, center.y, CIRCLE_SIDES, radius, 0.0, self.line_width, outline);
        self.draw_segment(center, center + rotation.rotate(Vec2::new(radius, 0.0)), color);
    }

    fn draw_segment(&mut self, start: Vec2, end: Vec2, color: Color) {
        shapes::draw_line(start.x, start.y, end.x, end.y, self.line_width, to_macroquad_color(color));
    }

    fn draw_point(&mut self, point: Vec2, color: Color) {
        shapes::draw_circle(point.x, point.y, 2.5 * self.line_width, to_macroquad_color(color));
    }
}
