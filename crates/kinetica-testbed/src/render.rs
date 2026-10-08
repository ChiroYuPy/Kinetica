//! Draws Kinetica debug primitives with macroquad.

use kinetica::math::{Rot, Vec2};
use kinetica::{Color, DebugDraw};
use macroquad::shapes;

/// Opacity of shape interiors, relative to their outline.
const FILL_OPACITY: f32 = 0.3;
const CIRCLE_SIDES: u8 = 40;
const LINE_WIDTH_PIXELS: f32 = 1.5;

pub struct MacroquadDraw {
    /// Line width in meters, so lines keep the same on-screen width at every zoom level.
    line_width: f32,
}

impl MacroquadDraw {
    pub fn new(pixels_per_meter: f32) -> Self {
        Self { line_width: LINE_WIDTH_PIXELS / pixels_per_meter }
    }
}

fn to_macroquad(color: Color) -> macroquad::color::Color {
    macroquad::color::Color::new(color.r, color.g, color.b, color.a)
}

fn fill_color(color: Color) -> macroquad::color::Color {
    to_macroquad(color.with_alpha(color.a * FILL_OPACITY))
}

impl DebugDraw for MacroquadDraw {
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
        shapes::draw_poly_lines(center.x, center.y, CIRCLE_SIDES, radius, 0.0, self.line_width, to_macroquad(color));
        self.draw_segment(center, center + rotation.rotate(Vec2::new(radius, 0.0)), color);
    }

    fn draw_segment(&mut self, start: Vec2, end: Vec2, color: Color) {
        shapes::draw_line(start.x, start.y, end.x, end.y, self.line_width, to_macroquad(color));
    }

    fn draw_point(&mut self, point: Vec2, color: Color) {
        shapes::draw_circle(point.x, point.y, 2.5 * self.line_width, to_macroquad(color));
    }
}
