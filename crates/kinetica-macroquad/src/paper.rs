//! A cut-paper look for games: shapes are colored paper cut-outs with an ink edge and a soft
//! shadow, on a sheet of paper, with handwritten text.

use kinetica::math::{Transform, Vec2};
use kinetica::shape::Shape;
use kinetica::{BodyHandle, ColliderHandle, World};
use macroquad::color::Color;
use macroquad::shapes;
use macroquad::text::{Font, TextParams, draw_text_ex, load_ttf_font_from_bytes, measure_text};
use macroquad::window::{clear_background, screen_height, screen_width};

use crate::WorldCamera;

/// The paper sheet behind everything.
pub const PAPER: Color = Color::new(0.96, 0.93, 0.86, 1.0);
/// Dark brown ink, for text and outlines.
pub const INK: Color = Color::new(0.24, 0.2, 0.18, 1.0);

/// Patrick Hand, by Patrick Wagesreiter, under the SIL Open Font License (see `assets/OFL.txt`).
const FONT: &[u8] = include_bytes!("../assets/PatrickHand-Regular.ttf");
/// Shadows fall down and to the right, this many pixels away.
const SHADOW_OFFSET_PIXELS: Vec2 = Vec2::new(4.0, -5.0);
const SHADOW: Color = Color::new(0.25, 0.18, 0.1, 0.22);
const EDGE_WIDTH_PIXELS: f32 = 2.5;
/// Edges are the fill color, darkened by this factor.
const EDGE_DARKNESS: f32 = 0.62;
const CIRCLE_SIDES: usize = 48;
/// Specks of paper grain drawn over the sheet.
const GRAIN_SPECKS: u32 = 700;

/// The paper look: the handwritten font, and drawing on the sheet.
///
/// Create it once, after the window is open; draw with a [`PaperRenderer`] every frame.
#[derive(Debug, Clone)]
pub struct Paper {
    font: Font,
}

impl Paper {
    /// Loads the font.
    ///
    /// # Panics
    ///
    /// Panics if the embedded font cannot be read, which would be a build mistake.
    #[must_use]
    pub fn new() -> Self {
        Self { font: load_ttf_font_from_bytes(FONT).expect("the embedded font is valid") }
    }

    /// Clears the window to a sheet of paper, with a faint grain.
    pub fn draw_sheet(&self, color: Color) {
        clear_background(color);
        let speck = darken(color, 0.85);
        // A fixed pseudo-random pattern, so the grain does not flicker.
        let mut seed: u32 = 0x9e37_79b9;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed % 10_000) as f32 / 10_000.0
        };
        for _ in 0..GRAIN_SPECKS {
            let (x, y, size) = (next() * screen_width(), next() * screen_height(), 1.0 + 2.0 * next());
            shapes::draw_rectangle(x, y, size, size, Color { a: 0.25 + 0.35 * next(), ..speck });
        }
    }

    /// Writes `text` `size` pixels tall, with its baseline at `position`, in pixels from the top
    /// left of the window.
    pub fn draw_text(&self, text: &str, position: Vec2, size: u16, color: Color) {
        let params = TextParams { font: Some(&self.font), font_size: size, color, ..TextParams::default() };
        draw_text_ex(text, position.x, position.y, params);
    }

    /// Writes `text` centered horizontally on the window, with its baseline `y` pixels from the
    /// top.
    pub fn draw_centered_text(&self, text: &str, y: f32, size: u16, color: Color) {
        let width = self.text_width(text, size);
        self.draw_text(text, Vec2::new((screen_width() - width) / 2.0, y), size, color);
    }

    /// Width of `text` written at `size`, in pixels.
    #[must_use]
    pub fn text_width(&self, text: &str, size: u16) -> f32 {
        measure_text(text, Some(&self.font), size, 1.0).width
    }
}

impl Default for Paper {
    fn default() -> Self {
        Self::new()
    }
}

/// Draws paper cut-outs in world coordinates; set the camera first with [`WorldCamera::apply`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PaperRenderer {
    /// Shadow offset and edge width, in meters.
    shadow_offset: Vec2,
    edge_width: f32,
}

impl PaperRenderer {
    /// A renderer for the zoom level of `camera`.
    #[must_use]
    pub fn new(camera: &WorldCamera) -> Self {
        let meters_per_pixel = 1.0 / camera.pixels_per_meter();
        Self {
            shadow_offset: SHADOW_OFFSET_PIXELS * meters_per_pixel,
            edge_width: EDGE_WIDTH_PIXELS * meters_per_pixel,
        }
    }

    /// Draws every collider of a body cut out of `color` paper, except its sensors, which are
    /// invisible.
    ///
    /// # Panics
    ///
    /// Panics if the body was removed.
    pub fn draw_body(&self, world: &World, body: BodyHandle, color: Color) {
        for &collider in world[body].colliders().iter().filter(|&&collider| !world[collider].is_sensor()) {
            self.draw_collider(world, collider, color);
        }
    }

    /// Draws a collider cut out of `color` paper.
    ///
    /// # Panics
    ///
    /// Panics if the collider was removed.
    pub fn draw_collider(&self, world: &World, collider: ColliderHandle, color: Color) {
        let collider = &world[collider];
        self.draw_shape(collider.shape(), world[collider.body()].transform(), color);
    }

    /// Draws `shape` placed at `transform`. Segments are drawn as ink lines.
    pub fn draw_shape(&self, shape: &Shape, transform: &Transform, color: Color) {
        match shape {
            Shape::Circle(circle) => {
                self.draw_circle(transform.transform_point(circle.center()), circle.radius(), color);
            }
            Shape::Polygon(polygon) => {
                let vertices: Vec<Vec2> =
                    polygon.vertices().iter().map(|&vertex| transform.transform_point(vertex)).collect();
                self.draw_polygon(&vertices, color);
            }
            Shape::Capsule(capsule) => {
                let start = transform.transform_point(capsule.start());
                let end = transform.transform_point(capsule.end());
                self.draw_capsule(start, end, capsule.radius(), color);
            }
            Shape::Segment(segment) => {
                let (start, end) =
                    (transform.transform_point(segment.start()), transform.transform_point(segment.end()));
                self.draw_line(start, end, color);
            }
            Shape::ChainSegment(chain) => {
                let segment = chain.segment();
                let (start, end) =
                    (transform.transform_point(segment.start()), transform.transform_point(segment.end()));
                self.draw_line(start, end, color);
            }
        }
    }

    /// Draws a convex polygon, its vertices in counter-clockwise order.
    pub fn draw_polygon(&self, vertices: &[Vec2], color: Color) {
        let shadow: Vec<Vec2> = vertices.iter().map(|&vertex| vertex + self.shadow_offset).collect();
        fill_convex(&shadow, SHADOW);
        fill_convex(vertices, color);
        self.draw_outline(vertices, darken(color, EDGE_DARKNESS));
    }

    /// Draws a disc.
    pub fn draw_circle(&self, center: Vec2, radius: f32, color: Color) {
        self.draw_polygon(&circle_points(center, radius), color);
    }

    /// Draws a capsule: every point within `radius` of the segment from `start` to `end`.
    pub fn draw_capsule(&self, start: Vec2, end: Vec2, radius: f32, color: Color) {
        let axis = (end - start).normalize_or_zero();
        let side = axis.perp();
        let half = CIRCLE_SIDES / 2;
        // Half circle around the end, then around the start, counter-clockwise.
        let outline: Vec<Vec2> = [(end, -side), (start, side)]
            .into_iter()
            .flat_map(|(center, from)| {
                (0..=half).map(move |i| {
                    let angle = std::f32::consts::PI * i as f32 / half as f32;
                    center + radius * Vec2::from_angle(angle).rotate(from)
                })
            })
            .collect();
        self.draw_polygon(&outline, color);
    }

    /// Draws the ground under a line of points going left to right: filled with `color` down to
    /// `bottom`, with a darker edge along the top.
    pub fn draw_ground(&self, surface: &[Vec2], bottom: f32, color: Color) {
        for pair in surface.windows(2) {
            let (left, right) = (pair[0], pair[1]);
            fill_convex(&[left, Vec2::new(left.x, bottom), Vec2::new(right.x, bottom), right], color);
        }
        let edge = darken(color, EDGE_DARKNESS);
        for pair in surface.windows(2) {
            self.draw_line(pair[0], pair[1], edge);
        }
    }

    /// Draws a band of grass `depth` meters thick along the top of the ground, skipping walls.
    pub fn draw_grass(&self, surface: &[Vec2], depth: f32, color: Color) {
        let edge = darken(color, EDGE_DARKNESS);
        for pair in surface.windows(2).filter(|pair| pair[0].x < pair[1].x) {
            let (left, right) = (pair[0], pair[1]);
            let below = Vec2::new(0.0, -depth);
            fill_convex(&[left + below, right + below, right, left], color);
            self.draw_line(left, right, edge);
        }
    }

    /// Draws a row of hills far behind the scene, across the whole view of `camera`.
    ///
    /// The hills scroll `parallax` times as fast as the scene (0 stays on screen, 1 moves with
    /// the ground), their middle `base` meters from the view center, `height` meters high.
    pub fn draw_hills(&self, camera: &WorldCamera, parallax: f32, base: f32, height: f32, color: Color) {
        const POINTS: usize = 80;
        let center = camera.center();
        let half_width = camera.view_height() * screen_width() / screen_height() / 2.0 + 1.0;
        let left = center.x - half_width;
        let points: Vec<Vec2> = (0..=POINTS)
            .map(|i| {
                let x = left + 2.0 * half_width * i as f32 / POINTS as f32;
                // Distant hills move less: measure them from a point following the camera.
                let far = x - parallax * center.x;
                let wave = (0.12 * far).sin() + 0.4 * (0.29 * far + 1.0).sin();
                Vec2::new(x, center.y + base + 0.5 * height * wave)
            })
            .collect();
        self.draw_ground(&points, center.y - camera.view_height(), color);
    }

    /// Draws an ink line.
    pub fn draw_line(&self, start: Vec2, end: Vec2, color: Color) {
        self.draw_thick_line(start, end, self.edge_width, color);
    }

    /// Draws a line `width` meters wide, with round ends.
    pub fn draw_thick_line(&self, start: Vec2, end: Vec2, width: f32, color: Color) {
        shapes::draw_line(start.x, start.y, end.x, end.y, width, color);
        for point in [start, end] {
            shapes::draw_circle(point.x, point.y, width / 2.0, color);
        }
    }

    /// Width of ink lines, in meters.
    #[must_use]
    pub fn edge_width(&self) -> f32 {
        self.edge_width
    }

    fn draw_outline(&self, vertices: &[Vec2], color: Color) {
        for (i, &start) in vertices.iter().enumerate() {
            self.draw_line(start, vertices[(i + 1) % vertices.len()], color);
        }
    }
}

/// `color` with its red, green and blue scaled by `factor`: below 1 darkens, above lightens.
#[must_use]
pub fn darken(color: Color, factor: f32) -> Color {
    Color::new(color.r * factor, color.g * factor, color.b * factor, color.a)
}

/// Points around a circle, counter-clockwise.
fn circle_points(center: Vec2, radius: f32) -> Vec<Vec2> {
    (0..CIRCLE_SIDES)
        .map(|i| center + radius * Vec2::from_angle(std::f32::consts::TAU * i as f32 / CIRCLE_SIDES as f32))
        .collect()
}

fn fill_convex(vertices: &[Vec2], color: Color) {
    for pair in vertices[1..].windows(2) {
        shapes::draw_triangle(vertices[0], pair[0], pair[1], color);
    }
}
