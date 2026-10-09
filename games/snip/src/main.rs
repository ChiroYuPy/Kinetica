//! Snip: cut the ropes with the mouse to drop the sweet in the basket, picking up the stars.

mod game;
mod level;

use std::f32::consts::{PI, TAU};

use kinetica::prelude::*;
use kinetica_macroquad::paper::{INK, PAPER, darken};
use kinetica_macroquad::{FixedTimestep, Paper, PaperRenderer, WorldCamera};
use macroquad::prelude::*;

use crate::game::{END_DELAY, Game, STAR_RADIUS, SWEET_RADIUS, State};

const CLOUD: Color = Color::new(0.9, 0.92, 0.95, 1.0);
const ROPE: Color = Color::new(0.55, 0.42, 0.3, 1.0);
const ELASTIC: Color = Color::new(0.85, 0.45, 0.55, 1.0);
const PIN: Color = Color::new(0.45, 0.45, 0.5, 1.0);
const SWEET: Color = Color::new(0.95, 0.5, 0.6, 1.0);
const WRAPPER: Color = Color::new(0.98, 0.85, 0.5, 1.0);
const BASKET: Color = Color::new(0.75, 0.55, 0.35, 1.0);
const STAR: Color = Color::new(0.98, 0.8, 0.3, 1.0);
const PAD: Color = Color::new(0.45, 0.75, 0.75, 1.0);
const WINDMILL: Color = Color::new(0.6, 0.65, 0.9, 1.0);
const STROKE: Color = Color::new(1.0, 1.0, 1.0, 0.9);
/// How long the trail of a cut stays on screen, in seconds.
const STROKE_LIFETIME: f32 = 0.2;

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Snip")
}

struct App {
    game: Game,
    paper: Paper,
    camera: WorldCamera,
    timestep: FixedTimestep,
    /// Recent mouse positions while cutting, with their age in seconds.
    stroke: Vec<(Vec2, f32)>,
}

impl App {
    fn new() -> Self {
        Self {
            game: Game::new(0),
            paper: Paper::new(),
            camera: WorldCamera::new(vec2(12.0, 7.3), 15.4),
            timestep: FixedTimestep::default(),
            stroke: Vec::new(),
        }
    }

    fn update(&mut self) {
        let level = self.game.level_index;
        let next = (level + 1) % Game::level_count();
        match self.game.state {
            State::Won(since)
                if since > END_DELAY
                    && (is_key_pressed(KeyCode::Enter) || is_mouse_button_pressed(MouseButton::Left)) =>
            {
                self.game = Game::new(next);
            }
            State::Lost(since) if since > END_DELAY => self.game = Game::new(level),
            _ if is_key_pressed(KeyCode::R) => self.game = Game::new(level),
            _ => {}
        }
        self.cut();
        for _ in 0..self.timestep.advance(get_frame_time()) {
            self.game.step();
        }
    }

    /// While the mouse button is held, every rope the mouse crosses is cut.
    fn cut(&mut self) {
        let dt = get_frame_time();
        for point in &mut self.stroke {
            point.1 += dt;
        }
        self.stroke.retain(|point| point.1 < STROKE_LIFETIME);
        if !is_mouse_button_down(MouseButton::Left) {
            return;
        }
        let mouse = self.camera.mouse_position();
        if let Some(&(last, _)) = self.stroke.last() {
            self.game.cut(last, mouse);
        }
        self.stroke.push((mouse, 0.0));
    }

    fn draw(&self) {
        self.paper.draw_sheet(PAPER);
        self.camera.apply();
        let pen = PaperRenderer::new(&self.camera);
        for (center, size) in [(vec2(4.0, 11.0), 1.0), (vec2(17.0, 12.0), 1.3), (vec2(21.0, 8.0), 0.8)] {
            draw_cloud(&pen, center, size);
        }
        self.draw_game(&pen);
        for pair in self.stroke.windows(2) {
            let fade = 1.0 - pair[1].1 / STROKE_LIFETIME;
            pen.draw_thick_line(pair[0].0, pair[1].0, 0.12 * fade, Color { a: STROKE.a * fade, ..STROKE });
        }
        set_default_camera();
        self.draw_hud();
    }

    fn draw_game(&self, pen: &PaperRenderer) {
        let game = &self.game;
        let world = &game.world;
        for &pad in &game.pads {
            pen.draw_body(world, pad, PAD);
        }
        for &windmill in &game.windmills {
            pen.draw_body(world, windmill, WINDMILL);
            pen.draw_circle(world[windmill].position(), 0.25, darken(WINDMILL, 0.7));
        }
        for &star in &game.stars {
            // Stars turn slowly.
            draw_star(pen, world[star].position(), STAR_RADIUS, 0.6 * game.time, STAR);
        }
        pen.draw_body(world, game.basket, BASKET);

        for strand in &game.strands {
            for (a, b) in strand.intact_links(world) {
                if strand.elastic {
                    draw_zigzag(pen, a, b, ELASTIC);
                } else {
                    pen.draw_thick_line(a, b, 0.09, ROPE);
                }
            }
            pen.draw_circle(world[strand.bodies[0]].position(), 0.15, PIN);
        }
        draw_sweet(pen, &world[game.sweet]);
    }

    fn draw_hud(&self) {
        let game = &self.game;
        let paper = &self.paper;
        paper.draw_text(&format!("Level {} - {}", game.level_index + 1, game.level.name), vec2(24.0, 48.0), 40, INK);
        for i in 0..3 {
            let center = vec2(40.0 + 40.0 * i as f32, 82.0);
            let color = if i < game.stars_collected { STAR } else { Color { a: 0.25, ..INK } };
            draw_star_pixels(center, 15.0, color);
        }
        let message = match game.state {
            State::Won(_) if game.level_index + 1 == Game::level_count() => {
                Some(("All the sweets are in!", "Click: play again"))
            }
            State::Won(_) => Some(("Yum!", "Click: next level")),
            State::Lost(_) => Some(("Missed...", "")),
            State::Playing if game.level_index == 0 && game.time < 4.0 => {
                Some(("Drop the sweet in the basket", "Hold the mouse button and swipe across a rope to cut it"))
            }
            State::Playing => None,
        };
        if let Some((title, hint)) = message {
            // In the band above the pins.
            paper.draw_centered_text(title, 60.0, 52, INK);
            paper.draw_centered_text(hint, 96.0, 28, INK);
        }
        paper.draw_text("R: restart the level", vec2(24.0, screen_height() - 24.0), 26, Color { a: 0.6, ..INK });
    }
}

/// A round sweet in a twisted wrapper, turning with its body.
fn draw_sweet(pen: &PaperRenderer, body: &kinetica::Body) {
    let at = |x: f32, y: f32| body.world_point(SWEET_RADIUS * vec2(x, y));
    for side in [-1.0, 1.0] {
        pen.draw_polygon(&ordered([at(side * 0.8, 0.0), at(side * 1.6, 0.55), at(side * 1.6, -0.55)]), WRAPPER);
    }
    pen.draw_circle(body.position(), SWEET_RADIUS, SWEET);
    // A white swirl.
    for i in 0..3 {
        let angle = body.angle() + TAU * i as f32 / 3.0;
        let start = body.position() + 0.2 * SWEET_RADIUS * Vec2::from_angle(angle);
        let end = body.position() + 0.75 * SWEET_RADIUS * Vec2::from_angle(angle + 0.9);
        pen.draw_line(start, end, Color::new(1.0, 1.0, 1.0, 0.8));
    }
}

/// A five-pointed star, drawn as five convex points around its center.
fn draw_star(pen: &PaperRenderer, center: Vec2, radius: f32, angle: f32, color: Color) {
    let point =
        |i: usize, scale: f32| center + scale * radius * Vec2::from_angle(angle + PI / 2.0 + PI * i as f32 / 5.0);
    for tip in (0..10).step_by(2) {
        pen.draw_polygon(&[center, point(tip + 9, 0.45), point(tip, 1.0), point(tip + 1, 0.45)], color);
    }
}

/// The same star, flat, in screen pixels.
fn draw_star_pixels(center: Vec2, radius: f32, color: Color) {
    // Screen y points down: flip the star so it stands upright.
    let point = |i: usize, scale: f32| {
        let direction = Vec2::from_angle(PI / 2.0 + PI * i as f32 / 5.0);
        center + scale * radius * vec2(direction.x, -direction.y)
    };
    for tip in (0..10).step_by(2) {
        draw_triangle(center, point(tip + 9, 0.45), point(tip, 1.0), color);
        draw_triangle(center, point(tip, 1.0), point(tip + 1, 0.45), color);
    }
}

/// An elastic band, drawn as a zigzag from `a` to `b`.
fn draw_zigzag(pen: &PaperRenderer, a: Vec2, b: Vec2, color: Color) {
    const TEETH: usize = 14;
    let side = (b - a).normalize_or_zero().perp() * 0.15;
    let points: Vec<Vec2> = (0..=TEETH)
        .map(|i| {
            let offset = if i == 0 || i == TEETH {
                0.0
            } else if i % 2 == 0 {
                1.0
            } else {
                -1.0
            };
            a.lerp(b, i as f32 / TEETH as f32) + offset * side
        })
        .collect();
    for pair in points.windows(2) {
        pen.draw_thick_line(pair[0], pair[1], 0.07, color);
    }
}

/// A cloud of three overlapping discs.
fn draw_cloud(pen: &PaperRenderer, center: Vec2, size: f32) {
    for (offset, radius) in [(vec2(-0.8, 0.0), 0.6), (vec2(0.0, 0.3), 0.85), (vec2(0.85, 0.0), 0.55)] {
        pen.draw_circle(center + size * offset, size * radius, CLOUD);
    }
}

/// The three corners of a triangle in counter-clockwise order.
fn ordered(mut corners: [Vec2; 3]) -> [Vec2; 3] {
    if (corners[1] - corners[0]).perp_dot(corners[2] - corners[0]) < 0.0 {
        corners.swap(1, 2);
    }
    corners
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut app = App::new();
    loop {
        app.update();
        app.draw();
        next_frame().await;
    }
}
