//! Fruits: drop fruits in the jar; two alike merge into a bigger one. Don't let it overflow.

mod game;

use std::f32::consts::PI;

use kinetica::prelude::*;
use kinetica_macroquad::paper::{INK, PAPER, darken};
use kinetica_macroquad::{FixedTimestep, Paper, PaperRenderer, WorldCamera};
use macroquad::prelude::*;

use crate::game::{DROP_HEIGHT, Game, JAR_SIZE, RADII, TOP_LINE};

/// One color per kind of fruit, from the smallest.
const FRUIT_COLORS: [Color; 9] = [
    Color::new(0.85, 0.25, 0.35, 1.0),
    Color::new(0.98, 0.55, 0.6, 1.0),
    Color::new(0.65, 0.45, 0.85, 1.0),
    Color::new(0.98, 0.7, 0.25, 1.0),
    Color::new(0.95, 0.5, 0.2, 1.0),
    Color::new(0.9, 0.3, 0.25, 1.0),
    Color::new(0.98, 0.88, 0.4, 1.0),
    Color::new(0.98, 0.72, 0.75, 1.0),
    Color::new(0.5, 0.78, 0.35, 1.0),
];
const JAR: Color = Color::new(0.62, 0.78, 0.85, 1.0);
const TABLE: Color = Color::new(0.75, 0.58, 0.42, 1.0);
const LEAF: Color = Color::new(0.4, 0.68, 0.35, 1.0);
const DANGER: Color = Color::new(0.85, 0.25, 0.25, 1.0);
/// Where the next fruit is shown, beside the jar.
const PREVIEW: Vec2 = vec2(JAR_SIZE.x / 2.0 + 2.6, 7.6);

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Fruits")
}

struct App {
    game: Game,
    best: u32,
    paper: Paper,
    camera: WorldCamera,
    timestep: FixedTimestep,
}

impl App {
    fn new() -> Self {
        Self {
            game: Game::new(seed()),
            best: 0,
            paper: Paper::new(),
            camera: WorldCamera::new(vec2(0.0, 5.2), 12.5),
            timestep: FixedTimestep::default(),
        }
    }

    fn update(&mut self) {
        let clicked = is_mouse_button_pressed(MouseButton::Left) || is_key_pressed(KeyCode::Space);
        if is_key_pressed(KeyCode::R) || (self.game.game_over && clicked) {
            self.game = Game::new(seed());
        } else if clicked {
            self.game.drop(self.camera.mouse_position().x);
        }
        for _ in 0..self.timestep.advance(get_frame_time()) {
            self.game.step();
        }
        self.best = self.best.max(self.game.score);
    }

    fn draw(&self) {
        self.paper.draw_sheet(PAPER);
        self.camera.apply();
        let pen = PaperRenderer::new(&self.camera);
        let game = &self.game;
        pen.draw_polygon(&[vec2(-30.0, -10.0), vec2(30.0, -10.0), vec2(30.0, -0.4), vec2(-30.0, -0.4)], TABLE);

        // The held fruit, above where it will fall, then the jar's content.
        if game.can_drop() {
            let x = game.drop_x(self.camera.mouse_position().x);
            draw_guide(&pen, vec2(x, DROP_HEIGHT), game.next);
            draw_fruit(&pen, vec2(x, DROP_HEIGHT), 0.0, game.next);
        }
        for fruit in &game.fruits {
            let body = &game.world[fruit.body];
            draw_fruit(&pen, body.position(), body.angle(), fruit.kind);
        }
        pen.draw_body(&game.world, game.jar, JAR);
        draw_top_line(&pen, game.overflow);
        draw_fruit(&pen, PREVIEW, 0.0, game.after);
        set_default_camera();
        self.draw_hud();
    }

    fn draw_hud(&self) {
        let (paper, game) = (&self.paper, &self.game);
        paper.draw_text(&format!("Score {}", game.score), vec2(24.0, 48.0), 40, INK);
        paper.draw_text(&format!("Best {}", self.best), vec2(24.0, 84.0), 28, Color { a: 0.7, ..INK });
        let preview = self.camera.world_to_screen(PREVIEW + vec2(0.0, 1.0));
        let label = "Then";
        paper.draw_text(label, preview - vec2(paper.text_width(label, 28) / 2.0, 0.0), 28, INK);
        if game.game_over {
            paper.draw_centered_text("The jar is full!", screen_height() / 2.0 - 20.0, 56, INK);
            paper.draw_centered_text("Click to play again", screen_height() / 2.0 + 20.0, 30, INK);
        } else if game.fruits.is_empty() {
            paper.draw_centered_text("Click to drop a fruit: two alike make a bigger one", 48.0, 30, INK);
        }
        paper.draw_text("R: start again", vec2(24.0, screen_height() - 24.0), 26, Color { a: 0.6, ..INK });
    }
}

/// A round fruit with a leaf and a little face, turning with its body.
fn draw_fruit(pen: &PaperRenderer, center: Vec2, angle: f32, kind: usize) {
    let radius = RADII[kind];
    let at = |x: f32, y: f32| center + Vec2::from_angle(angle).rotate(radius * vec2(x, y));
    pen.draw_polygon(&[at(0.0, 0.9), at(0.45, 1.25), at(0.15, 1.0)], LEAF);
    pen.draw_circle(center, radius, FRUIT_COLORS[kind]);
    let face = darken(FRUIT_COLORS[kind], 0.35);
    for side in [-1.0, 1.0] {
        draw_circle(at(side * 0.3, 0.12).x, at(side * 0.3, 0.12).y, 0.09 * radius, face);
    }
    let smile: Vec<Vec2> =
        (0..=6).map(|i| at(0.0, -0.05) + 0.28 * radius * rotated(angle, PI + PI * i as f32 / 6.0)).collect();
    for pair in smile.windows(2) {
        pen.draw_thick_line(pair[0], pair[1], 0.05 * radius, face);
    }
    // A highlight.
    draw_circle(at(-0.45, 0.45).x, at(-0.45, 0.45).y, 0.14 * radius, Color::new(1.0, 1.0, 1.0, 0.55));
}

fn rotated(angle: f32, direction: f32) -> Vec2 {
    Vec2::from_angle(angle + direction)
}

/// A dotted line under the held fruit, down to the floor of the jar.
fn draw_guide(pen: &PaperRenderer, top: Vec2, kind: usize) {
    let mut y = top.y - RADII[kind] - 0.3;
    while y > 0.2 {
        pen.draw_thick_line(vec2(top.x, y), vec2(top.x, y - 0.2), 0.04, Color { a: 0.3, ..INK });
        y -= 0.45;
    }
}

/// The line not to cross, dashed, turning red and blinking while fruits are above it.
fn draw_top_line(pen: &PaperRenderer, overflow: f32) {
    let color = if overflow > 0.0 {
        Color { a: 0.5 + 0.5 * (overflow * 10.0).sin().abs(), ..DANGER }
    } else {
        Color { a: 0.35, ..INK }
    };
    let mut x = -JAR_SIZE.x / 2.0;
    while x < JAR_SIZE.x / 2.0 {
        pen.draw_thick_line(vec2(x, TOP_LINE), vec2((x + 0.3).min(JAR_SIZE.x / 2.0), TOP_LINE), 0.06, color);
        x += 0.5;
    }
}

/// A different game every time.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, reason = "any bits will do")]
fn seed() -> u32 {
    (macroquad::miniquad::date::now() * 1000.0) as u32
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
