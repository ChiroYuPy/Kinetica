//! Slingshot: pull the sling with the mouse and knock down the paper towers to pop every
//! target.

mod game;
mod level;

use kinetica::prelude::*;
use kinetica_macroquad::paper::{INK, PAPER, darken};
use kinetica_macroquad::{FixedTimestep, Paper, PaperRenderer, WorldCamera};
use macroquad::prelude::*;

use crate::game::{DEBRIS_LIFETIME, Debris, Game, PROJECTILE_RADIUS, SHOTS_PER_LEVEL, SLING, Scrap, State};
use crate::level::{Material, TARGET_RADIUS};

const SUN: Color = Color::new(0.98, 0.82, 0.45, 1.0);
const FAR_HILLS: Color = Color::new(0.78, 0.84, 0.72, 1.0);
const NEAR_HILLS: Color = Color::new(0.67, 0.78, 0.6, 1.0);
const GRASS: Color = Color::new(0.52, 0.7, 0.42, 1.0);
const WOOD: Color = Color::new(0.86, 0.64, 0.4, 1.0);
const GLASS: Color = Color::new(0.66, 0.85, 0.92, 0.9);
const STONE: Color = Color::new(0.67, 0.67, 0.7, 1.0);
const TARGET: Color = Color::new(0.55, 0.8, 0.38, 1.0);
const PROJECTILE: Color = Color::new(0.42, 0.4, 0.42, 1.0);
const SLING_WOOD: Color = Color::new(0.55, 0.36, 0.22, 1.0);
const BAND: Color = Color::new(0.6, 0.2, 0.18, 1.0);
/// The sling can be grabbed this close to its pouch, in meters.
const GRAB_DISTANCE: f32 = 1.2;
/// The two prong tips of the sling, which the bands hang from.
const PRONGS: [Vec2; 2] = [vec2(SLING.x - 0.35, SLING.y + 0.15), vec2(SLING.x + 0.35, SLING.y + 0.15)];

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Slingshot")
}

struct App {
    game: Game,
    /// Score when the level started, kept when it is replayed.
    score_before_level: u32,
    paper: Paper,
    camera: WorldCamera,
    timestep: FixedTimestep,
    /// Where the pouch is pulled to, while the mouse holds it.
    aiming: Option<Vec2>,
}

impl App {
    fn new() -> Self {
        Self {
            game: Game::new(0, 0),
            score_before_level: 0,
            paper: Paper::new(),
            camera: WorldCamera::new(vec2(5.0, 5.2), 12.5),
            timestep: FixedTimestep::default(),
            aiming: None,
        }
    }

    fn start_level(&mut self, index: usize, score: u32) {
        self.game = Game::new(index, score);
        self.score_before_level = score;
        self.aiming = None;
    }

    fn update(&mut self) {
        let next = (self.game.level_index + 1) % Game::level_count();
        match self.game.state {
            State::Won if is_key_pressed(KeyCode::Enter) => self.start_level(next, self.game.score),
            State::Lost if is_key_pressed(KeyCode::Enter) => {
                self.start_level(self.game.level_index, self.score_before_level);
            }
            _ if is_key_pressed(KeyCode::R) => self.start_level(self.game.level_index, self.score_before_level),
            _ => {}
        }
        self.aim();
        for _ in 0..self.timestep.advance(get_frame_time()) {
            self.game.step();
        }
    }

    /// Grab the pouch, pull it, and let go to shoot.
    fn aim(&mut self) {
        let mouse = self.camera.mouse_position();
        if self.game.state != State::Aiming {
            self.aiming = None;
        } else if is_mouse_button_pressed(MouseButton::Left) && mouse.distance(SLING) < GRAB_DISTANCE {
            self.aiming = Some(Game::pouch(mouse));
        } else if let Some(pouch) = &mut self.aiming {
            *pouch = Game::pouch(mouse);
            if is_mouse_button_released(MouseButton::Left) {
                self.game.launch(*pouch);
                self.aiming = None;
            }
        }
    }

    fn draw(&self) {
        self.paper.draw_sheet(PAPER);
        self.camera.apply();
        let pen = PaperRenderer::new(&self.camera);
        draw_scenery(&pen);

        let game = &self.game;
        let world = &game.world;
        for &(block, material) in &game.blocks {
            pen.draw_body(world, block, material_color(material));
        }
        for &target in &game.targets {
            draw_target(&pen, world, target);
        }
        for &stone in &game.projectiles {
            pen.draw_body(world, stone, PROJECTILE);
        }
        for debris in &game.debris {
            draw_debris(&pen, debris);
        }
        self.draw_sling(&pen);

        set_default_camera();
        self.draw_hud();
    }

    /// The wooden fork, with the stone waiting in its pouch, or pulled back while aiming.
    fn draw_sling(&self, pen: &PaperRenderer) {
        let fork = vec2(SLING.x, SLING.y - 1.0);
        pen.draw_thick_line(vec2(SLING.x, 0.0), fork, 0.3, SLING_WOOD);
        for prong in PRONGS {
            pen.draw_thick_line(fork, prong, 0.22, SLING_WOOD);
        }
        if self.game.state != State::Aiming {
            pen.draw_line(PRONGS[0], PRONGS[1], BAND);
            return;
        }
        let pouch = self.aiming.unwrap_or(SLING);
        if self.aiming.is_some() {
            for point in self.game.trajectory(pouch) {
                draw_circle(point.x, point.y, 0.08, Color { a: 0.45, ..INK });
            }
        }
        // The back band, the stone, then the front band over it while pulled.
        pen.draw_thick_line(PRONGS[0], pouch, 0.12, BAND);
        if self.aiming.is_none() {
            pen.draw_thick_line(PRONGS[1], pouch, 0.12, BAND);
        }
        pen.draw_circle(pouch, PROJECTILE_RADIUS, PROJECTILE);
        if self.aiming.is_some() {
            pen.draw_thick_line(PRONGS[1], pouch, 0.12, BAND);
        }
    }

    fn draw_hud(&self) {
        let game = &self.game;
        let paper = &self.paper;
        paper.draw_text(&format!("Level {} - {}", game.level_index + 1, game.level_name), vec2(24.0, 44.0), 40, INK);
        paper.draw_text(&format!("Score {}", game.score), vec2(24.0, 84.0), 32, INK);
        // Stones left, as little stones.
        for i in 0..SHOTS_PER_LEVEL {
            let color = if i < game.shots_left { PROJECTILE } else { Color { a: 0.25, ..PROJECTILE } };
            draw_circle(36.0 + 30.0 * i as f32, 112.0, 10.0, color);
        }
        let message = match game.state {
            State::Aiming if game.shots_left == SHOTS_PER_LEVEL && game.level_index == 0 => {
                Some(("Pull the sling with the mouse and let go", ""))
            }
            State::Won if game.is_last_level() => Some(("Every tower is down!", "Enter: play again from the start")),
            State::Won => Some(("Level cleared!", "Enter: next level")),
            State::Lost => Some(("Out of stones", "Enter: try again")),
            _ => None,
        };
        if let Some((title, hint)) = message {
            paper.draw_centered_text(title, screen_height() * 0.3, 56, INK);
            paper.draw_centered_text(hint, screen_height() * 0.3 + 44.0, 32, INK);
        }
        paper.draw_text("R: restart the level", vec2(24.0, screen_height() - 24.0), 26, Color { a: 0.6, ..INK });
    }
}

fn material_color(material: Material) -> Color {
    match material {
        Material::Wood => WOOD,
        Material::Glass => GLASS,
        Material::Stone => STONE,
    }
}

/// The sun and two rows of hills behind the ground.
fn draw_scenery(pen: &PaperRenderer) {
    pen.draw_circle(vec2(13.0, 9.6), 1.0, SUN);
    let hills = |base: f32, height: f32, waves: f32, phase: f32| -> Vec<Vec2> {
        (0..=60)
            .map(|i| {
                let x = -10.0 + i as f32 * 0.5;
                vec2(x, base + height * (0.5 + 0.5 * (waves * x + phase).sin()))
            })
            .collect()
    };
    pen.draw_ground(&hills(1.5, 3.0, 0.18, 1.0), -1.0, FAR_HILLS);
    pen.draw_ground(&hills(0.5, 1.8, 0.31, 4.0), -1.0, NEAR_HILLS);
    pen.draw_ground(&[vec2(-30.0, 0.0), vec2(40.0, 0.0)], -12.0, GRASS);
}

/// A round paper creature that looks the way it is turned.
fn draw_target(pen: &PaperRenderer, world: &World, target: BodyHandle) {
    let body = &world[target];
    pen.draw_body(world, target, TARGET);
    // Face features, in fractions of the radius.
    let at = |x: f32, y: f32| body.world_point(TARGET_RADIUS * vec2(x, y));
    for side in [-1.0, 1.0] {
        let eye = at(0.35 * side, 0.2);
        draw_circle(eye.x, eye.y, 0.22 * TARGET_RADIUS, WHITE);
        draw_circle(eye.x, eye.y, 0.11 * TARGET_RADIUS, INK);
    }
    let mouth = [at(-0.3, -0.3), at(0.0, -0.48), at(0.3, -0.3)];
    pen.draw_line(mouth[0], mouth[1], darken(TARGET, 0.5));
    pen.draw_line(mouth[1], mouth[2], darken(TARGET, 0.5));
}

/// A scrap of paper, fading out.
fn draw_debris(pen: &PaperRenderer, debris: &Debris) {
    let color = match debris.kind {
        Scrap::Block(material) => material_color(material),
        Scrap::Target => TARGET,
    };
    let fade = 1.0 - debris.age / DEBRIS_LIFETIME;
    let corners = [vec2(-1.0, -0.6), vec2(1.0, -0.8), vec2(0.2, 0.9)]
        .map(|corner| debris.position + debris.size * Vec2::from_angle(debris.angle).rotate(corner));
    pen.draw_polygon(&corners, Color { a: color.a * fade, ..color });
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
