//! Platformer: run and jump across a paper world to the flag, picking up coins on the way.

mod game;
mod level;
mod player;

use kinetica::prelude::*;
use kinetica_macroquad::paper::{INK, PAPER, darken};
use kinetica_macroquad::{FixedTimestep, Paper, PaperRenderer, WorldCamera};
use macroquad::prelude::*;

use crate::game::{Game, State};
use crate::level::{BOX_SIZE, CHECKPOINTS, COIN_RADIUS, GOAL, SPIKE_HEIGHT};
use crate::player::{HEIGHT, Input, RADIUS};

const FAR_HILLS: Color = Color::new(0.8, 0.83, 0.88, 1.0);
const NEAR_HILLS: Color = Color::new(0.7, 0.76, 0.84, 1.0);
const DIRT: Color = Color::new(0.62, 0.52, 0.45, 1.0);
const GRASS: Color = Color::new(0.48, 0.68, 0.4, 1.0);
const WOOD: Color = Color::new(0.82, 0.62, 0.4, 1.0);
const LIGHT_WOOD: Color = Color::new(0.9, 0.76, 0.55, 1.0);
const CARDBOARD: Color = Color::new(0.85, 0.7, 0.5, 1.0);
const COIN: Color = Color::new(0.98, 0.8, 0.3, 1.0);
const SPIKE: Color = Color::new(0.6, 0.6, 0.65, 1.0);
const PLAYER: Color = Color::new(0.35, 0.65, 0.7, 1.0);
const SCARF: Color = Color::new(0.9, 0.35, 0.3, 1.0);
const FLAG: Color = Color::new(0.9, 0.3, 0.3, 1.0);
const CHECKPOINT_FLAG: Color = Color::new(0.97, 0.8, 0.35, 1.0);
const CHECKPOINT_PASSED: Color = Color::new(0.5, 0.8, 0.45, 1.0);
const GRASS_DEPTH: f32 = 0.3;

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Platformer")
}

struct App {
    game: Game,
    paper: Paper,
    camera: WorldCamera,
    timestep: FixedTimestep,
    /// The jump key went down since the last step.
    jump_pressed: bool,
}

impl App {
    fn new() -> Self {
        let game = Game::new();
        let camera = WorldCamera::new(game.player.position(&game.world) + vec2(3.0, 2.0), 13.0);
        Self { game, paper: Paper::new(), camera, timestep: FixedTimestep::default(), jump_pressed: false }
    }

    fn update(&mut self) {
        if is_key_pressed(KeyCode::R) || (self.game.state == State::Won && is_key_pressed(KeyCode::Enter)) {
            *self = Self::new();
            return;
        }
        let any_down = |keys: &[KeyCode]| keys.iter().any(|&key| is_key_down(key));
        let jump_keys = [KeyCode::Space, KeyCode::Up, KeyCode::W];
        // A press between two steps must not be lost: keep it until a step reads it.
        self.jump_pressed |= jump_keys.iter().any(|&key| is_key_pressed(key));
        let right = any_down(&[KeyCode::Right, KeyCode::D]);
        let left = any_down(&[KeyCode::Left, KeyCode::A]);
        let run = f32::from(i8::from(right) - i8::from(left));
        for _ in 0..self.timestep.advance(get_frame_time()) {
            let input = Input { run, jump_pressed: self.jump_pressed, jump_held: any_down(&jump_keys) };
            self.jump_pressed = false;
            self.game.step(input);
            let target = self.game.player.position(&self.game.world) + vec2(3.0 * self.game.player.facing, 2.0);
            self.camera.follow(target, self.timestep.dt());
        }
    }

    fn draw(&self) {
        self.paper.draw_sheet(PAPER);
        self.camera.apply();
        let pen = PaperRenderer::new(&self.camera);
        pen.draw_hills(&self.camera, 0.2, 0.0, 4.0, FAR_HILLS);
        pen.draw_hills(&self.camera, 0.45, -2.5, 3.0, NEAR_HILLS);
        self.draw_level(&pen);
        draw_player(&pen, &self.game);
        set_default_camera();
        self.draw_hud();
    }

    fn draw_level(&self, pen: &PaperRenderer) {
        let game = &self.game;
        let (world, level) = (&game.world, &game.level);

        for pit in &level.pits {
            // The back wall in the shade, then spikes half a meter wide.
            let width = pit.right - pit.left;
            draw_rectangle(pit.left, pit.floor, width, pit.rim - pit.floor, darken(DIRT, 0.7));
            let mut x = pit.left;
            while x < pit.right {
                let tip = vec2(x + 0.25, pit.floor + SPIKE_HEIGHT);
                pen.draw_polygon(&[vec2(x, pit.floor), vec2(x + 0.5, pit.floor), tip], SPIKE);
                x += 0.5;
            }
        }
        pen.draw_ground(&level.surface, -20.0, DIRT);
        pen.draw_grass(&level.surface, GRASS_DEPTH, GRASS);

        for (index, &checkpoint) in CHECKPOINTS.iter().enumerate().skip(1) {
            let color = if index <= game.checkpoint { CHECKPOINT_PASSED } else { CHECKPOINT_FLAG };
            draw_flag(pen, checkpoint - vec2(1.5, 1.2), 2.0, color);
        }
        draw_flag(pen, vec2(GOAL, 0.0), 3.5, FLAG);

        for &platform in &level.one_way {
            pen.draw_body(world, platform, LIGHT_WOOD);
        }
        for mover in &level.movers {
            pen.draw_body(world, mover.body, WOOD);
        }
        for &plank in &level.planks {
            pen.draw_body(world, plank, WOOD);
        }
        for &crate_body in &level.boxes {
            pen.draw_body(world, crate_body, CARDBOARD);
            let body = &world[crate_body];
            let half = BOX_SIZE / 2.0;
            pen.draw_line(
                body.world_point(vec2(-half, 0.0)),
                body.world_point(vec2(half, 0.0)),
                darken(CARDBOARD, 0.75),
            );
        }
        for &coin in &level.coins {
            // Coins bob gently, each at its own pace.
            let center = world[coin].position();
            let bob = 0.08 * (3.0 * game.time + center.x).sin();
            pen.draw_circle(center + vec2(0.0, bob), COIN_RADIUS, COIN);
            pen.draw_circle(center + vec2(0.0, bob), 0.55 * COIN_RADIUS, darken(COIN, 0.9));
        }
    }

    fn draw_hud(&self) {
        let game = &self.game;
        let paper = &self.paper;
        paper.draw_text(&format!("Coins {} / {}", game.coins, game.total_coins()), vec2(24.0, 48.0), 40, INK);
        paper.draw_text(&format!("{:.1} s   Falls {}", game.time, game.deaths), vec2(24.0, 84.0), 28, INK);
        let message = match game.state {
            State::Won => Some(("You made it!".to_owned(), "Enter: play again".to_owned())),
            State::Playing if game.time < 4.0 => Some((
                "Reach the red flag".to_owned(),
                "Left / Right: run   Space: jump (hold to jump higher)".to_owned(),
            )),
            State::Playing => None,
        };
        if let Some((title, hint)) = message {
            paper.draw_centered_text(&title, screen_height() * 0.25, 56, INK);
            paper.draw_centered_text(&hint, screen_height() * 0.25 + 40.0, 28, INK);
        }
        paper.draw_text("R: restart", vec2(24.0, screen_height() - 24.0), 26, Color { a: 0.6, ..INK });
    }
}

/// A pole with a triangular flag at its top.
fn draw_flag(pen: &PaperRenderer, foot: Vec2, height: f32, color: Color) {
    let top = foot + vec2(0.0, height);
    pen.draw_thick_line(foot, top, 0.1, INK);
    pen.draw_polygon(&[top - vec2(0.0, 0.8), top + vec2(1.1, -0.4), top], color);
}

/// A paper character with a scarf, looking where it runs.
fn draw_player(pen: &PaperRenderer, game: &Game) {
    let world = &game.world;
    let player = &game.player;
    let body = &world[player.body];
    let center = body.position();
    let facing = player.facing;
    // The scarf flutters behind, more when running.
    let speed = body.linear_velocity().x.abs().min(6.0);
    let neck = center + vec2(0.0, 0.15);
    let tail_end = neck + vec2(-facing * (0.3 + 0.08 * speed), -0.1 + 0.05 * (12.0 * game.time).sin() * speed / 6.0);
    pen.draw_thick_line(neck, tail_end, 0.16, SCARF);

    pen.draw_body(world, player.body, PLAYER);
    pen.draw_thick_line(neck - vec2(RADIUS * 0.9, 0.0), neck + vec2(RADIUS * 0.9, 0.0), 0.16, SCARF);
    let eye_height = center.y + HEIGHT / 2.0 - 0.05;
    for offset in [0.02, 0.2] {
        let eye = vec2(center.x + facing * offset, eye_height);
        draw_circle(eye.x, eye.y, 0.08, WHITE);
        draw_circle(eye.x + 0.03 * facing, eye.y, 0.04, INK);
    }
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
