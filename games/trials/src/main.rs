//! Trials: drive a little car over hills, a plank bridge, a seesaw and a jump to the flag.

mod car;
mod game;
mod track;

use kinetica::prelude::*;
use kinetica_macroquad::paper::{INK, PAPER, darken};
use kinetica_macroquad::{FixedTimestep, Paper, PaperRenderer, WorldCamera};
use macroquad::prelude::*;

use crate::car::{AXLES, CABIN_OFFSET, CABIN_SIZE, Car, Controls, HEAD, HEAD_RADIUS, WHEEL_RADIUS};
use crate::game::{Game, State};
use crate::track::{CHECKPOINTS, FINISH, PIT_DEPTH, PITS, ground_height};

const FAR_HILLS: Color = Color::new(0.8, 0.85, 0.74, 1.0);
const NEAR_HILLS: Color = Color::new(0.7, 0.79, 0.62, 1.0);
const DIRT: Color = Color::new(0.72, 0.56, 0.4, 1.0);
const GRASS: Color = Color::new(0.5, 0.7, 0.38, 1.0);
const PLANK: Color = Color::new(0.82, 0.62, 0.4, 1.0);
const CARDBOARD: Color = Color::new(0.85, 0.7, 0.5, 1.0);
const BODYWORK: Color = Color::new(0.88, 0.32, 0.28, 1.0);
const WINDOW: Color = Color::new(0.75, 0.88, 0.95, 1.0);
const TIRE: Color = Color::new(0.3, 0.29, 0.3, 1.0);
const HUB: Color = Color::new(0.82, 0.82, 0.8, 1.0);
const SKIN: Color = Color::new(0.96, 0.8, 0.65, 1.0);
const HELMET: Color = Color::new(0.3, 0.5, 0.85, 1.0);
const FLAG: Color = Color::new(0.9, 0.3, 0.3, 1.0);
const CHECKPOINT_FLAG: Color = Color::new(0.97, 0.8, 0.35, 1.0);
const CHECKPOINT_PASSED: Color = Color::new(0.5, 0.8, 0.45, 1.0);
/// The camera looks this far ahead of the car, in meters.
const LOOK_AHEAD: f32 = 4.0;
/// Thickness of the grass on the ground, in meters.
const GRASS_DEPTH: f32 = 0.35;

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Trials")
}

struct App {
    game: Game,
    /// Fastest finish so far, in seconds.
    best_time: Option<f32>,
    paper: Paper,
    camera: WorldCamera,
    timestep: FixedTimestep,
}

impl App {
    fn new() -> Self {
        let game = Game::new();
        let camera = WorldCamera::new(game.car.position(&game.world) + vec2(LOOK_AHEAD, 1.0), 16.0);
        Self { game, best_time: None, paper: Paper::new(), camera, timestep: FixedTimestep::default() }
    }

    fn update(&mut self) {
        if is_key_pressed(KeyCode::R) || (self.game.state == State::Finished && is_key_pressed(KeyCode::Enter)) {
            self.game = Game::new();
        }
        let pedal = |keys: &[KeyCode]| keys.iter().any(|&key| is_key_down(key));
        let forward = pedal(&[KeyCode::Right, KeyCode::D, KeyCode::Up, KeyCode::W]);
        let backward = pedal(&[KeyCode::Left, KeyCode::A, KeyCode::Down, KeyCode::S]);
        let controls = Controls { throttle: f32::from(i8::from(forward) - i8::from(backward)) };
        for _ in 0..self.timestep.advance(get_frame_time()) {
            let was_finished = self.game.state == State::Finished;
            self.game.step(controls);
            if !was_finished && self.game.state == State::Finished {
                let time = self.game.time;
                self.best_time = Some(self.best_time.map_or(time, |best| best.min(time)));
            }
            self.camera.follow(self.game.car.position(&self.game.world) + vec2(LOOK_AHEAD, 1.0), self.timestep.dt());
        }
    }

    fn draw(&self) {
        self.paper.draw_sheet(PAPER);
        self.camera.apply();
        let pen = PaperRenderer::new(&self.camera);
        let (left, right) = self.visible_x();
        self.draw_scenery(&pen);
        self.draw_track(&pen, left, right);
        draw_car(&pen, &self.game.world, &self.game.car);
        set_default_camera();
        self.draw_hud();
    }

    /// The range of x the camera sees, with a margin.
    fn visible_x(&self) -> (f32, f32) {
        let half_width = self.camera.view_height() * screen_width() / screen_height() / 2.0 + 2.0;
        (self.camera.center().x - half_width, self.camera.center().x + half_width)
    }

    /// Two rows of hills scrolling slower than the ground, so they look far away.
    fn draw_scenery(&self, pen: &PaperRenderer) {
        pen.draw_hills(&self.camera, 0.25, -1.5, 4.0, FAR_HILLS);
        pen.draw_hills(&self.camera, 0.5, -3.5, 2.5, NEAR_HILLS);
    }

    fn draw_track(&self, pen: &PaperRenderer, left: f32, right: f32) {
        let game = &self.game;
        let world = &game.world;
        let track = &game.track;
        let visible: Vec<Vec2> =
            track.surface.iter().copied().filter(|point| point.x > left - 1.0 && point.x < right + 1.0).collect();
        let bottom = PIT_DEPTH - 20.0;
        // The back walls of the pits, in the shade.
        for (pit_left, pit_right) in PITS {
            let corners = [
                vec2(pit_left, PIT_DEPTH),
                vec2(pit_right, PIT_DEPTH),
                vec2(pit_right, ground_height(pit_right)),
                vec2(pit_left, ground_height(pit_left)),
            ];
            draw_triangle(corners[0], corners[1], corners[2], darken(DIRT, 0.7));
            draw_triangle(corners[0], corners[2], corners[3], darken(DIRT, 0.7));
        }
        pen.draw_ground(&visible, bottom, DIRT);
        pen.draw_grass(&visible, GRASS_DEPTH, GRASS);

        for (index, &x) in CHECKPOINTS.iter().enumerate().skip(1) {
            let color = if index <= game.checkpoint { CHECKPOINT_PASSED } else { CHECKPOINT_FLAG };
            draw_flag(pen, vec2(x, ground_height(x)), 2.0, color);
        }
        draw_flag(pen, vec2(FINISH, ground_height(FINISH)), 3.5, FLAG);

        for &plank in &track.planks {
            pen.draw_body(world, plank, PLANK);
        }
        pen.draw_body(world, track.pivot, darken(PLANK, 0.8));
        pen.draw_body(world, track.seesaw, PLANK);
        for &crate_body in &track.crates {
            pen.draw_body(world, crate_body, CARDBOARD);
            // A strip of tape across the box.
            let body = &world[crate_body];
            let half = crate::track::CRATE_SIZE / 2.0;
            pen.draw_line(
                body.world_point(vec2(-half, 0.0)),
                body.world_point(vec2(half, 0.0)),
                darken(CARDBOARD, 0.75),
            );
        }
    }

    fn draw_hud(&self) {
        let game = &self.game;
        let paper = &self.paper;
        paper.draw_text(&format!("{:.1} s", game.time), vec2(24.0, 48.0), 44, INK);
        let best = self.best_time.map_or_else(|| "-".to_owned(), |time| format!("{time:.1} s"));
        paper.draw_text(&format!("Best {best}   Crashes {}", game.crashes), vec2(24.0, 84.0), 28, INK);
        // Progress along the track.
        let (x, y, width) = (24.0, 104.0, 260.0);
        draw_rectangle(x, y, width, 10.0, Color { a: 0.15, ..INK });
        draw_rectangle(x, y, width * game.progress(), 10.0, FLAG);

        let message = match game.state {
            State::Crashed(_) => Some(("Crash!", "back to the last flag")),
            State::Finished => Some(("Finish!", "Enter: drive again")),
            State::Driving if game.time < 3.0 => {
                Some(("Reach the red flag", "Right: drive   Left: brake   in the air, they tilt the car"))
            }
            State::Driving => None,
        };
        if let Some((title, hint)) = message {
            paper.draw_centered_text(title, screen_height() * 0.28, 56, INK);
            paper.draw_centered_text(hint, screen_height() * 0.28 + 40.0, 28, INK);
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

/// The driver and the bodywork, then the wheels with spokes, so they visibly turn.
fn draw_car(pen: &PaperRenderer, world: &World, car: &Car) {
    let chassis = &world[car.chassis];
    // Suspension struts from the bodywork to the axles.
    for (offset, &wheel) in AXLES.iter().zip(&car.wheels) {
        let top = chassis.world_point(vec2(offset.x, 0.0));
        pen.draw_thick_line(top, world[wheel].position(), 0.12, darken(TIRE, 1.5));
    }
    let neck = chassis.world_point(HEAD - vec2(0.0, 0.4));
    let head = chassis.world_point(HEAD);
    pen.draw_thick_line(neck, head, 0.2, SKIN);
    pen.draw_circle(head, HEAD_RADIUS, SKIN);
    // The helmet covers the back and top of the head.
    let helmet: Vec<Vec2> = (0..=12)
        .map(|i| {
            let angle = 0.4 + 2.6 * i as f32 / 12.0;
            chassis.world_point(HEAD + 1.08 * HEAD_RADIUS * Vec2::from_angle(angle))
        })
        .collect();
    pen.draw_polygon(&helmet, HELMET);

    pen.draw_body(world, car.chassis, BODYWORK);
    // The window in the cabin, in front of the driver.
    let corner = |x: f32, y: f32| chassis.world_point(CABIN_OFFSET + 0.5 * CABIN_SIZE * vec2(x, y));
    pen.draw_polygon(&[corner(0.15, -0.55), corner(0.8, -0.55), corner(0.8, 0.65), corner(0.15, 0.65)], WINDOW);

    for &wheel in &car.wheels {
        let wheel = &world[wheel];
        let center = wheel.position();
        pen.draw_circle(center, WHEEL_RADIUS, TIRE);
        pen.draw_circle(center, 0.5 * WHEEL_RADIUS, HUB);
        for spoke in 0..3 {
            let direction = wheel.rotation().rotate(Vec2::from_angle(spoke as f32 * std::f32::consts::TAU / 3.0));
            pen.draw_line(center, center + 0.5 * WHEEL_RADIUS * direction, TIRE);
        }
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
