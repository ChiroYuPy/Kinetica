//! Hill climb: drive as far as you can over the hills. Grab fuel cans to keep going, and do not
//! land on your head.

mod car;
mod game;
mod level;

use kinetica::shape::Shape;
use kinetica_macroquad::{DebugRenderer, FixedTimestep, WorldCamera, draw_centered_text};
use macroquad::prelude::*;

use crate::car::{Controls, HEAD_CENTER, HEAD_RADIUS, WHEEL_RADIUS};
use crate::game::{FULL_TANK, Game, State};
use crate::level::{COIN_RADIUS, FUEL_SIZE, TRACK_LENGTH};

const SKY: Color = Color::new(0.45, 0.65, 0.85, 1.0);
const HILLS: Color = Color::new(0.35, 0.55, 0.3, 1.0);
const GRASS: Color = Color::new(0.5, 0.75, 0.35, 1.0);
const DIRT: Color = Color::new(0.4, 0.3, 0.2, 1.0);
const BODYWORK: Color = Color::new(0.85, 0.2, 0.2, 1.0);
const BODYWORK_EDGE: Color = Color::new(0.5, 0.1, 0.1, 1.0);
const STRUT: Color = Color::new(0.3, 0.3, 0.3, 1.0);
const TIRE: Color = Color::new(0.15, 0.15, 0.17, 1.0);
const RIM: Color = Color::new(0.75, 0.75, 0.8, 1.0);
const SKIN: Color = Color::new(0.95, 0.8, 0.65, 1.0);
const HELMET: Color = Color::new(0.2, 0.35, 0.85, 1.0);
const COIN: Color = Color::new(1.0, 0.85, 0.2, 1.0);
const FUEL: Color = Color::new(0.85, 0.15, 0.15, 1.0);
const PANEL: Color = Color::new(0.0, 0.0, 0.0, 0.5);
/// How far the ground is drawn below its surface, in meters.
const GROUND_DEPTH: f32 = 30.0;
/// The camera looks this far ahead of the car, in meters.
const LOOK_AHEAD: f32 = 4.0;

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Hill Climb")
}

struct App {
    game: Game,
    camera: WorldCamera,
    timestep: FixedTimestep,
    best: u32,
}

impl App {
    fn new(best: u32) -> Self {
        let game = Game::new();
        let camera = WorldCamera::new(game.car.position(&game.world) + vec2(LOOK_AHEAD, 1.0), 16.0);
        Self { game, camera, timestep: FixedTimestep::default(), best }
    }

    fn update(&mut self) {
        let finished = self.game.state != State::Driving;
        if is_key_pressed(KeyCode::R) || (finished && is_key_pressed(KeyCode::Enter)) {
            *self = Self::new(self.best);
            return;
        }
        let forward = is_key_down(KeyCode::Right) || is_key_down(KeyCode::D);
        let backward = is_key_down(KeyCode::Left) || is_key_down(KeyCode::A);
        let controls = Controls { throttle: f32::from(i8::from(forward) - i8::from(backward)) };
        for _ in 0..self.timestep.advance(get_frame_time()) {
            self.game.step(controls);
            let target = self.game.car.position(&self.game.world) + vec2(LOOK_AHEAD, 1.0);
            self.camera.follow(target, self.timestep.dt());
        }
        self.best = self.best.max(self.game.score());
    }

    fn draw(&self) {
        clear_background(SKY);
        self.camera.apply();
        self.draw_ground();
        self.draw_pickups();
        self.draw_car();
        set_default_camera();
        self.draw_hud();
    }

    /// The visible part of the ground, filled down below the screen.
    fn draw_ground(&self) {
        let center = self.camera.center();
        let half_width = self.camera.view_height() * screen_width() / screen_height() / 2.0 + 2.0;
        // Distant hills, scrolling at half speed.
        let far_height = |x: f32| {
            let far = x - 0.5 * center.x;
            center.y - 1.0 + 2.5 * (0.08 * far).sin() + 1.2 * (0.21 * far).sin()
        };
        let bottom = center.y - self.camera.view_height();
        let mut x = center.x - half_width;
        while x < center.x + half_width {
            let (a, b) = (vec2(x, far_height(x)), vec2(x + 1.0, far_height(x + 1.0)));
            draw_triangle(a, b, vec2(b.x, bottom), HILLS);
            draw_triangle(a, vec2(b.x, bottom), vec2(a.x, bottom), HILLS);
            x += 1.0;
        }

        let terrain = &self.game.level.terrain;
        // Solid rock behind the start.
        if let Some(&first) = terrain.first() {
            draw_rectangle(first.x - 50.0, first.y - GROUND_DEPTH, 50.0, GROUND_DEPTH + 10.0, DIRT);
        }
        for pair in terrain.windows(2).filter(|pair| (pair[0].x - center.x).abs() < half_width) {
            let (a, b) = (pair[0], pair[1]);
            let (a_low, b_low) = (a - vec2(0.0, GROUND_DEPTH), b - vec2(0.0, GROUND_DEPTH));
            draw_triangle(a, b, b_low, DIRT);
            draw_triangle(a, b_low, a_low, DIRT);
            let grass = vec2(0.0, -0.3);
            draw_triangle(a, b, b + grass, GRASS);
            draw_triangle(a, b + grass, a + grass, GRASS);
        }
        // The finish line, a checkered banner.
        let finish = vec2(TRACK_LENGTH, level::height(TRACK_LENGTH));
        for row in 0..10 {
            for column in 0..2 {
                let color = if (row + column) % 2 == 0 { WHITE } else { BLACK };
                draw_rectangle(finish.x - 0.4 + 0.4 * column as f32, finish.y + 0.4 * row as f32, 0.4, 0.4, color);
            }
        }
    }

    fn draw_pickups(&self) {
        let world = &self.game.world;
        let time = self.game.time;
        for &coin in &self.game.level.coins {
            let center = world[coin].position();
            let width = COIN_RADIUS * (3.0 * time + center.x).cos().abs().max(0.15);
            draw_ellipse(center.x, center.y, width, COIN_RADIUS, 0.0, COIN);
        }
        for &can in &self.game.level.fuel_cans {
            let corner = world[can].position() - Vec2::splat(FUEL_SIZE / 2.0);
            draw_rectangle(corner.x, corner.y, FUEL_SIZE, FUEL_SIZE, FUEL);
            // The cap, and a white stripe across the can.
            draw_rectangle(corner.x + FUEL_SIZE / 2.0 - 0.1, corner.y + FUEL_SIZE, 0.2, 0.12, FUEL);
            draw_rectangle(corner.x, corner.y + FUEL_SIZE / 2.0 - 0.06, FUEL_SIZE, 0.12, WHITE);
        }
    }

    fn draw_car(&self) {
        let world = &self.game.world;
        let car = &self.game.car;
        let mut renderer = DebugRenderer::new(&self.camera);
        // Suspension struts, behind everything.
        let chassis = &world[car.chassis];
        for &wheel in &car.wheels {
            let wheel = world[wheel].position();
            let top = chassis.world_point(chassis.local_point(wheel) * vec2(1.0, 0.0));
            renderer.draw_segment(top, wheel, STRUT);
        }
        // The driver, then the bodywork over them.
        let head = chassis.world_point(HEAD_CENTER);
        let neck = chassis.world_point(HEAD_CENTER - vec2(0.0, 0.5));
        draw_line(neck.x, neck.y, head.x, head.y, 0.25, SKIN);
        draw_circle(head.x, head.y, HEAD_RADIUS, SKIN);
        let helmet = chassis.world_point(HEAD_CENTER + vec2(-0.05, 0.08));
        draw_circle(helmet.x, helmet.y, HEAD_RADIUS * 0.85, HELMET);
        // Solid bodywork, with a darker outline.
        for &collider in chassis.colliders() {
            if let Shape::Polygon(polygon) = world[collider].shape() {
                let vertices: Vec<Vec2> = polygon.vertices().iter().map(|&v| chassis.world_point(v)).collect();
                for pair in vertices[1..].windows(2) {
                    draw_triangle(vertices[0], pair[0], pair[1], BODYWORK);
                }
                for (i, &start) in vertices.iter().enumerate() {
                    renderer.draw_segment(start, vertices[(i + 1) % vertices.len()], BODYWORK_EDGE);
                }
            }
        }
        // Wheels with spokes, so they visibly turn.
        for &wheel in &car.wheels {
            let wheel = &world[wheel];
            let center = wheel.position();
            draw_circle(center.x, center.y, WHEEL_RADIUS, TIRE);
            draw_circle(center.x, center.y, WHEEL_RADIUS * 0.55, RIM);
            for spoke in 0..3 {
                let direction = wheel.rotation().rotate(Vec2::from_angle(spoke as f32 * std::f32::consts::TAU / 3.0));
                let end = center + WHEEL_RADIUS * 0.55 * direction;
                draw_line(center.x, center.y, end.x, end.y, 0.06, TIRE);
            }
        }
    }

    fn draw_hud(&self) {
        let game = &self.game;
        draw_rectangle(10.0, 10.0, 330.0, 110.0, PANEL);
        draw_text(format!("Distance {:.0} m", game.distance), 24.0, 42.0, 30.0, WHITE);
        draw_text(
            format!("Coins {}   Score {}   Best {}", game.coins, game.score(), self.best),
            24.0,
            72.0,
            22.0,
            WHITE,
        );
        // Fuel gauge.
        let fuel = game.fuel / FULL_TANK;
        draw_text("Fuel", 24.0, 104.0, 22.0, WHITE);
        draw_rectangle(80.0, 90.0, 240.0, 18.0, Color::new(0.2, 0.2, 0.2, 1.0));
        let gauge = if fuel < 0.25 { FUEL } else { Color::new(0.3, 0.85, 0.3, 1.0) };
        draw_rectangle(80.0, 90.0, 240.0 * fuel, 18.0, gauge);
        // Progress along the track.
        let bar = screen_width() - 40.0;
        draw_rectangle(20.0, screen_height() - 46.0, bar, 6.0, PANEL);
        draw_circle(20.0 + bar * game.progress(), screen_height() - 43.0, 8.0, WHITE);
        draw_text(
            "Right/D: accelerate   Left/A: brake and reverse   in the air, they tilt the car   R: restart",
            20.0,
            screen_height() - 14.0,
            22.0,
            WHITE,
        );

        let message = match game.state {
            State::Driving => None,
            State::Crashed => Some("Ouch! You landed on your head."),
            State::OutOfFuel => Some("Out of fuel."),
            State::Finished => Some("You reached the end of the track!"),
        };
        if let Some(message) = message {
            draw_centered_text(message, screen_height() * 0.35, 48, WHITE);
            draw_centered_text("Press Enter to drive again", screen_height() * 0.43, 28, WHITE);
        }
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut app = App::new(0);
    loop {
        app.update();
        app.draw();
        next_frame().await;
    }
}
