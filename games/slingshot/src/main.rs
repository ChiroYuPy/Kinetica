//! Slingshot: pull the sling with the mouse and knock down the towers to pop every target.

mod game;
mod level;

use kinetica::prelude::*;
use kinetica_macroquad::{DebugRenderer, FixedTimestep, WorldCamera, draw_centered_text};
use macroquad::prelude::*;

use crate::game::{Game, POP_DURATION, PROJECTILE_RADIUS, SLING, State};
use crate::level::{Material, TARGET_RADIUS};

const SKY: Color = Color::new(0.1, 0.12, 0.17, 1.0);
const GROUND: Color = Color::new(0.45, 0.35, 0.25, 1.0);
const WOOD: Color = Color::new(0.85, 0.6, 0.3, 1.0);
const STONE: Color = Color::new(0.6, 0.62, 0.68, 1.0);
const TARGET: Color = Color::new(0.4, 0.85, 0.35, 1.0);
const POP: Color = Color::new(0.5, 1.0, 0.4, 1.0);
const PROJECTILE: Color = Color::new(0.9, 0.25, 0.25, 1.0);
const MESSAGE: Color = Color::new(1.0, 0.9, 0.5, 1.0);
const BAND: Color = Color::new(0.55, 0.3, 0.2, 1.0);
const TRAJECTORY: Color = Color::new(1.0, 1.0, 1.0, 0.5);
/// The sling can be grabbed this close to its rest position, in meters.
const GRAB_DISTANCE: f32 = 1.2;

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Slingshot")
}

struct App {
    game: Game,
    camera: WorldCamera,
    timestep: FixedTimestep,
    /// Where the sling is pulled to, while the mouse holds it.
    aiming: Option<Vec2>,
}

impl App {
    fn new(game: Game) -> Self {
        Self { game, camera: WorldCamera::new(vec2(8.0, 4.0), 11.0), timestep: FixedTimestep::default(), aiming: None }
    }

    fn update(&mut self) {
        let mouse = self.camera.mouse_position();
        if self.game.can_shoot() {
            if is_mouse_button_pressed(MouseButton::Left) && mouse.distance(SLING) < GRAB_DISTANCE {
                self.aiming = Some(mouse);
            }
            if self.aiming.is_some() {
                self.aiming = Some(mouse);
                if is_mouse_button_released(MouseButton::Left) {
                    self.game.shoot(mouse);
                    self.aiming = None;
                }
            }
        }

        match self.game.state {
            State::Won if is_key_pressed(KeyCode::Space) || is_mouse_button_pressed(MouseButton::Left) => {
                *self = Self::new(self.game.next_level());
            }
            _ if is_key_pressed(KeyCode::R) => *self = Self::new(Game::new(self.game.level_index)),
            _ => {}
        }

        for _ in 0..self.timestep.advance(get_frame_time()) {
            self.game.step();
        }
    }

    fn draw(&self) {
        clear_background(SKY);
        self.camera.apply();
        let mut renderer = DebugRenderer::new(&self.camera);
        let world = &self.game.world;

        if let Some(ground) = self.game.level.ground {
            renderer.draw_body(world, ground, GROUND);
        }
        for &(block, material) in &self.game.level.blocks {
            let color = match material {
                Material::Wood => WOOD,
                Material::Stone => STONE,
            };
            renderer.draw_body(world, block, color);
        }
        for &target in &self.game.level.targets {
            renderer.draw_body(world, target, TARGET);
        }
        for &projectile in &self.game.projectiles {
            renderer.draw_body(world, projectile, PROJECTILE);
        }
        // Popped targets leave a ring that grows and fades.
        for pop in &self.game.pops {
            let progress = pop.age / POP_DURATION;
            let radius = TARGET_RADIUS * (1.0 + 2.0 * progress);
            draw_circle_lines(pop.position.x, pop.position.y, radius, 0.05, Color { a: 1.0 - progress, ..POP });
        }
        self.draw_sling(&mut renderer);

        set_default_camera();
        self.draw_hud();
    }

    fn draw_sling(&self, renderer: &mut DebugRenderer) {
        let fork = [SLING + vec2(-0.3, 0.2), SLING + vec2(0.3, 0.2)];
        draw_line(SLING.x, 0.0, SLING.x, SLING.y - 0.2, 0.2, BAND);
        for prong in fork {
            draw_line(SLING.x, SLING.y - 0.2, prong.x, prong.y, 0.15, BAND);
        }
        let Some(aim) = self.aiming else {
            if self.game.can_shoot() {
                renderer.draw_circle(SLING, PROJECTILE_RADIUS, Rot::IDENTITY, PROJECTILE);
            }
            return;
        };
        let position = Game::pulled_position(aim);
        for prong in fork {
            draw_line(prong.x, prong.y, position.x, position.y, 0.08, BAND);
        }
        renderer.draw_circle(position, PROJECTILE_RADIUS, Rot::IDENTITY, PROJECTILE);

        // Where the shot would go, ignoring collisions.
        let velocity = Game::launch_velocity(aim);
        let gravity = self.game.world.gravity();
        for i in 1..30 {
            let t = i as f32 * 0.08;
            let point = position + velocity * t + 0.5 * gravity * t * t;
            draw_circle(point.x, point.y, 0.06, TRAJECTORY);
        }
    }

    fn draw_hud(&self) {
        let game = &self.game;
        let line = |text: &str, y: f32, size: f32| {
            draw_text(text, 20.0, y, size, WHITE);
        };
        line(&format!("Level {}   Score {}", game.level_index + 1, game.score), 36.0, 32.0);
        line(&format!("Shots left: {}   Targets: {}", game.shots_left, game.level.targets.len()), 66.0, 26.0);
        line("Drag the ball back from the sling and release. R: restart level", screen_height() - 20.0, 22.0);

        let message = match game.state {
            State::Won if game.has_next_level() => Some("Level cleared! Click for the next one"),
            State::Won => Some("All levels cleared! Click to play again"),
            State::Lost => Some("Out of shots. Press R to retry"),
            State::Playing => None,
        };
        if let Some(message) = message {
            draw_centered_text(message, screen_height() / 3.0, 40, MESSAGE);
        }
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut app = App::new(Game::new(0));
    loop {
        app.update();
        app.draw();
        next_frame().await;
    }
}
