//! Platformer: run and jump through the level, collect every coin, then reach the flag.

mod game;
mod level;
mod player;

use kinetica_macroquad::{DebugRenderer, FixedTimestep, WorldCamera, draw_centered_text};
use macroquad::prelude::*;

use crate::game::{Game, State};
use crate::level::{COIN_RADIUS, GOAL_HEIGHT};
use crate::player::Input;

const SKY: Color = Color::new(0.12, 0.15, 0.22, 1.0);
const GROUND: Color = Color::new(0.25, 0.35, 0.25, 1.0);
const GROUND_EDGE: Color = Color::new(0.45, 0.65, 0.35, 1.0);
const PLATFORM: Color = Color::new(0.7, 0.55, 0.35, 1.0);
const MOVING_PLATFORM: Color = Color::new(0.45, 0.6, 0.95, 1.0);
const CRATE: Color = Color::new(0.8, 0.55, 0.3, 1.0);
const PLAYER: Color = Color::new(0.95, 0.45, 0.35, 1.0);
const COIN: Color = Color::new(1.0, 0.85, 0.25, 1.0);
const SPIKES: Color = Color::new(0.8, 0.8, 0.85, 1.0);
const FLAG: Color = Color::new(0.95, 0.3, 0.3, 1.0);
/// Width of one spike, in meters.
const SPIKE_WIDTH: f32 = 0.4;
/// How far the ground is drawn below the lowest point of the level, in meters.
const GROUND_BOTTOM: f32 = -30.0;

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Platformer")
}

struct App {
    game: Game,
    camera: WorldCamera,
    timestep: FixedTimestep,
    /// Jump presses since the last step, so a press between two steps is not lost.
    jump_pressed: bool,
}

impl App {
    fn new() -> Self {
        let game = Game::new();
        let camera = WorldCamera::new(game.level.spawn + vec2(0.0, 2.0), 12.0);
        Self { game, camera, timestep: FixedTimestep::default(), jump_pressed: false }
    }

    fn update(&mut self) {
        if is_key_pressed(KeyCode::R) || (self.game.state == State::Won && is_key_pressed(KeyCode::Enter)) {
            *self = Self::new();
            return;
        }
        let jump_keys = [KeyCode::Space, KeyCode::Up, KeyCode::W];
        self.jump_pressed |= jump_keys.iter().any(|&key| is_key_pressed(key));
        let right = is_key_down(KeyCode::Right) || is_key_down(KeyCode::D);
        let left = is_key_down(KeyCode::Left) || is_key_down(KeyCode::A);
        let run = f32::from(i8::from(right) - i8::from(left));
        let jump_held = jump_keys.iter().any(|&key| is_key_down(key));

        for _ in 0..self.timestep.advance(get_frame_time()) {
            let input = Input { run, jump_pressed: self.jump_pressed, jump_held };
            self.jump_pressed = false;
            self.game.step(input);
            let target = self.game.player.position(&self.game.world) + vec2(0.0, 1.5);
            self.camera.follow(target, self.timestep.dt());
        }
    }

    fn draw(&self) {
        clear_background(SKY);
        self.camera.apply();
        let mut renderer = DebugRenderer::new(&self.camera);
        let (game, world) = (&self.game, &self.game.world);
        let level = &game.level;

        // Solid rock beyond the walls at both ends.
        if let (Some(first), Some(last)) = (level.terrain.first(), level.terrain.last()) {
            draw_rectangle(first.x - 50.0, GROUND_BOTTOM, 50.0, 80.0, GROUND);
            draw_rectangle(last.x, GROUND_BOTTOM, 50.0, 80.0, GROUND);
        }
        // The ground, filled down to well below the screen.
        for pair in level.terrain.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            draw_triangle(a, b, vec2(b.x, GROUND_BOTTOM), GROUND);
            draw_triangle(a, vec2(b.x, GROUND_BOTTOM), vec2(a.x, GROUND_BOTTOM), GROUND);
            draw_line(a.x, a.y, b.x, b.y, 0.08, GROUND_EDGE);
        }
        // A row of spikes filling each spike sensor.
        for &spikes in &level.spikes {
            let aabb = world[spikes].aabb();
            let mut left = aabb.min.x;
            // A little tolerance, so rounding errors do not drop the last spike.
            while left + SPIKE_WIDTH <= aabb.max.x + 0.01 {
                let tip = vec2(left + SPIKE_WIDTH / 2.0, aabb.max.y);
                draw_triangle(vec2(left, aabb.min.y), vec2(left + SPIKE_WIDTH, aabb.min.y), tip, SPIKES);
                left += SPIKE_WIDTH;
            }
        }
        for &platform in &level.platforms {
            renderer.draw_collider(world, platform, PLATFORM);
        }
        for platform in &level.moving_platforms {
            renderer.draw_body(world, platform.body, MOVING_PLATFORM);
        }
        for &crate_body in &level.crates {
            renderer.draw_body(world, crate_body, CRATE);
        }
        // Coins spin: their width follows a cosine.
        for &coin in &level.coins {
            let center = world[coin].position();
            let width = COIN_RADIUS * (3.0 * game.time + center.x).cos().abs().max(0.15);
            draw_ellipse(center.x, center.y, width, COIN_RADIUS, 0.0, COIN);
        }
        // A flag on a pole at the left edge of the goal.
        let bottom = world[level.goal].aabb().min;
        let top = bottom + vec2(0.0, GOAL_HEIGHT);
        draw_line(bottom.x, bottom.y, top.x, top.y, 0.1, WHITE);
        draw_triangle(top, top - vec2(0.0, 0.8), top + vec2(1.0, -0.4), FLAG);
        // The body only, not the feet sensor.
        for &collider in world[game.player.body].colliders() {
            if !world[collider].is_sensor() {
                renderer.draw_collider(world, collider, PLAYER);
            }
        }
        let eye = game.player.position(world) + vec2(0.12, 0.25);
        renderer.draw_point(eye, WHITE);

        set_default_camera();
        self.draw_hud();
    }

    fn draw_hud(&self) {
        let game = &self.game;
        let text = format!(
            "Coins {}/{}   Time {:.1} s   Falls {}",
            game.coins_collected, game.coin_count, game.time, game.deaths
        );
        draw_text(&text, 20.0, 36.0, 32.0, WHITE);
        draw_text(
            "Arrows or A/D: run   Space/Up/W: jump (hold for higher)   R: restart",
            20.0,
            screen_height() - 20.0,
            22.0,
            WHITE,
        );
        let message = match game.state {
            State::Won => Some("You made it! Press Enter to play again".to_owned()),
            State::Playing => game
                .missing_coins_hint
                .filter(|&age| age < 2.0)
                .map(|_| format!("Collect every coin first: {} left", game.coin_count - game.coins_collected)),
        };
        if let Some(message) = message {
            draw_centered_text(&message, screen_height() / 3.0, 40, COIN);
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
