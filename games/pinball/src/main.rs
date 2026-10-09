//! Pinball: launch the ball with the plunger, keep it in play with the flippers, and score on the
//! bumpers, slingshots and rollover lanes.

mod game;
mod table;

use kinetica::shape::Shape;
use kinetica_macroquad::{DebugRenderer, FixedTimestep, WorldCamera, draw_centered_text};
use macroquad::prelude::*;

use crate::game::{BALLS_PER_GAME, FLASH_DURATION, Game, Input, State};
use crate::table::ROLLOVER_RADIUS;

const BACKGROUND: Color = Color::new(0.05, 0.06, 0.12, 1.0);
const PLAYFIELD: Color = Color::new(0.1, 0.14, 0.3, 1.0);
const WALL: Color = Color::new(0.75, 0.78, 0.85, 1.0);
const FLIPPER: Color = Color::new(0.95, 0.95, 0.95, 1.0);
const PLUNGER: Color = Color::new(0.85, 0.3, 0.25, 1.0);
const BALL: Color = Color::new(0.85, 0.87, 0.92, 1.0);
const KICKER: Color = Color::new(0.95, 0.45, 0.2, 1.0);
const KICKER_LIT: Color = Color::new(1.0, 0.95, 0.5, 1.0);
const ROLLOVER: Color = Color::new(0.3, 0.3, 0.4, 1.0);
const ROLLOVER_LIT: Color = Color::new(0.4, 0.95, 0.5, 1.0);
const MESSAGE: Color = Color::new(1.0, 0.9, 0.5, 1.0);
/// Wall thickness on screen, in meters.
const WALL_WIDTH: f32 = 0.12;

fn window_conf() -> Conf {
    kinetica_macroquad::window_conf("Pinball")
}

struct App {
    game: Game,
    camera: WorldCamera,
    timestep: FixedTimestep,
}

impl App {
    fn new() -> Self {
        Self { game: Game::new(), camera: WorldCamera::new(vec2(0.4, 10.2), 22.0), timestep: FixedTimestep::default() }
    }

    fn update(&mut self) {
        if is_key_pressed(KeyCode::R) || (self.game.state == State::GameOver && is_key_pressed(KeyCode::Enter)) {
            *self = Self::new();
            return;
        }
        let any_down = |keys: &[KeyCode]| keys.iter().any(|&key| is_key_down(key));
        let input = Input {
            left_flipper: any_down(&[KeyCode::Left, KeyCode::LeftShift, KeyCode::A]),
            right_flipper: any_down(&[KeyCode::Right, KeyCode::RightShift, KeyCode::D]),
            plunger: any_down(&[KeyCode::Space, KeyCode::Down, KeyCode::S]),
        };
        for _ in 0..self.timestep.advance(get_frame_time()) {
            self.game.step(input);
        }
    }

    fn draw(&self) {
        clear_background(BACKGROUND);
        self.camera.apply();
        let mut renderer = DebugRenderer::new(&self.camera);
        let (game, world) = (&self.game, &self.game.world);
        let table = &game.table;

        for piece in table::floor() {
            for pair in piece[1..].windows(2) {
                draw_triangle(piece[0], pair[0], pair[1], PLAYFIELD);
            }
        }
        for pair in table.outline.windows(2) {
            draw_line(pair[0].x, pair[0].y, pair[1].x, pair[1].y, WALL_WIDTH, WALL);
        }
        for &wall in &table.walls {
            renderer.draw_collider(world, wall, WALL);
        }

        for (&rollover, &lit) in table.rollovers.iter().zip(&game.rollovers_lit) {
            let center = world[rollover].aabb().center();
            draw_circle(center.x, center.y, ROLLOVER_RADIUS, if lit { ROLLOVER_LIT } else { ROLLOVER });
        }
        // Bumpers are discs, slingshots have a rubber band; both light up when hit.
        for kicker in &table.kickers {
            let lit = game.flashes.iter().any(|flash| flash.collider == kicker.collider && flash.age < FLASH_DURATION);
            let color = if lit { KICKER_LIT } else { KICKER };
            let collider = &world[kicker.collider];
            match collider.shape() {
                Shape::Circle(circle) => {
                    let center = collider.aabb().center();
                    draw_circle(center.x, center.y, circle.radius(), color);
                    draw_circle_lines(center.x, center.y, 0.7 * circle.radius(), 0.08, BACKGROUND);
                }
                Shape::Segment(segment) => {
                    let (start, end) = (segment.start(), segment.end());
                    draw_line(start.x, start.y, end.x, end.y, 0.12, color);
                }
                _ => renderer.draw_collider(world, kicker.collider, color),
            }
        }

        for flipper in &table.flippers {
            renderer.draw_body(world, flipper.body, FLIPPER);
        }
        renderer.draw_body(world, table.plunger.body, PLUNGER);
        if let Some(ball) = game.ball {
            renderer.draw_body(world, ball, BALL);
        }

        set_default_camera();
        self.draw_hud();
    }

    fn draw_hud(&self) {
        let game = &self.game;
        draw_text(format!("Score {}", game.score), 20.0, 40.0, 36.0, WHITE);
        let ball = format!("Ball {}/{BALLS_PER_GAME}", game.ball_number);
        draw_text(&ball, 20.0, 72.0, 26.0, WHITE);
        let tension = game.table.plunger.tension(&game.world);
        if tension > 0.0 {
            draw_text(format!("Plunger {:.0}%", 100.0 * tension), 20.0, 100.0, 26.0, PLUNGER);
        }
        let help = "Left/Right (or A/D, Shift): flippers   Hold Space: pull the plunger   R: restart";
        draw_text(help, 20.0, screen_height() - 20.0, 22.0, WHITE);
        if game.state == State::GameOver {
            draw_centered_text("Game over", screen_height() * 0.4, 56, MESSAGE);
            draw_centered_text("Press Enter to play again", screen_height() * 0.47, 28, MESSAGE);
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
