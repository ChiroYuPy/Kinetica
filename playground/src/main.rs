use macroquad::prelude::*;
use kinetica::World;

mod mouse_grab;
mod renderer;
mod scenes;

use mouse_grab::MouseGrab;
use renderer::render;
use scenes::{Scene, BallShower, BouncingBalls, MixedShower, Pyramid, Stairs, ZeroGravity};

const WINDOW_WIDTH: f32 = 800.0;
const WINDOW_HEIGHT: f32 = 600.0;

#[macroquad::main("Kinetica Physics Engine")]
async fn main() {
    request_new_screen_size(WINDOW_WIDTH, WINDOW_HEIGHT);

    let mut current_scene: usize = 0;
    let mut world = World::new();
    let mut mouse_grab = MouseGrab::new();

    let mut scenes: Vec<Box<dyn Scene>> = vec![
        Box::new(Pyramid),
        Box::new(BallShower::new()),
        Box::new(BouncingBalls),
        Box::new(Stairs),
        Box::new(ZeroGravity),
        Box::new(MixedShower::new()),
    ];

    scenes[0].setup(&mut world, WINDOW_WIDTH, WINDOW_HEIGHT);

    let mut last_time = get_time() as f32;

    loop {
        let now = get_time() as f32;
        let dt = now - last_time;
        last_time = now;

        clear_background(Color::new(0.125, 0.125, 0.125, 1.0));

        // Scene switching
        if let Some(c) = get_char_pressed() {
            match c {
                '0' | '1' | '2' | '3' | '4' | '5' | '6' => {
                    let new_scene = c.to_digit(10).unwrap() as usize;
                    if new_scene < scenes.len() {
                        current_scene = new_scene;
                        world = World::new();
                        scenes[current_scene].setup(&mut world, WINDOW_WIDTH, WINDOW_HEIGHT);
                    }
                }
                _ => {}
            }
        }

        // Update
        scenes[current_scene].update(&mut world, dt);
        mouse_grab.update(&mut world);
        world.step(0.016);

        // Render
        render(&world);
        mouse_grab.draw(&world);

        // UI
        draw_text(
            &format!("Scene {}: {}", current_scene, scenes[current_scene].name()),
            10.0,
            10.0,
            20.0,
            WHITE,
        );
        draw_text("Press 0-6 to switch scenes", 10.0, 40.0, 16.0, GRAY);
        draw_text("Click & drag to grab objects", 10.0, 60.0, 16.0, GRAY);

        next_frame().await
    }
}
