use macroquad::prelude::*;

mod renderer;
mod scenes;

use renderer::render;
use scenes::Scene;

#[macroquad::main("Kinetica Physics Engine")]
async fn main() {
    let mut current_scene: usize = 0;
    let mut world = kinetica::core::World::new();

    let scenes: Vec<Box<dyn Scene>> = vec![
        Box::new(scenes::scenes::FallingBalls),
        Box::new(scenes::scenes::BoxDrop),
        Box::new(scenes::scenes::Pyramid),
    ];

    scenes[0].setup(&mut world);

    loop {
        clear_background(Color::new(0.125, 0.125, 0.125, 1.0));

        // Check for numeric key input
        if let Some(c) = get_char_pressed() {
            match c {
                '0' | '1' | '2' => {
                    let new_scene = c.to_digit(10).unwrap() as usize;
                    if new_scene < scenes.len() {
                        current_scene = new_scene;
                        world = kinetica::core::World::new();
                        scenes[current_scene].setup(&mut world);
                    }
                }
                _ => {}
            }
        }

        world.step(0.016);
        render(&world);

        draw_text(&format!("Scene {}: {}", current_scene, scenes[current_scene].name()), 10.0, 10.0, 20.0, WHITE);
        draw_text("Press 0, 1, 2 to switch scenes", 10.0, 40.0, 16.0, GRAY);

        next_frame().await
    }
}
