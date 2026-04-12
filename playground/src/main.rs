use kinetica::collisions::NaiveCollisionDetector;
use kinetica::core::{RigidBody, Shape, World};
use kinetica::forces::LinearGravity;
use kinetica::math::Vec2;
use macroquad::prelude::*;

#[macroquad::main("Kinetica Physics Engine")]
async fn main() {
    let mut world = World::new();
    world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 5.0) }));
    world.collision_detector = Some(Box::new(NaiveCollisionDetector::new()));

    let radius = 10.0;
    let spacing = radius * 2.0 + 2.0;

    let cols = 15;
    let rows = 10;

    let grid_width = (cols - 1) as f32 * spacing;
    let grid_height = (rows - 1) as f32 * spacing;

    let screen_width = screen_width();
    let screen_height = screen_height();
    let offset_x = (screen_width - grid_width) / 2.0;
    let offset_y = (screen_height - grid_height) / 2.0;

    for i in 0..cols {
        for j in 0..rows {
            let x = offset_x + i as f32 * spacing;
            let y = offset_y + j as f32 * spacing;

            world.add_body(RigidBody::new(
                Vec2::new(x, y),
                1.0,
                Shape::Circle(radius),
            ));
        }
    }

    let border_count_x = (screen_width / spacing).ceil() as usize;
    let border_count_y = (screen_height / spacing).ceil() as usize;

    for i in 0..border_count_x {
        let x = i as f32 * spacing;
        world.add_body(RigidBody::static_body(
            Vec2::new(x, radius),
            Shape::Circle(radius),
        ));
        world.add_body(RigidBody::static_body(
            Vec2::new(x, screen_height - radius),
            Shape::Circle(radius),
        ));
    }

    for j in 1..border_count_y - 1 {
        let y = j as f32 * spacing;
        world.add_body(RigidBody::static_body(
            Vec2::new(radius, y),
            Shape::Circle(radius),
        ));
        world.add_body(RigidBody::static_body(
            Vec2::new(screen_width - radius, y),
            Shape::Circle(radius),
        ));
    }

    loop {
        clear_background(Color::new(0.125, 0.125, 0.125, 1.0));

        world.step(0.016);

        for body in &world.bodies {
            let color = if body.is_static() {
                DARKGRAY
            } else {
                WHITE
            };
            draw_circle(body.position().x, body.position().y, radius, color);
        }

        draw_text(&format!("Bodies: {}", world.len()), 10.0, 10.0, 20.0, WHITE);

        next_frame().await
    }
}
