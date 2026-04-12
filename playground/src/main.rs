use kinetica::core::{RigidBody, World};
use kinetica::forces::LinearGravity;
use kinetica::math::Vec2;
use kinetica::shape::{CircleShape, Shape};
use macroquad::prelude::*;

#[macroquad::main("Kinetica Physics Engine")]
async fn main() {
    let mut world = World::new();
    world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 9.81) }));

    world.add_body(RigidBody {
        position: Vec2::new(100.0, 100.0),
        velocity: Vec2::new(50.0, 0.0),
        force: Vec2::ZERO,
        mass: 1.0,
        inv_mass: 1.0,
        shape: Shape::Circle(CircleShape { radius: 10.0 }),
    });

    world.add_body(RigidBody {
        position: Vec2::new(200.0, 50.0),
        velocity: Vec2::ZERO,
        force: Vec2::ZERO,
        mass: 2.0,
        inv_mass: 0.5,
        shape: Shape::Circle(CircleShape { radius: 10.0 }),
    });

    loop {
        clear_background(Color::new(0.125, 0.125, 0.125, 1.0));

        world.step(0.016);

        for body in &world.bodies {
            draw_circle(body.position.x, body.position.y, 10.0, WHITE);
        }

        next_frame().await
    }
}