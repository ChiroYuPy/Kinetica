use crate::scenes::Scene;
use kinetica::collisions::NaiveCollisionDetector;
use kinetica::core::{RigidBody, Shape, World};
use kinetica::forces::LinearGravity;
use kinetica::math::Vec2;

pub struct Pyramid;

impl Scene for Pyramid {
    fn name(&self) -> &str {
        "1. Pyramid"
    }

    fn setup(&self, world: &mut World, width: f32, height: f32) {
        world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 200.0) }));
        world.collision_detector = Some(Box::new(NaiveCollisionDetector::new()));

        add_bounds(world, width, height);

        let size = 10;
        let ball_radius = 10.0;
        let spacing = ball_radius * 2.0 + 2.0;
        let start_y = 150.0;

        for row in 0..size {
            let balls_in_row = size - row;
            let row_width = balls_in_row as f32 * spacing;
            let start_x = width / 2.0 - row_width / 2.0 + spacing / 2.0;

            for col in 0..balls_in_row {
                world.add_body(RigidBody::new(
                    Vec2::new(start_x + col as f32 * spacing, start_y - row as f32 * spacing * 0.86),
                    1.0,
                    Shape::Circle(ball_radius),
                ));
            }
        }
    }

    fn update(&mut self, _world: &mut World, _dt: f32) {}
}

fn add_bounds(world: &mut World, width: f32, height: f32) {
    let margin = 40.0;
    let wall_thick = 20.0;

    // Floor
    add_wall(world, width / 2.0, height - margin - wall_thick / 2.0, width - margin * 2.0, wall_thick);
    // Left wall
    add_wall(world, margin + wall_thick / 2.0, height / 2.0, wall_thick, height - margin * 2.0);
    // Right wall
    add_wall(world, width - margin - wall_thick / 2.0, height / 2.0, wall_thick, height - margin * 2.0);
}

fn add_wall(world: &mut World, x: f32, y: f32, width: f32, height: f32) {
    world.add_body(RigidBody::static_body(
        Vec2::new(x, y),
        Shape::Rectangle(Vec2::new(width, height)),
    ));
}
