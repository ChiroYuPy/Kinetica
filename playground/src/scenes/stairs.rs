use crate::scenes::Scene;
use kinetica::collisions::CollisionDetector;
use kinetica::core::{RigidBody, Shape, World};
use kinetica::forces::LinearGravity;
use kinetica::math::Vec2;

pub struct Stairs;

impl Scene for Stairs {
    fn name(&self) -> &str {
        "4. Stairs"
    }

    fn setup(&self, world: &mut World, width: f32, height: f32) {
        world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 500.0) }));
        world.collision_detector = Some(CollisionDetector::new());

        add_bounds(world, width, height);

        // Create stairs - each step is the same rectangle
        let step_width = 50.0;
        let step_height = 10.0;
        let num_steps = 10;
        let start_x = 150.0;
        let start_y = height - 100.0;

        for i in 0..num_steps {
            let x = start_x + i as f32 * step_width;
            let y = start_y - i as f32 * step_height;

            // One rectangle per step
            add_wall(world, x + step_width / 2.0, y, step_width, step_height);
        }

        // Spawn one ball above the highest step, with small impulse
        let top_step_x = start_x + (num_steps - 1) as f32 * step_width + step_width / 2.0;
        let top_step_y = start_y - (num_steps - 1) as f32 * step_height;

        let mut body = RigidBody::new(
            Vec2::new(top_step_x, top_step_y - 30.0),
            1.0,
            Shape::Circle(12.0),
        );
        body.state.velocity = Vec2::new(-30.0, 0.0); // Small impulse to the left
        world.add_body(body);
    }

    fn update(&mut self, _world: &mut World, _dt: f32) {}
}

fn add_bounds(world: &mut World, width: f32, height: f32) {
    let margin = 40.0;
    let wall_thick = 20.0;

    add_wall(world, width / 2.0, height - margin - wall_thick / 2.0, width - margin * 2.0, wall_thick);
    add_wall(world, margin + wall_thick / 2.0, height / 2.0, wall_thick, height - margin * 2.0);
    add_wall(world, width - margin - wall_thick / 2.0, height / 2.0, wall_thick, height - margin * 2.0);
}

fn add_wall(world: &mut World, x: f32, y: f32, width: f32, height: f32) {
    world.add_body(RigidBody::static_body(
        Vec2::new(x, y),
        Shape::Rectangle(Vec2::new(width, height)),
    ));
}
