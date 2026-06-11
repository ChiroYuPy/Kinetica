use crate::scenes::Scene;
use kinetica::collisions::CollisionDetector;
use kinetica::math::Vec2;
use kinetica::{RigidBody, Shape, World};

pub struct ZeroGravity;

impl Scene for ZeroGravity {
    fn name(&self) -> &str {
        "5. Zero gravity"
    }

    fn setup(&self, world: &mut World, width: f32, height: f32) {
        world.collision_detector = Some(CollisionDetector::new());

        add_bounds(world, width, height);

        // Spawn balls with random velocities
        for i in 0..30 {
            let angle = (i as f32 * 12.0).to_radians();
            let speed = 150.0;

            let mut body = RigidBody::new(
                Vec2::new(width / 2.0, height / 2.0),
                1.0,
                Shape::Circle(10.0),
            );
            body.motion.linear_velocity = Vec2::new(angle.cos() * speed, angle.sin() * speed);
            world.add_body(body);
        }
    }

    fn update(&mut self, _world: &mut World, _dt: f32) {}
}

fn add_bounds(world: &mut World, width: f32, height: f32) {
    let margin = 40.0;
    let wall_thick = 20.0;

    add_wall(world, width / 2.0, height - margin - wall_thick / 2.0, width - margin * 2.0, wall_thick);
    add_wall(world, margin + wall_thick / 2.0, height / 2.0, wall_thick, height - margin * 2.0);
    add_wall(world, width - margin - wall_thick / 2.0, height / 2.0, wall_thick, height - margin * 2.0);
    add_wall(world, width / 2.0, margin + wall_thick / 2.0, width - margin * 2.0, wall_thick);
}

fn add_wall(world: &mut World, x: f32, y: f32, width: f32, height: f32) {
    world.add_body(RigidBody::static_body(
        Vec2::new(x, y),
        Shape::Rectangle(Vec2::new(width, height)),
    ));
}
