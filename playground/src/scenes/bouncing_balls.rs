use crate::scenes::Scene;
use kinetica::collisions::CollisionDetector;
use kinetica::forces::LinearGravity;
use kinetica::math::Vec2;
use kinetica::{RigidBody, Shape, World};

pub struct BouncingBalls;

impl Scene for BouncingBalls {
    fn name(&self) -> &str {
        "3. Bouncing balls"
    }

    fn setup(&self, world: &mut World, width: f32, height: f32) {
        world.default_restitution = 1.0; // Perfect bounce
        world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 400.0) }));
        world.collision_detector = Some(CollisionDetector::new());

        add_bounds(world, width, height);

        // Spawn bouncing balls
        for i in 0..15 {
            let x = 200.0 + i as f32 * 30.0;
            let mut body = RigidBody::new(
                Vec2::new(x, 100.0),
                1.0,
                Shape::Circle(12.0),
            );
            body.motion.linear_velocity = Vec2::new(0.0, 200.0);
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
}

fn add_wall(world: &mut World, x: f32, y: f32, width: f32, height: f32) {
    world.add_body(RigidBody::static_body(
        Vec2::new(x, y),
        Shape::Rectangle(Vec2::new(width, height)),
    ));
}
