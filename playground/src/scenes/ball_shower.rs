use crate::scenes::Scene;
use kinetica::collisions::CollisionDetector;
use kinetica::forces::LinearGravity;
use kinetica::math::Vec2;
use macroquad::rand;
use kinetica::{RigidBody, Shape, World};

pub struct BallShower {
    timer: f32,
}

impl BallShower {
    pub fn new() -> Self {
        Self { timer: 0.0 }
    }
}

impl Scene for BallShower {
    fn name(&self) -> &str {
        "2. Ball shower"
    }

    fn setup(&self, world: &mut World, width: f32, height: f32) {
        world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 300.0) }));
        world.collision_detector = Some(CollisionDetector::new());

        add_bounds(world, width, height);
    }

    fn update(&mut self, world: &mut World, dt: f32) {
        self.timer += dt;

        if self.timer >= 0.25 && world.bodies.len() < 128 {
            self.timer = 0.0;

            let angle = (rand::gen_range(-45.0, 45.0) as f32).to_radians();
            let speed = 100.0;
            let velocity = Vec2::new(angle.sin() * speed, angle.cos() * speed);

            let mut body = RigidBody::new(
                Vec2::new(400.0, 100.0),
                1.0,
                Shape::Circle(10.0),
            );
            body.state.velocity = velocity;
            world.add_body(body);
        }
    }
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
