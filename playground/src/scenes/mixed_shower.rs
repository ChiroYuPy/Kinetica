use crate::scenes::Scene;
use kinetica::collisions::NaiveCollisionDetector;
use kinetica::core::{RigidBody, Shape, World};
use kinetica::forces::LinearGravity;
use kinetica::math::Vec2;
use macroquad::rand;

pub struct MixedShower {
    timer: f32,
}

impl MixedShower {
    pub fn new() -> Self {
        Self { timer: 0.0 }
    }
}

impl Scene for MixedShower {
    fn name(&self) -> &str {
        "6. Mixed shower"
    }

    fn setup(&self, world: &mut World, width: f32, height: f32) {
        world.add_force_generator(Box::new(LinearGravity { acceleration: Vec2::new(0.0, 400.0) }));
        world.collision_detector = Some(Box::new(NaiveCollisionDetector::new()));

        add_bounds(world, width, height);
    }

    fn update(&mut self, world: &mut World, dt: f32) {
        self.timer += dt;

        if self.timer >= 0.15 && world.bodies.len() < 200 {
            self.timer = 0.0;

            let angle = (rand::gen_range(-60.0, 60.0) as f32).to_radians();
            let speed = rand::gen_range(50.0, 150.0);
            let velocity = Vec2::new(angle.sin() * speed, angle.cos() * speed);

            let x = rand::gen_range(150.0, 650.0);

            let body = if rand::gen_range(0, 2) == 0 {
                // Circle
                let radius = rand::gen_range(8.0, 20.0);
                let mut b = RigidBody::new(Vec2::new(x, 50.0), radius * radius, Shape::Circle(radius));
                b.state.velocity = velocity;
                b
            } else {
                // Rectangle
                let w = rand::gen_range(15.0, 40.0);
                let h = rand::gen_range(15.0, 40.0);
                let mut b = RigidBody::new(Vec2::new(x, 50.0), w * h / 100.0, Shape::Rectangle(Vec2::new(w, h)));
                b.state.velocity = velocity;
                b
            };

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
