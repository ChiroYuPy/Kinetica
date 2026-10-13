use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Center of mass",
    category: Category::Bodies,
    description: "A body may have several colliders. It turns around their common center of mass, the cross: the \
                  hammer's is near its head.",
    camera_center: vec2(0.0, 3.5),
    view_height: 11.0,
    build: Build::Animated(build),
};

const HAMMER_START: Vec2 = vec2(-3.5, 1.0);
/// The hammer is thrown again this often, in seconds.
const THROW_PERIOD: f32 = 4.0;
const CENTER_COLOR: Color = Color::new(1.0, 0.9, 0.3, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);

    // A hammer: one body, a light handle and a heavy head.
    let handle = ColliderDesc::rectangle(1.6, 0.16);
    let head = ColliderDesc::rectangle(0.3, 0.7).offset(vec2(0.8, 0.0)).density(6.0);
    let hammer = world.add_body(BodyDesc::dynamic().position(HAMMER_START).collider(handle).collider(head));

    // Round-bottomed toys: where the mass sits decides whether they stand up.
    add_toy(world, labels, vec2(1.5, 0.6), vec2(0.0, -0.3), "Heavy bottom");
    add_toy(world, labels, vec2(5.0, 0.6), vec2(0.0, 1.3), "Heavy top");

    Box::new(Hammer { hammer, throws: 0 })
}

/// A ball with a post on top, tilted, with a small heavy weight at `weight_center`.
fn add_toy(world: &mut World, labels: &mut Labels, position: Vec2, weight_center: Vec2, text: &str) {
    // Colliders of a body may overlap: their masses simply add up.
    world.add_body(
        BodyDesc::dynamic()
            .position(position)
            .angle(0.9)
            .angular_damping(1.5)
            .collider(ColliderDesc::circle(0.6).density(0.5))
            .collider(ColliderDesc::rectangle(0.4, 1.2).offset(vec2(0.0, 0.9)).density(0.5))
            .collider(ColliderDesc::circle(0.25).offset(weight_center).density(20.0)),
    );
    labels.at(position + vec2(0.0, 2.6), text);
}

struct Hammer {
    hammer: BodyHandle,
    throws: usize,
}

impl SceneUpdate for Hammer {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if frame.time >= THROW_PERIOD * self.throws as f32 {
            self.throws += 1;
            // Sent up from its starting point, spinning.
            let hammer = &mut world[self.hammer];
            hammer.set_position(HAMMER_START);
            hammer.set_angle(0.0);
            hammer.set_linear_velocity(vec2(0.0, 9.0));
            hammer.set_angular_velocity(7.0);
        }
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            let center = body.center_of_mass();
            let size = 0.15;
            drawer.draw_segment(center - vec2(size, 0.0), center + vec2(size, 0.0), CENTER_COLOR);
            drawer.draw_segment(center - vec2(0.0, size), center + vec2(0.0, size), CENTER_COLOR);
        }
    }
}
