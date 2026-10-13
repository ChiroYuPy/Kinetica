use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Forces",
    category: Category::Bodies,
    description: "A force pushes for one step and must be applied again every step. Up fires the rocket's engine, \
                  Left and Right apply a torque.",
    camera_center: vec2(0.0, 6.0),
    view_height: 16.0,
    build: Build::Animated(build),
};

const ROCKET_START: Vec2 = vec2(0.0, 1.2);
/// Engine thrust, in multiples of the rocket's weight.
const THRUST_TO_WEIGHT: f32 = 1.8;
/// Steering angular acceleration, in rad/s².
const STEERING: f32 = 12.0;
const FLAME_COLOR: Color = Color::new(1.0, 0.6, 0.2, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let ground = add_ground(world, 30.0);
    for x in [-14.75, 14.75] {
        world.add_collider(ground, ColliderDesc::rectangle(0.5, 30.0).offset(vec2(x, 15.5)));
    }
    world.add_collider(ground, ColliderDesc::rectangle(2.4, 0.3).offset(ROCKET_START + vec2(0.0, -0.55)));

    let nose = ColliderDesc::convex_hull(&[vec2(-0.3, 0.8), vec2(0.3, 0.8), vec2(0.0, 1.4)]).expect("a triangle");
    let fin = |side: f32| {
        let corners = [vec2(side * 0.3, -0.8), vec2(side * 0.3, -0.2), vec2(side * 0.7, -0.9)];
        ColliderDesc::convex_hull(&corners).expect("a triangle")
    };
    let rocket = world.add_body(
        BodyDesc::dynamic()
            .position(ROCKET_START)
            .angular_damping(2.0)
            .collider(ColliderDesc::rectangle(0.6, 1.6))
            .collider(nose)
            .collider(fin(-1.0))
            .collider(fin(1.0)),
    );
    labels.at(ROCKET_START + vec2(0.0, -2.6), "Up: force     Left / Right: torque");
    Box::new(Rocket { rocket, thrusting: false })
}

struct Rocket {
    rocket: BodyHandle,
    thrusting: bool,
}

impl SceneUpdate for Rocket {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        let gravity = world.gravity().length();
        let rocket = &mut world[self.rocket];
        self.thrusting = frame.is_down(KeyCode::Up);
        if self.thrusting {
            // Applied at the nozzle, along the rocket.
            let thrust = rocket.world_vector(Vec2::Y) * THRUST_TO_WEIGHT * rocket.mass() * gravity;
            rocket.apply_force_at_point(thrust, rocket.world_point(vec2(0.0, -0.8)));
        }
        let steering = frame.axis(KeyCode::Right, KeyCode::Left);
        if steering != 0.0 {
            rocket.apply_torque(steering * STEERING * rocket.inertia());
        }
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        if self.thrusting {
            let rocket = &world[self.rocket];
            let flicker = 0.8 + 0.2 * (frame.time * 40.0).sin();
            let flame = rocket.world_point(vec2(0.0, -0.8 - flicker));
            for side in [-0.15, 0.0, 0.15] {
                drawer.draw_segment(rocket.world_point(vec2(side, -0.8)), flame, FLAME_COLOR);
            }
        }
    }
}
