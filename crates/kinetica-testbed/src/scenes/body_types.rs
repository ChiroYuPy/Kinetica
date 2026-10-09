use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Body types",
    category: Category::Bodies,
    description: "Every body is fixed, kinematic or dynamic. Fixed bodies never move. Kinematic bodies move at \
                  the velocity you give them and push everything out of their way, but nothing pushes them back. \
                  Dynamic bodies are the simulated ones: gravity, contacts and joints move them. Grab one!",
    camera_center: vec2(0.0, 4.5),
    view_height: 14.0,
    build: Build::Animated(build),
};

/// The elevator rises twice this far above its start, in meters.
const HALF_TRAVEL: f32 = 1.5;
/// Seconds per round trip.
const ELEVATOR_PERIOD: f32 = 5.0;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let ground = add_ground(world, 10.0);
    for x in [-9.75, 9.75] {
        world.add_collider(ground, ColliderDesc::offset_rectangle(0.25, 4.0, vec2(x, 4.5), 0.0));
    }
    labels.at(vec2(0.0, -1.4), "Fixed: the ground");

    // Fixed: a ledge holding a few boxes.
    let ledge = world.add_body(BodyDesc::fixed().position(vec2(-6.5, 4.0)));
    world.add_collider(ledge, ColliderDesc::rectangle(2.0, 0.2));
    labels.at(vec2(-6.5, 3.0), "Fixed\nnever moves");

    // Kinematic: an elevator moved by its velocity, and a turning paddle.
    let elevator = world.add_body(BodyDesc::kinematic().position(vec2(0.0, 0.5)));
    world.add_collider(elevator, ColliderDesc::rectangle(1.5, 0.2));
    let paddle = world.add_body(BodyDesc::kinematic().position(vec2(6.0, 3.0)).angular_velocity(1.0));
    world.add_collider(paddle, ColliderDesc::rectangle(2.0, 0.15));
    labels.at(vec2(0.0, 8.5), "Kinematic\nmoves at the velocity you set,\npushes but is never pushed");
    labels.at(vec2(6.0, 6.0), "Kinematic\nturning at 1 rad/s");

    // Dynamic: boxes and balls on each of them.
    for position in [vec2(-7.5, 4.7), vec2(-6.0, 4.7), vec2(-6.0, 5.7), vec2(-0.8, 1.2), vec2(0.8, 1.2)] {
        let body = world.add_body(BodyDesc::dynamic().position(position));
        world.add_collider(body, ColliderDesc::rectangle(0.5, 0.5));
    }
    for i in 0..6 {
        let body = world.add_body(BodyDesc::dynamic().position(vec2(4.5 + 0.6 * i as f32, 6.0 + 0.7 * i as f32)));
        world.add_collider(body, ColliderDesc::circle(0.3));
    }
    let dynamic = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 2.2)));
    world.add_collider(dynamic, ColliderDesc::rectangle(0.5, 0.5));
    labels.above(dynamic, "Dynamic\nsimulated");

    // The script sets the elevator's velocity; its position follows.
    Box::new(move |world: &mut World, frame: &Frame| {
        let angular_frequency = std::f32::consts::TAU / ELEVATOR_PERIOD;
        let speed = HALF_TRAVEL * angular_frequency * (angular_frequency * frame.time).sin();
        if let Some(body) = world.body_mut(elevator) {
            body.set_linear_velocity(vec2(0.0, speed));
        }
    })
}
