use std::f32::consts::TAU;

use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Body types",
    category: Category::Bodies,
    description: "Fixed bodies never move, kinematic ones follow the velocity they are given, dynamic ones are \
                  simulated. Grab one!",
    camera_center: vec2(0.0, 3.5),
    view_height: 11.0,
    build: Build::Animated(build),
};

/// The elevator rises twice this far above its start, in meters.
const HALF_TRAVEL: f32 = 1.5;
/// Seconds per round trip.
const ELEVATOR_PERIOD: f32 = 5.0;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 20.0);
    labels.at(vec2(0.0, -1.4), "Fixed");

    let elevator =
        world.add_body(BodyDesc::kinematic().position(vec2(0.0, 0.7)).collider(ColliderDesc::rectangle(4.0, 0.4)));
    labels.at(vec2(-4.0, 2.0), "Kinematic");

    let mut top = None;
    for position in [vec2(-1.0, 1.4), vec2(0.0, 1.4), vec2(1.0, 1.4), vec2(-0.5, 2.4), vec2(0.5, 2.4), vec2(0.0, 3.4)] {
        top = Some(world.add_body(BodyDesc::dynamic().position(position).collider(ColliderDesc::square(1.0))));
    }
    labels.above(top.expect("there are boxes"), "Dynamic");

    // The scene sets the elevator's velocity; its position follows, whatever it carries.
    Box::new(move |world: &mut World, frame: &Frame| {
        let angular_frequency = TAU / ELEVATOR_PERIOD;
        let speed = HALF_TRAVEL * angular_frequency * (angular_frequency * frame.time).sin();
        world[elevator].set_linear_velocity(vec2(0.0, speed));
    })
}
