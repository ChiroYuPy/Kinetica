use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Weld joint",
    category: Category::Joints,
    description: "A weld joint glues two bodies together. Each beam is seven bars welded end to end; with an \
                  angular spring, the welds bend.",
    camera_center: vec2(0.0, 4.0),
    view_height: 10.0,
    build: Build::Static(build),
};

const SEGMENT_COUNT: usize = 7;
const SEGMENT_LENGTH: f32 = 1.0;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);
    let wall = world.add_body(BodyDesc::fixed().position(vec2(-4.0, 4.0)).collider(ColliderDesc::rectangle(0.5, 8.0)));
    add_beam(world, wall, vec2(-3.75, 6.5), None);
    labels.at(vec2(0.0, 7.0), "Rigid welds");
    add_beam(world, wall, vec2(-3.75, 3.0), Some((20.0, 0.5)));
    labels.at(vec2(0.0, 3.5), "Welds with an angular spring");
}

/// A horizontal beam welded to `wall` at `start`, rigid or with angular springs.
fn add_beam(world: &mut World, wall: BodyHandle, start: Vec2, spring: Option<(f32, f32)>) {
    let mut previous = wall;
    let mut weld_point = start;
    for _ in 0..SEGMENT_COUNT {
        let bar = ColliderDesc::rectangle(SEGMENT_LENGTH, 0.24);
        let segment =
            world.add_body(BodyDesc::dynamic().position(weld_point + vec2(SEGMENT_LENGTH / 2.0, 0.0)).collider(bar));
        let mut weld = WeldJointDesc::new(previous, segment, weld_point);
        if let Some((hertz, damping_ratio)) = spring {
            weld = weld.angular_spring(hertz, damping_ratio);
        }
        world.add_joint(weld);
        previous = segment;
        weld_point += vec2(SEGMENT_LENGTH, 0.0);
    }
}
