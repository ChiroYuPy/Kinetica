use kinetica::prelude::*;

use super::{Build, Category, Scene};

pub const SCENE: Scene = Scene {
    name: "Cantilever",
    category: Category::Joints,
    description: "Eight welded segments: rigid welds on top (joints are slightly soft, so it sags a little), \
                  25 Hz angular springs below.",
    camera_center: vec2(0.0, 2.0),
    view_height: 12.0,
    build: Build::Static(build),
};

const SEGMENT_COUNT: usize = 8;
const SEGMENT_HALF_LENGTH: f32 = 0.5;

fn build(world: &mut World) {
    let wall = world.add_body(BodyDesc::fixed().position(vec2(-5.0, 2.0)));
    world.add_collider(wall, ColliderDesc::rectangle(0.25, 4.0));

    add_beam(world, wall, vec2(-4.75, 5.0), None);
    add_beam(world, wall, vec2(-4.75, 1.0), Some((25.0, 0.7)));
}

/// A horizontal beam welded to `wall` at `start`, rigid or with angular springs.
fn add_beam(world: &mut World, wall: BodyHandle, start: Vec2, spring: Option<(f32, f32)>) {
    let mut previous = wall;
    let mut weld_point = start;
    for _ in 0..SEGMENT_COUNT {
        let segment = world.add_body(BodyDesc::dynamic().position(weld_point + vec2(SEGMENT_HALF_LENGTH, 0.0)));
        world.add_collider(segment, ColliderDesc::rectangle(SEGMENT_HALF_LENGTH, 0.12));
        let mut weld = WeldJointDesc::at_world_point(world, previous, segment, weld_point);
        if let Some((hertz, damping_ratio)) = spring {
            weld = weld.angular_spring(hertz, damping_ratio);
        }
        world.add_joint(weld);
        previous = segment;
        weld_point += vec2(2.0 * SEGMENT_HALF_LENGTH, 0.0);
    }
}
