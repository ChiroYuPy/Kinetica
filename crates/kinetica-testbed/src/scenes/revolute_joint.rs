use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground, add_pin};

pub const SCENE: Scene = Scene {
    name: "Revolute joint",
    category: Category::Joints,
    description: "A revolute joint pins two bodies together at a point, like a hinge or an axle: a double \
                  pendulum, and a chain of links.",
    camera_center: vec2(0.0, 4.0),
    view_height: 11.0,
    build: Build::Static(build),
};

const TOP: f32 = 7.5;

fn build(world: &mut World, _labels: &mut Labels) {
    add_ground(world, 40.0);
    // Both released horizontally.
    add_links(world, vec2(-6.0, TOP), 2, 1.6);
    add_links(world, vec2(1.0, TOP), 12, 0.5);
}

/// `count` links hinged end to end, going right from a pin at `start`.
fn add_links(world: &mut World, start: Vec2, count: usize, length: f32) {
    let mut previous = add_pin(world, start);
    for i in 0..count {
        let hinge = start + vec2(length * i as f32, 0.0);
        let shape = ColliderDesc::capsule(vec2(-length / 2.0, 0.0), vec2(length / 2.0, 0.0), 0.1);
        let link = world.add_body(BodyDesc::dynamic().position(hinge + vec2(length / 2.0, 0.0)).collider(shape));
        world.add_joint(RevoluteJointDesc::new(previous, link, hinge));
        previous = link;
    }
}
