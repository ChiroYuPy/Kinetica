use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Bridge",
    category: Category::Joints,
    description: "Forty planks hinged together between two pillars, loaded with boxes and balls.",
    camera_center: vec2(0.0, 3.0),
    view_height: 14.0,
    build: Build::Static(build),
};

const PLANK_COUNT: usize = 40;
const PLANK_LENGTH: f32 = 0.5;
const HEIGHT: f32 = 5.0;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 80.0);

    let span = PLANK_LENGTH * PLANK_COUNT as f32;
    let left_end = vec2(-span / 2.0, HEIGHT);
    for side in [-1.0, 1.0] {
        let pillar = ColliderDesc::rectangle(1.0, HEIGHT);
        world.add_body(BodyDesc::fixed().position(vec2(side * (span / 2.0 + 0.5), HEIGHT / 2.0)).collider(pillar));
    }

    // Hinge points are bound to fixed bodies at both ends.
    let mut previous = world.add_body(BodyDesc::fixed().position(left_end));
    let mut hinge = left_end;
    for _ in 0..PLANK_COUNT {
        let shape = ColliderDesc::rectangle(PLANK_LENGTH, 0.16).friction(0.8);
        let plank = world.add_body(BodyDesc::dynamic().position(hinge + vec2(PLANK_LENGTH / 2.0, 0.0)).collider(shape));
        world.add_joint(RevoluteJointDesc::new(previous, plank, hinge));
        previous = plank;
        hinge += vec2(PLANK_LENGTH, 0.0);
    }
    let right_anchor = world.add_body(BodyDesc::fixed().position(hinge));
    world.add_joint(RevoluteJointDesc::new(previous, right_anchor, hinge));
    labels.at(
        vec2(0.0, HEIGHT + 4.3),
        "40 planks, each pinned to the next by a revolute joint,\nand to a fixed body at both ends",
    );

    // Load.
    for i in 0..6 {
        let position = vec2(-6.0 + 2.4 * i as f32, HEIGHT + 2.0 + (i % 2) as f32);
        let shape = if i % 2 == 0 { ColliderDesc::square(0.8) } else { ColliderDesc::circle(0.4) };
        world.add_body(BodyDesc::dynamic().position(position).collider(shape.density(2.0)));
    }
}
