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
const PLANK_HALF_LENGTH: f32 = 0.25;
const HEIGHT: f32 = 5.0;

fn build(world: &mut World, _labels: &mut Labels) {
    add_ground(world, 40.0);

    let span = 2.0 * PLANK_HALF_LENGTH * PLANK_COUNT as f32;
    let left_end = vec2(-span / 2.0, HEIGHT);
    for side in [-1.0, 1.0] {
        let pillar = world.add_body(BodyDesc::fixed().position(vec2(side * (span / 2.0 + 0.5), HEIGHT / 2.0)));
        world.add_collider(pillar, ColliderDesc::rectangle(0.5, HEIGHT / 2.0));
    }

    // Hinge points are bound to fixed bodies at both ends.
    let mut previous = world.add_body(BodyDesc::fixed().position(left_end));
    let mut hinge = left_end;
    for _ in 0..PLANK_COUNT {
        let plank = world.add_body(BodyDesc::dynamic().position(hinge + vec2(PLANK_HALF_LENGTH, 0.0)));
        world.add_collider(plank, ColliderDesc::rectangle(PLANK_HALF_LENGTH, 0.08).friction(0.8));
        world.add_joint(RevoluteJointDesc::at_world_point(world, previous, plank, hinge));
        previous = plank;
        hinge += vec2(2.0 * PLANK_HALF_LENGTH, 0.0);
    }
    let right_anchor = world.add_body(BodyDesc::fixed().position(hinge));
    world.add_joint(RevoluteJointDesc::at_world_point(world, previous, right_anchor, hinge));

    // Load.
    for i in 0..6 {
        let position = vec2(-6.0 + 2.4 * i as f32, HEIGHT + 2.0 + (i % 2) as f32);
        let load = world.add_body(BodyDesc::dynamic().position(position));
        let collider = if i % 2 == 0 { ColliderDesc::rectangle(0.4, 0.4) } else { ColliderDesc::circle(0.4) };
        world.add_collider(load, collider.density(2.0));
    }
}
