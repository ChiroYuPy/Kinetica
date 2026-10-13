use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Bridge",
    category: Category::SoftStructures,
    description: "A hanging bridge is a line of planks, each hinged to the next by a revolute joint and pinned \
                  to the pillars at both ends.",
    camera_center: vec2(0.0, 3.0),
    view_height: 9.0,
    build: Build::Static(build),
};

const PLANK_COUNT: usize = 24;
const PLANK_LENGTH: f32 = 0.5;
const HEIGHT: f32 = 3.0;

fn build(world: &mut World, _labels: &mut Labels) {
    let ground = add_ground(world, 40.0);
    let left_end = vec2(-PLANK_LENGTH * PLANK_COUNT as f32 / 2.0, HEIGHT);
    let right_end = vec2(-left_end.x, HEIGHT);
    for end in [left_end, right_end] {
        world.add_collider(ground, ColliderDesc::rectangle(0.4, HEIGHT).offset(vec2(end.x, 0.5 + HEIGHT / 2.0)));
    }

    let mut previous = ground;
    for i in 0..PLANK_COUNT {
        let hinge = left_end + vec2(PLANK_LENGTH * i as f32, 0.0);
        let shape = ColliderDesc::rectangle(PLANK_LENGTH, 0.16).friction(0.8);
        let plank = world.add_body(BodyDesc::dynamic().position(hinge + vec2(PLANK_LENGTH / 2.0, 0.0)).collider(shape));
        world.add_joint(RevoluteJointDesc::new(previous, plank, hinge));
        previous = plank;
    }
    world.add_joint(RevoluteJointDesc::new(previous, ground, right_end));

    // A load dropped on it.
    for i in 0..5 {
        let position = vec2(-4.0 + 2.0 * i as f32, HEIGHT + 2.0 + (i % 2) as f32);
        let shape = if i % 2 == 0 { ColliderDesc::square(0.7) } else { ColliderDesc::circle(0.35) };
        world.add_body(BodyDesc::dynamic().position(position).collider(shape.density(1.5)));
    }
}
