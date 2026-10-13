use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground, add_pin};

pub const SCENE: Scene = Scene {
    name: "Distance joint",
    category: Category::Joints,
    description: "A distance joint keeps two points a fixed distance apart, like a rod. As a rope, it only \
                  stops them from getting further: it is slack until pulled tight.",
    camera_center: vec2(0.0, 4.5),
    view_height: 11.0,
    build: Build::Static(build),
};

const TOP: f32 = 8.0;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);

    // Both bobs start 1.5 m from their pivot, to the right and a little above.
    let offset = vec2(1.4, 0.5);
    let pivot = vec2(-4.0, TOP);
    let (pin, bob) = (add_pin(world, pivot), add_bob(world, pivot + offset));
    world.add_joint(DistanceJointDesc::new(pin, bob, pivot, pivot + offset));
    labels.at(pivot + vec2(0.0, 0.5), "Rod");

    // A 3 m rope: the bob falls freely until the rope is tight.
    let pivot = vec2(3.0, TOP);
    let (pin, bob) = (add_pin(world, pivot), add_bob(world, pivot + offset));
    world.add_joint(DistanceJointDesc::new(pin, bob, pivot, pivot + offset).rope(3.0));
    labels.at(pivot + vec2(0.0, 0.5), "Rope");
}

fn add_bob(world: &mut World, position: Vec2) -> BodyHandle {
    world.add_body(BodyDesc::dynamic().position(position).collider(ColliderDesc::circle(0.3)))
}
