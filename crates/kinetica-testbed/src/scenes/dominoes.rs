use kinetica::prelude::*;

use super::{Build, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Dominoes",
    description: "The first domino is tipped over; the others follow, up the steps.",
    camera_center: vec2(4.0, 3.0),
    view_height: 14.0,
    build: Build::Static(build),
};

const HALF_THICKNESS: f32 = 0.1;
const HALF_HEIGHT: f32 = 0.8;
const SPACING: f32 = 1.0;

fn build(world: &mut World) {
    add_ground(world, 40.0);

    // Flat run, then three steps of rising height, each with a few dominoes on top.
    let mut x = -10.0;
    let first = add_domino(world, x, 0.0);
    x += SPACING;
    for _ in 1..10 {
        add_domino(world, x, 0.0);
        x += SPACING;
    }
    for step_height in [0.4, 0.8, 1.2] {
        let step_half_width = 1.5;
        let step = world.add_body(BodyDesc::fixed().position(vec2(x + step_half_width - 0.4, step_height / 2.0)));
        world.add_collider(step, ColliderDesc::rectangle(step_half_width, step_height / 2.0));
        for _ in 0..3 {
            add_domino(world, x, step_height);
            x += SPACING;
        }
    }

    // A ball at the end of the run, knocked off by the last domino.
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(x + 0.2, 1.2 + 0.5)));
    world.add_collider(ball, ColliderDesc::circle(0.5));

    // Tip the first domino towards the others (clockwise).
    if let Some(first) = world.body_mut(first) {
        first.set_angular_velocity(-2.0);
    }
}

fn add_domino(world: &mut World, x: f32, floor: f32) -> BodyHandle {
    let domino = world.add_body(BodyDesc::dynamic().position(vec2(x, floor + HALF_HEIGHT)));
    world.add_collider(domino, ColliderDesc::rectangle(HALF_THICKNESS, HALF_HEIGHT).density(2.0));
    domino
}
