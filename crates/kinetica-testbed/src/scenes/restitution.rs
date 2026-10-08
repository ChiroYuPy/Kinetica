use kinetica::prelude::*;

use super::{Build, Category, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Restitution",
    category: Category::Contacts,
    description: "Balls with restitution from 0 (left) to 1 (right), dropped from the same height.",
    camera_center: vec2(0.0, 5.0),
    view_height: 14.0,
    build: Build::Static(build),
};

fn build(world: &mut World) {
    add_ground(world, 40.0);
    let count = 11;
    for i in 0..count {
        let restitution = i as f32 / (count - 1) as f32;
        let x = (i as f32 - (count - 1) as f32 / 2.0) * 1.4;
        let ball = world.add_body(BodyDesc::dynamic().position(vec2(x, 9.0)));
        world.add_collider(ball, ColliderDesc::circle(0.5).restitution(restitution));
    }
}
