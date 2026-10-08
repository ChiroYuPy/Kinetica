use kinetica::prelude::*;

use super::{Build, Category, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Seesaw",
    category: Category::Contacts,
    description: "A heavy box drops on a plank resting on a fulcrum and launches a light ball.",
    camera_center: vec2(0.0, 5.0),
    view_height: 16.0,
    build: Build::Static(build),
};

fn build(world: &mut World) {
    add_ground(world, 40.0);

    let fulcrum = world.add_body(BodyDesc::fixed());
    let triangle = [vec2(-0.7, 0.0), vec2(0.7, 0.0), vec2(0.0, 1.0)];
    world.add_collider(fulcrum, ColliderDesc::convex_hull(&triangle).expect("the fulcrum is a triangle"));

    let plank = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 1.15)));
    world.add_collider(plank, ColliderDesc::rectangle(5.0, 0.15).friction(0.8));

    let ball = world.add_body(BodyDesc::dynamic().position(vec2(-4.5, 1.7)));
    world.add_collider(ball, ColliderDesc::circle(0.4).density(0.5));

    let weight = world.add_body(BodyDesc::dynamic().position(vec2(4.3, 9.0)));
    world.add_collider(weight, ColliderDesc::rectangle(0.6, 0.6).density(8.0));
}
