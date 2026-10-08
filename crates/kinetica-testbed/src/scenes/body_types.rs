use kinetica::prelude::*;

use super::{Build, Category, Scene};

pub const SCENE: Scene = Scene {
    name: "Body types",
    category: Category::Basics,
    description: "Gray: fixed. Blue: kinematic (scripted motion). Orange: dynamic (simulated). Press C to see centers of mass.",
    camera_center: vec2(0.0, 3.0),
    view_height: 14.0,
    build: Build::Static(build),
};

fn build(world: &mut World) {
    // Fixed frame.
    for (position, half_width, half_height) in
        [(vec2(0.0, -2.0), 9.0, 0.3), (vec2(-9.0, 3.0), 0.3, 5.0), (vec2(9.0, 3.0), 0.3, 5.0)]
    {
        let wall = world.add_body(BodyDesc::fixed().position(position));
        world.add_collider(wall, ColliderDesc::rectangle(half_width, half_height));
    }

    // Kinematic: a spinner and a slowly sliding platform.
    let spinner = world.add_body(BodyDesc::kinematic().position(vec2(-4.0, 3.0)).angular_velocity(1.5));
    world.add_collider(spinner, ColliderDesc::rectangle(2.0, 0.15));
    let platform = world.add_body(BodyDesc::kinematic().position(vec2(-6.0, 0.0)).linear_velocity(vec2(0.4, 0.0)));
    world.add_collider(platform, ColliderDesc::rectangle(1.2, 0.2));

    // Dynamic, floating: spins around its center of mass, not around its origin.
    let compound =
        world.add_body(BodyDesc::dynamic().position(vec2(3.0, 4.0)).gravity_scale(0.0).angular_velocity(2.0));
    world.add_collider(compound, ColliderDesc::rectangle(1.0, 0.2));
    world.add_collider(compound, ColliderDesc::new(kinetica::shape::Circle::new(0.5).with_center(vec2(1.2, 0.0))));

    // Dynamic, rising: negative gravity scale.
    let balloon = world.add_body(BodyDesc::dynamic().position(vec2(6.0, 0.0)).gravity_scale(-0.1));
    world.add_collider(balloon, ColliderDesc::circle(0.6));

    // Dynamic, heavily damped: falls at a slow terminal velocity.
    let feather =
        world.add_body(BodyDesc::dynamic().position(vec2(0.0, 8.0)).linear_damping(4.0).angular_velocity(1.0));
    world.add_collider(feather, ColliderDesc::regular_polygon(5, 0.5));
}
