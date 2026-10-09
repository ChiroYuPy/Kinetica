use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene};

pub const SCENE: Scene = Scene {
    name: "Tumbler",
    category: Category::Benchmarks,
    description: "A slowly rotating kinematic drum full of shapes: many simultaneous contacts.",
    camera_center: vec2(0.0, 0.0),
    view_height: 16.0,
    build: Build::Static(build),
};

const HALF_SIZE: f32 = 5.0;
const WALL: f32 = 0.25;

fn build(world: &mut World, labels: &mut Labels) {
    // One kinematic body made of four wall colliders.
    let drum = world.add_body(BodyDesc::kinematic().angular_velocity(0.3));
    for (center, half_width, half_height) in [
        (vec2(0.0, -HALF_SIZE), HALF_SIZE, WALL),
        (vec2(0.0, HALF_SIZE), HALF_SIZE, WALL),
        (vec2(-HALF_SIZE, 0.0), WALL, HALF_SIZE),
        (vec2(HALF_SIZE, 0.0), WALL, HALF_SIZE),
    ] {
        world.add_collider(drum, ColliderDesc::offset_rectangle(half_width, half_height, center, 0.0));
    }

    labels.at(vec2(0.0, 7.3), "One kinematic body, four colliders, turning at 0.3 rad/s");

    let per_side = 12;
    let spacing = 2.0 * (HALF_SIZE - 1.0) / (per_side - 1) as f32;
    for row in 0..per_side {
        for column in 0..per_side {
            let position = vec2(column as f32, row as f32) * spacing - Vec2::splat(HALF_SIZE - 1.0);
            let body = world.add_body(BodyDesc::dynamic().position(position));
            let collider = match (row * per_side + column) % 4 {
                0 => ColliderDesc::circle(0.25),
                1 => ColliderDesc::rectangle(0.25, 0.15),
                2 => ColliderDesc::regular_polygon(3, 0.28),
                _ => ColliderDesc::regular_polygon(5, 0.25),
            };
            world.add_collider(body, collider);
        }
    }
}
