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

/// Inside size of the drum, and thickness of its walls.
const SIZE: f32 = 10.0;
const WALL: f32 = 0.5;

fn build(world: &mut World, labels: &mut Labels) {
    // One kinematic body made of four wall colliders.
    let drum = world.add_body(BodyDesc::kinematic().angular_velocity(0.3));
    for (center, width, height) in [
        (vec2(0.0, -SIZE / 2.0), SIZE, WALL),
        (vec2(0.0, SIZE / 2.0), SIZE, WALL),
        (vec2(-SIZE / 2.0, 0.0), WALL, SIZE),
        (vec2(SIZE / 2.0, 0.0), WALL, SIZE),
    ] {
        world.add_collider(drum, ColliderDesc::rectangle(width, height).offset(center));
    }

    labels.at(vec2(0.0, 7.3), "One kinematic body, four colliders, turning at 0.3 rad/s");

    let per_side = 12;
    let spacing = (SIZE - 2.0) / (per_side - 1) as f32;
    for row in 0..per_side {
        for column in 0..per_side {
            let position = vec2(column as f32, row as f32) * spacing - Vec2::splat(SIZE / 2.0 - 1.0);
            let shape = match (row * per_side + column) % 4 {
                0 => ColliderDesc::circle(0.25),
                1 => ColliderDesc::rectangle(0.5, 0.3),
                2 => ColliderDesc::regular_polygon(3, 0.28),
                _ => ColliderDesc::regular_polygon(5, 0.25),
            };
            world.add_body(BodyDesc::dynamic().position(position).collider(shape));
        }
    }
}
