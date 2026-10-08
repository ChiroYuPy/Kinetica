use kinetica::prelude::*;

use super::{Build, Category, Scene};

pub const SCENE: Scene = Scene {
    name: "Falling shapes",
    category: Category::Contacts,
    description: "Every kind of shape falls and piles up on the ground.",
    camera_center: vec2(0.0, 5.0),
    view_height: 16.0,
    build: Build::Static(build),
};

fn build(world: &mut World) {
    let ground = world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)));
    world.add_collider(ground, ColliderDesc::rectangle(15.0, 0.5));

    let (columns, rows) = (8, 5);
    for row in 0..rows {
        for column in 0..columns {
            let x = (column as f32 - (columns - 1) as f32 / 2.0) * 1.6;
            let y = 3.0 + row as f32 * 1.6;
            let spin = (column as f32 - 3.5) * 0.4;
            let body = world.add_body(BodyDesc::dynamic().position(vec2(x, y)).angular_velocity(spin));

            let collider = match (row + column) % 4 {
                0 => ColliderDesc::circle(0.5),
                1 => ColliderDesc::rectangle(0.6, 0.35),
                2 => ColliderDesc::regular_polygon(3, 0.6),
                _ => ColliderDesc::regular_polygon(6, 0.55),
            };
            world.add_collider(body, collider);
        }
    }
}
