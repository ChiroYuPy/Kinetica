//! What the query scenes look at, and how they show what a query found.

use kinetica::prelude::*;
use macroquad::color::Color;

use super::add_ground;

/// What a query is drawn with.
pub const PROBE_COLOR: Color = Color::new(0.45, 0.75, 0.95, 1.0);
/// The colliders a query found.
pub const FOUND_COLOR: Color = Color::new(1.0, 0.85, 0.3, 1.0);
pub const NORMAL_COLOR: Color = Color::new(0.35, 0.85, 0.45, 1.0);

/// A few fixed bars in the air and a row of small bodies on the ground, all in `layers`.
pub fn add_targets(world: &mut World, layers: CollisionLayers) -> BodyHandle {
    add_ground(world, 80.0);
    let bars = world.add_body(BodyDesc::fixed());
    for (center, length, angle) in [
        (vec2(-3.0, 8.5), 3.0, -0.3),
        (vec2(5.0, 8.0), 2.4, 0.6),
        (vec2(8.5, 3.5), 2.0, 1.3),
        (vec2(1.0, 4.5), 2.4, 0.0),
    ] {
        world.add_collider(bars, ColliderDesc::rectangle(length, 0.3).offset(center).angle(angle).layers(layers));
    }
    for i in 0..14 {
        let position = vec2(-8.0 + 1.2 * i as f32, 0.6 + 0.3 * (i % 3) as f32);
        let shape = match i % 4 {
            0 => ColliderDesc::circle(0.45),
            1 => ColliderDesc::rectangle(1.0, 0.6),
            2 => ColliderDesc::regular_polygon(3, 0.55),
            _ => ColliderDesc::capsule(vec2(-0.4, 0.0), vec2(0.4, 0.0), 0.2),
        };
        world.add_body(BodyDesc::dynamic().position(position).angle(0.4 * i as f32).collider(shape.layers(layers)));
    }
    bars
}
