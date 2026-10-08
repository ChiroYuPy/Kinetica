use kinetica::prelude::*;

use super::{Build, Scene};

pub const SCENE: Scene = Scene {
    name: "Pool break",
    description: "Top-down pool: no gravity, damping as cloth friction, bouncy balls and cushions.",
    camera_center: vec2(0.0, 0.6),
    view_height: 10.0,
    build: Build::Static(build),
};

const BALL_RADIUS: f32 = 0.18;
const TABLE_HALF_WIDTH: f32 = 6.0;
const TABLE_HALF_HEIGHT: f32 = 3.0;

fn build(world: &mut World) {
    world.set_gravity(Vec2::ZERO);

    let table = world.add_body(BodyDesc::fixed());
    let cushion = 0.2;
    for (center, half_width, half_height) in [
        (vec2(0.0, -TABLE_HALF_HEIGHT - cushion), TABLE_HALF_WIDTH + 2.0 * cushion, cushion),
        (vec2(0.0, TABLE_HALF_HEIGHT + cushion), TABLE_HALF_WIDTH + 2.0 * cushion, cushion),
        (vec2(-TABLE_HALF_WIDTH - cushion, 0.0), cushion, TABLE_HALF_HEIGHT),
        (vec2(TABLE_HALF_WIDTH + cushion, 0.0), cushion, TABLE_HALF_HEIGHT),
    ] {
        world.add_collider(
            table,
            ColliderDesc::offset_rectangle(half_width, half_height, center, 0.0).restitution(0.8).friction(0.2),
        );
    }

    // Fifteen balls racked in a triangle, apex towards the cue ball.
    let row_spacing = 2.0 * BALL_RADIUS * 3f32.sqrt() / 2.0 + 0.002;
    for row in 0..5 {
        for i in 0..=row {
            let x = 2.5 + row as f32 * row_spacing;
            let y = (i as f32 - row as f32 / 2.0) * (2.0 * BALL_RADIUS + 0.002);
            add_ball(world, vec2(x, y), Vec2::ZERO);
        }
    }
    add_ball(world, vec2(-3.5, 0.0), vec2(14.0, 0.15));
}

fn add_ball(world: &mut World, position: Vec2, velocity: Vec2) {
    let ball = world.add_body(
        BodyDesc::dynamic().position(position).linear_velocity(velocity).linear_damping(0.6).angular_damping(1.0),
    );
    world.add_collider(ball, ColliderDesc::circle(BALL_RADIUS).restitution(0.95).friction(0.05));
}
