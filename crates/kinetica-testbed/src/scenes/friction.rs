use kinetica::prelude::*;

use super::Scene;

pub const SCENE: Scene = Scene {
    name: "Friction",
    description: "Friction 0.8, 0.4, 0.2, 0.05, 0 on a ramp of friction 1 (combined: sqrt of the product).",
    camera_center: vec2(0.0, 6.0),
    view_height: 18.0,
    build,
};

const FRICTIONS: [f32; 5] = [0.8, 0.4, 0.2, 0.05, 0.0];

fn build(world: &mut World) {
    let angle = -0.45_f32;
    let ground = world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)));
    world.add_collider(ground, ColliderDesc::rectangle(30.0, 0.5));

    let ramp = world.add_body(BodyDesc::fixed().position(vec2(0.0, 6.0)).angle(angle));
    world.add_collider(ramp, ColliderDesc::rectangle(9.0, 0.25).friction(1.0));

    // Boxes start side by side near the top of the ramp, resting on it.
    let along = Rot::from_angle(angle).rotate(Vec2::X);
    let up = Rot::from_angle(angle).rotate(Vec2::Y);
    for (i, friction) in FRICTIONS.into_iter().enumerate() {
        let position = vec2(0.0, 6.0) + (-7.5 + 1.6 * i as f32) * along + 0.75 * up;
        let block = world.add_body(BodyDesc::dynamic().position(position).angle(angle));
        world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5).friction(friction));
    }
}
