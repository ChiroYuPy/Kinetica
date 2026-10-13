use kinetica::math::Rot;
use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Friction",
    category: Category::Collisions,
    description: "Each collider has a friction, from 0 (ice) to 1 and more (rubber). On the same ramp, the grippy \
                  boxes stay and the slippery ones slide.",
    camera_center: vec2(0.0, 3.5),
    view_height: 11.0,
    build: Build::Static(build),
};

const FRICTIONS: [f32; 5] = [0.8, 0.4, 0.2, 0.05, 0.0];
/// The ramp's lower end touches the ground.
const RAMP_CENTER: Vec2 = vec2(-1.5, 2.8);
const RAMP_ANGLE: f32 = -0.45;

fn build(world: &mut World, labels: &mut Labels) {
    let ground = add_ground(world, 40.0);
    world.add_collider(ground, ColliderDesc::rectangle(0.5, 2.0).offset(vec2(7.5, 1.0)));
    world.add_body(
        BodyDesc::fixed()
            .position(RAMP_CENTER)
            .angle(RAMP_ANGLE)
            .collider(ColliderDesc::rectangle(12.0, 0.4).friction(1.0)),
    );

    // Side by side near the top of the ramp, resting on it.
    let rotation = Rot::from_angle(RAMP_ANGLE);
    let (along, up) = (rotation.rotate(Vec2::X), rotation.rotate(Vec2::Y));
    for (i, friction) in FRICTIONS.into_iter().enumerate() {
        let position = RAMP_CENTER + (-5.0 + 1.3 * i as f32) * along + 0.6 * up;
        let shape = ColliderDesc::square(0.8).friction(friction);
        let block = world.add_body(BodyDesc::dynamic().position(position).angle(RAMP_ANGLE).collider(shape));
        labels.above(block, format!("{friction}"));
    }
}
