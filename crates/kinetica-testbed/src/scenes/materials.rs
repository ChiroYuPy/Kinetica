use kinetica::math::Rot;
use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Materials",
    category: Category::Collisions,
    description: "Each collider has a friction and a restitution. Left, boxes of decreasing friction on a ramp: the \
                  grippy ones stay, the slippery ones slide. Right, balls of increasing restitution dropped from the \
                  same height: 0 does not bounce, 1 bounces back up forever. When two colliders touch, their \
                  frictions are combined by geometric mean and the larger restitution wins.",
    camera_center: vec2(-1.5, 5.0),
    view_height: 17.0,
    build: Build::Static(build),
};

const FRICTIONS: [f32; 5] = [0.8, 0.4, 0.2, 0.05, 0.0];
const RESTITUTIONS: [f32; 6] = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
/// The ramp's lower end touches the ground.
const RAMP_CENTER: Vec2 = vec2(-7.0, 2.8);
const RAMP_ANGLE: f32 = -0.45;

fn build(world: &mut World, labels: &mut Labels) {
    let ground = add_ground(world, 40.0);
    // Stops the slippery boxes before they reach the balls.
    world.add_collider(ground, ColliderDesc::offset_rectangle(0.2, 0.75, vec2(0.6, 1.25), 0.0));

    let ramp = world.add_body(BodyDesc::fixed().position(RAMP_CENTER).angle(RAMP_ANGLE));
    world.add_collider(ramp, ColliderDesc::rectangle(6.0, 0.2).friction(1.0));
    labels.at(RAMP_CENTER + vec2(-3.0, -2.2), "Ramp friction 1, slope 24°");

    // Boxes start side by side near the top of the ramp, resting on it.
    let rotation = Rot::from_angle(RAMP_ANGLE);
    let (along, up) = (rotation.rotate(Vec2::X), rotation.rotate(Vec2::Y));
    for (i, friction) in FRICTIONS.into_iter().enumerate() {
        let position = RAMP_CENTER + (-5.0 + 1.3 * i as f32) * along + 0.6 * up;
        let block = world.add_body(BodyDesc::dynamic().position(position).angle(RAMP_ANGLE));
        world.add_collider(block, ColliderDesc::rectangle(0.4, 0.4).friction(friction));
        labels.above(block, format!("{friction}"));
    }
    labels.at(RAMP_CENTER + vec2(-4.0, 4.5), "friction");

    for (i, restitution) in RESTITUTIONS.into_iter().enumerate() {
        let ball = world.add_body(BodyDesc::dynamic().position(vec2(2.5 + 1.4 * i as f32, 8.0)));
        world.add_collider(ball, ColliderDesc::circle(0.4).restitution(restitution));
        labels.above(ball, format!("{restitution}"));
    }
    labels.at(vec2(6.0, 10.0), "restitution");
}
