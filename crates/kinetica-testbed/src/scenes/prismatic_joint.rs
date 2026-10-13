use kinetica::math::Rot;
use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Prismatic joint",
    category: Category::Joints,
    description: "A prismatic joint lets a body slide along an axis without turning, like a drawer or a piston. \
                  Grab the blocks: they only follow their rail.",
    camera_center: vec2(0.0, 3.5),
    view_height: 11.0,
    build: Build::Static(build),
};

const RAIL_LENGTH: f32 = 5.0;

fn build(world: &mut World, _labels: &mut Labels) {
    add_ground(world, 40.0);
    add_rail(world, vec2(-4.0, 4.0), 0.5);
    add_rail(world, vec2(4.0, 3.0), 0.0);
}

/// A block on a rail tilted by `angle`, starting at its upper end and stopped at both ends.
fn add_rail(world: &mut World, center: Vec2, angle: f32) {
    let axis = Rot::from_angle(angle).rotate(Vec2::X);
    // The rail is only drawn: a sensor pushes nothing.
    let rail_shape = ColliderDesc::rectangle(RAIL_LENGTH + 0.8, 0.08).sensor();
    let rail = world.add_body(BodyDesc::fixed().position(center).angle(angle).collider(rail_shape));

    let start = center + (RAIL_LENGTH / 2.0) * axis;
    let block =
        world.add_body(BodyDesc::dynamic().position(start).angle(angle).collider(ColliderDesc::rectangle(0.8, 0.5)));
    // Limits are measured along the axis from the starting position.
    world.add_joint(PrismaticJointDesc::new(rail, block, start, axis).limits(-RAIL_LENGTH..=0.0));
}
