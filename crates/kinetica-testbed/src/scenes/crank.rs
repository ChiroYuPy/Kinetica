use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Crank and piston",
    category: Category::SoftStructures,
    description: "Joints combine into machines: a revolute motor turns the crank, two hinges hold the rod, and \
                  a prismatic joint keeps the piston on its line.",
    camera_center: vec2(0.0, 2.5),
    view_height: 7.0,
    build: Build::Static(build),
};

const CRANK_LENGTH: f32 = 1.0;
const ROD_LENGTH: f32 = 3.0;

fn build(world: &mut World, _labels: &mut Labels) {
    add_ground(world, 40.0);
    let pivot = vec2(-2.5, 2.5);
    let frame = world.add_body(BodyDesc::fixed().position(pivot));

    let crank_shape = ColliderDesc::rectangle(CRANK_LENGTH + 0.2, 0.2);
    let crank =
        world.add_body(BodyDesc::dynamic().position(pivot + vec2(CRANK_LENGTH / 2.0, 0.0)).collider(crank_shape));
    world.add_joint(RevoluteJointDesc::new(frame, crank, pivot).motor(2.0, 1000.0));

    let crank_pin = pivot + vec2(CRANK_LENGTH, 0.0);
    let piston_pin = crank_pin + vec2(ROD_LENGTH, 0.0);
    let rod_shape = ColliderDesc::rectangle(ROD_LENGTH, 0.12);
    let rod = world.add_body(BodyDesc::dynamic().position((crank_pin + piston_pin) / 2.0).collider(rod_shape));
    world.add_joint(RevoluteJointDesc::new(crank, rod, crank_pin));

    let piston = world.add_body(BodyDesc::dynamic().position(piston_pin).collider(ColliderDesc::rectangle(0.8, 0.6)));
    world.add_joint(RevoluteJointDesc::new(rod, piston, piston_pin));
    world.add_joint(PrismaticJointDesc::new(frame, piston, piston_pin, Vec2::X));
}
