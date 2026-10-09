use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Distance joints",
    category: Category::Joints,
    description: "A distance joint keeps two anchor points at a given distance. Without a spring it is a rigid \
                  rod. A spring of zero stiffness limited to 0..length is a rope: slack until pulled tight. A real \
                  spring oscillates at its frequency; its damping ratio decides how fast it settles: 0 never \
                  does, 1 settles without bouncing.",
    camera_center: vec2(-2.75, 5.0),
    view_height: 15.0,
    build: Build::Static(build),
};

const TOP: f32 = 9.0;
const DAMPING_RATIOS: [f32; 4] = [0.0, 0.1, 0.3, 1.0];

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);

    // A rod, released horizontally.
    let pivot = add_anchor(world, vec2(-10.0, TOP));
    let bob = add_bob(world, vec2(-7.0, TOP));
    world.add_joint(DistanceJointDesc::between_world_points(world, pivot, bob, vec2(-10.0, TOP), vec2(-7.0, TOP)));
    labels.at(vec2(-9.0, TOP + 1.0), "Rod");

    // A rope, 3 m long, its bob starting closer than that: it falls freely until the rope is tight.
    let pivot = add_anchor(world, vec2(-4.0, TOP));
    let start = vec2(-2.8, TOP + 0.6);
    let bob = add_bob(world, start);
    let rope = DistanceJointDesc::between_world_points(world, pivot, bob, vec2(-4.0, TOP), start)
        .spring(0.0, 0.0)
        .limits(0.0..=3.0);
    world.add_joint(rope);
    labels.at(vec2(-4.0, TOP + 1.6), "Rope");

    // 1 Hz springs with a 3 m rest length, starting compressed to 1.5 m.
    for (i, damping_ratio) in DAMPING_RATIOS.into_iter().enumerate() {
        let anchor_position = vec2(1.5 + 1.8 * i as f32, TOP);
        let anchor = add_anchor(world, anchor_position);
        let weight_position = anchor_position - vec2(0.0, 1.5);
        let weight = world.add_body(BodyDesc::dynamic().position(weight_position));
        world.add_collider(weight, ColliderDesc::rectangle(0.4, 0.4));
        world.add_joint(
            DistanceJointDesc::between_world_points(world, anchor, weight, anchor_position, weight_position)
                .length(3.0)
                .spring(1.0, damping_ratio),
        );
        labels.at(anchor_position + vec2(0.0, 0.4), format!("{damping_ratio}"));
    }
    labels.at(vec2(4.2, TOP + 1.3), "Springs, 1 Hz, damping ratio:");
}

fn add_anchor(world: &mut World, position: Vec2) -> BodyHandle {
    let anchor = world.add_body(BodyDesc::fixed().position(position));
    world.add_collider(anchor, ColliderDesc::circle(0.1).sensor(true));
    anchor
}

fn add_bob(world: &mut World, position: Vec2) -> BodyHandle {
    let bob = world.add_body(BodyDesc::dynamic().position(position));
    world.add_collider(bob, ColliderDesc::circle(0.3));
    bob
}
