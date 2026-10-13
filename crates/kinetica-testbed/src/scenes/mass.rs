use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Mass",
    category: Category::Bodies,
    description: "Mass is density times area. A small dense box balances a large light one; of two boxes of the \
                  same size, the denser wins.",
    camera_center: vec2(0.0, 2.0),
    view_height: 9.0,
    build: Build::Static(build),
};

const PLANK_LENGTH: f32 = 5.0;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);
    // 1 m² at density 1 and 0.25 m² at density 4: the same mass.
    add_seesaw(world, labels, -3.5, ColliderDesc::square(1.0), ColliderDesc::square(0.5).density(4.0));
    // The same size at densities 1 and 3.
    add_seesaw(world, labels, 3.5, ColliderDesc::square(1.0), ColliderDesc::square(1.0).density(3.0));
}

/// A plank on a pivot with a box near each end, labeled with its mass.
fn add_seesaw(world: &mut World, labels: &mut Labels, x: f32, left: ColliderDesc, right: ColliderDesc) {
    // The post stops short of the plank, which only rests on its pivot.
    let post = world.add_body(BodyDesc::fixed().position(vec2(x, 0.25)).collider(ColliderDesc::rectangle(0.24, 0.5)));
    let pivot = vec2(x, 0.7);
    let plank = world.add_body(
        BodyDesc::dynamic().position(pivot).collider(ColliderDesc::rectangle(PLANK_LENGTH, 0.2).friction(0.9)),
    );
    // Pinned to the post, not the ground: jointed bodies do not collide, and the plank must hit the ground.
    world.add_joint(RevoluteJointDesc::new(post, plank, pivot));

    for (side, collider) in [(-1.0, left), (1.0, right)] {
        let half_height = collider.shape.compute_aabb(&Transform::IDENTITY).max.y;
        let position = pivot + vec2(side * (PLANK_LENGTH / 2.0 - 0.6), 0.1 + half_height);
        let body = world.add_body(BodyDesc::dynamic().position(position).collider(collider.friction(0.9)));
        labels.above(body, format!("{:.0} kg", world[body].mass()));
    }
}
