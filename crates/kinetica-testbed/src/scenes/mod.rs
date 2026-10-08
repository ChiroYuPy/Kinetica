//! Demonstration scenes. Add a scene by writing a module and listing it in [`SCENES`].

mod arch;
mod body_types;
mod contacts;
mod dominoes;
mod falling_shapes;
mod friction;
mod pool;
mod pyramid;
mod restitution;
mod seesaw;
mod stacks;
mod tumbler;

use kinetica::math::Vec2;
use kinetica::prelude::*;

/// Every scene, in the order of the number keys.
pub const SCENES: &[Scene] = &[
    falling_shapes::SCENE,
    pyramid::SCENE,
    stacks::SCENE,
    friction::SCENE,
    restitution::SCENE,
    dominoes::SCENE,
    arch::SCENE,
    seesaw::SCENE,
    tumbler::SCENE,
    pool::SCENE,
    body_types::SCENE,
    contacts::SCENE,
];

pub struct Scene {
    pub name: &'static str,
    pub description: &'static str,
    /// Initial camera target, in meters.
    pub camera_center: Vec2,
    /// Initial vertical extent of the view, in meters.
    pub view_height: f32,
    /// Fills an empty world with the scene content.
    pub build: fn(&mut World),
}

/// Adds a wide fixed ground whose top surface is at `y = 0`.
fn add_ground(world: &mut World, half_width: f32) {
    let ground = world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)));
    world.add_collider(ground, ColliderDesc::rectangle(half_width, 0.5));
}

impl Scene {
    pub fn build_world(&self) -> World {
        let mut world = World::new();
        (self.build)(&mut world);
        world
    }
}
