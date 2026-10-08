//! Demonstration scenes. Add a scene by writing a module and listing it in [`SCENES`].

mod arch;
mod body_types;
mod bridge;
mod cantilever;
mod car;
mod contacts;
mod dominoes;
mod falling_shapes;
mod friction;
mod machines;
mod newtons_cradle;
mod pendulum_wave;
mod pendulums;
mod pool;
mod pyramid;
mod ragdolls;
mod restitution;
mod seesaw;
mod springs;
mod stacks;
mod tumbler;
mod wrecking_ball;

use kinetica::math::Vec2;
use kinetica::prelude::*;

/// Every scene, in the order of the number keys and of the scene list.
pub const SCENES: &[Scene] = &[
    // Contacts.
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
    // Joints.
    pendulums::SCENE,
    pendulum_wave::SCENE,
    newtons_cradle::SCENE,
    wrecking_ball::SCENE,
    bridge::SCENE,
    ragdolls::SCENE,
    car::SCENE,
    springs::SCENE,
    machines::SCENE,
    cantilever::SCENE,
    // Engine basics.
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
    pub build: Build,
}

/// How a scene fills its world.
pub enum Build {
    /// The scene only needs its initial content.
    Static(fn(&mut World)),
    /// The scene also runs code before every step, for example to read the keyboard.
    Animated(fn(&mut World) -> Box<dyn SceneUpdate>),
}

/// Code run by an animated scene before every step; `time` is the simulated time in seconds.
pub trait SceneUpdate {
    fn update(&mut self, world: &mut World, time: f32);

    /// A point the camera should follow, if any.
    fn camera_target(&self, _world: &World) -> Option<Vec2> {
        None
    }
}

impl<F: FnMut(&mut World, f32)> SceneUpdate for F {
    fn update(&mut self, world: &mut World, time: f32) {
        self(world, time);
    }
}

/// Adds a wide fixed ground whose top surface is at `y = 0`.
fn add_ground(world: &mut World, half_width: f32) {
    let ground = world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)));
    world.add_collider(ground, ColliderDesc::rectangle(half_width, 0.5));
}

impl Scene {
    /// Builds a fresh world for the scene, with its per-step update if it has one.
    pub fn build_world(&self) -> (World, Option<Box<dyn SceneUpdate>>) {
        let mut world = World::new();
        let update = match self.build {
            Build::Static(build) => {
                build(&mut world);
                None
            }
            Build::Animated(build) => Some(build(&mut world)),
        };
        (world, update)
    }
}
