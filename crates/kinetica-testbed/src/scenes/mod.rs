//! Demonstration scenes. Add a scene by writing a module and listing it in [`SCENES`].

mod arch;
mod body_types;
mod bridge;
mod bullets;
mod cantilever;
mod car;
mod cloth;
mod contacts;
mod dominoes;
mod falling_shapes;
mod friction;
mod layers;
mod machines;
mod newtons_cradle;
mod one_way;
mod pendulum_wave;
mod pendulums;
mod pool;
mod pyramid;
mod queries;
mod ragdolls;
mod restitution;
mod ropes;
mod seesaw;
mod sensors;
mod shapes;
mod sleeping;
mod soft_bodies;
mod springs;
mod stacks;
mod tearing;
mod tumbler;
mod wrecking_ball;

use kinetica::math::Vec2;
use kinetica::prelude::*;

use kinetica_macroquad::DebugRenderer;

/// Every scene, grouped by category; the command line selects them by position, from 1.
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
    shapes::SCENE,
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
    cloth::SCENE,
    tearing::SCENE,
    soft_bodies::SCENE,
    ropes::SCENE,
    sleeping::SCENE,
    layers::SCENE,
    sensors::SCENE,
    bullets::SCENE,
    one_way::SCENE,
    queries::SCENE,
    body_types::SCENE,
    contacts::SCENE,
];

pub struct Scene {
    pub name: &'static str,
    pub category: Category,
    pub description: &'static str,
    /// Initial camera target, in meters.
    pub camera_center: Vec2,
    /// Initial vertical extent of the view, in meters.
    pub view_height: f32,
    /// Fills an empty world with the scene content.
    pub build: Build,
}

/// Groups scenes in the menu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Category {
    Contacts,
    Joints,
    SoftStructures,
    GameFeatures,
    Queries,
    Basics,
}

impl Category {
    pub const ALL: [Self; 6] =
        [Self::Contacts, Self::Joints, Self::SoftStructures, Self::GameFeatures, Self::Queries, Self::Basics];

    pub fn label(self) -> &'static str {
        match self {
            Self::Contacts => "Contacts",
            Self::Joints => "Joints",
            Self::SoftStructures => "Soft structures",
            Self::GameFeatures => "Game features",
            Self::Queries => "Queries",
            Self::Basics => "Engine basics",
        }
    }
}

/// How a scene fills its world.
pub enum Build {
    /// The scene only needs its initial content.
    Static(fn(&mut World)),
    /// The scene also runs code before every step, for example to read the keyboard.
    Animated(fn(&mut World) -> Box<dyn SceneUpdate>),
}

/// What an animated scene knows about the current frame.
pub struct Frame {
    /// Simulated time since the scene started, in seconds.
    pub time: f32,
    /// Mouse position, in world coordinates.
    pub mouse: Vec2,
}

/// Code run by an animated scene.
pub trait SceneUpdate {
    /// Runs before every step.
    fn update(&mut self, world: &mut World, frame: &Frame);

    /// Draws on top of the world, every frame, even when paused.
    fn draw(&self, _world: &World, _frame: &Frame, _drawer: &mut DebugRenderer) {}

    /// A point the camera should follow, if any.
    fn camera_target(&self, _world: &World) -> Option<Vec2> {
        None
    }
}

impl<F: FnMut(&mut World, &Frame)> SceneUpdate for F {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        self(world, frame);
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
