//! Demonstration scenes. Add one by writing a module and listing it in [`SCENES`].
//!
//! A scene shows one idea of the engine, in the simplest setting where it can be seen: if two
//! things need explaining, they are two scenes. Its description states the idea in a sentence or
//! two; labels in the world only name what differs between the things shown, and a scene needs few
//! of them. Many bodies are welcome, many things to understand are not.

mod arch;
mod body_types;
mod breakable_joint;
mod bridge;
mod bullets;
mod center_of_mass;
mod chains;
mod cloth;
mod contact_events;
mod contact_points;
mod crank;
mod distance_joint;
mod forces;
mod friction;
mod gravity;
mod gravity_scale;
mod impulses;
mod joint_limits;
mod joint_motors;
mod layers;
mod locked_rotation;
mod mass;
mod one_way;
mod overlaps;
mod pendulum_wave;
mod prismatic_joint;
mod pyramid;
mod queries;
mod ragdolls;
mod ray_cast;
mod ray_cast_all;
mod restitution;
mod revolute_joint;
mod rope;
mod sensors;
mod shape_cast;
mod shapes;
mod sleeping;
mod soft_bodies;
mod soft_body_pool;
mod springs;
mod stacking;
mod tearing;
mod trampoline;
mod weld_joint;
mod wheel_joint;

use std::collections::{HashSet, VecDeque};

use kinetica::math::Vec2;
use kinetica::prelude::*;
use macroquad::input::KeyCode;

use kinetica_macroquad::DebugRenderer;

/// Every scene, in menu order; the command line selects them by position, from 1.
pub const SCENES: &[Scene] = &[
    body_types::SCENE,
    mass::SCENE,
    center_of_mass::SCENE,
    gravity::SCENE,
    gravity_scale::SCENE,
    locked_rotation::SCENE,
    forces::SCENE,
    impulses::SCENE,
    shapes::SCENE,
    chains::SCENE,
    friction::SCENE,
    restitution::SCENE,
    contact_points::SCENE,
    contact_events::SCENE,
    stacking::SCENE,
    revolute_joint::SCENE,
    prismatic_joint::SCENE,
    joint_limits::SCENE,
    joint_motors::SCENE,
    distance_joint::SCENE,
    springs::SCENE,
    weld_joint::SCENE,
    breakable_joint::SCENE,
    wheel_joint::SCENE,
    rope::SCENE,
    trampoline::SCENE,
    bridge::SCENE,
    cloth::SCENE,
    tearing::SCENE,
    soft_bodies::SCENE,
    soft_body_pool::SCENE,
    ragdolls::SCENE,
    crank::SCENE,
    sleeping::SCENE,
    layers::SCENE,
    one_way::SCENE,
    sensors::SCENE,
    bullets::SCENE,
    ray_cast::SCENE,
    ray_cast_all::SCENE,
    shape_cast::SCENE,
    overlaps::SCENE,
    pyramid::SCENE,
    arch::SCENE,
    pendulum_wave::SCENE,
];

pub struct Scene {
    pub name: &'static str,
    pub category: Category,
    /// What the scene teaches and the keys it reads, in a sentence or two.
    pub description: &'static str,
    /// Initial camera target, in meters.
    pub camera_center: Vec2,
    /// Initial vertical extent of the view, in meters.
    pub view_height: f32,
    /// Fills an empty world with the scene content.
    pub build: Build,
}

/// Groups scenes in the menu, from the first concepts to learn to the most specialized.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Category {
    Bodies,
    Collisions,
    Joints,
    SoftStructures,
    GameFeatures,
    Queries,
    Benchmarks,
}

impl Category {
    pub const ALL: [Self; 7] = [
        Self::Bodies,
        Self::Collisions,
        Self::Joints,
        Self::SoftStructures,
        Self::GameFeatures,
        Self::Queries,
        Self::Benchmarks,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Bodies => "Bodies",
            Self::Collisions => "Shapes and collisions",
            Self::Joints => "Joints",
            Self::SoftStructures => "Built from joints",
            Self::GameFeatures => "Game features",
            Self::Queries => "Queries",
            Self::Benchmarks => "Stability and accuracy",
        }
    }
}

/// How a scene fills its world.
pub enum Build {
    /// The scene only needs its initial content.
    Static(fn(&mut World, &mut Labels)),
    /// The scene also runs code before every step, for example to read the keyboard.
    Animated(fn(&mut World, &mut Labels) -> Box<dyn SceneUpdate>),
}

/// What an animated scene knows about the current step.
#[derive(Debug, Clone, Default)]
pub struct Frame {
    /// Simulated time since the scene started, in seconds.
    pub time: f32,
    /// Mouse position, in world coordinates.
    pub mouse: Vec2,
    /// Keys held down.
    pub keys_down: HashSet<KeyCode>,
    /// Keys pressed since the previous step.
    pub keys_pressed: HashSet<KeyCode>,
}

impl Frame {
    pub fn is_down(&self, key: KeyCode) -> bool {
        self.keys_down.contains(&key)
    }

    pub fn is_pressed(&self, key: KeyCode) -> bool {
        self.keys_pressed.contains(&key)
    }

    /// Reads a key pair as an axis: `-1`, `0` or `1`.
    pub fn axis(&self, negative: KeyCode, positive: KeyCode) -> f32 {
        f32::from(i8::from(self.is_down(positive)) - i8::from(self.is_down(negative)))
    }
}

/// Code run by an animated scene.
pub trait SceneUpdate {
    /// Runs before every step.
    fn update(&mut self, world: &mut World, frame: &Frame);

    /// Draws on top of the world, every frame, even when paused.
    fn draw(&self, _world: &World, _frame: &Frame, _drawer: &mut DebugRenderer) {}

    /// Adds labels whose text changes, every frame.
    fn labels(&self, _world: &World, _frame: &Frame, _labels: &mut Labels) {}

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

/// Text written in the world to explain what is shown.
pub struct Label {
    pub anchor: Anchor,
    /// One or more lines.
    pub text: String,
}

/// Where a label is drawn.
#[derive(Debug, Clone, Copy)]
pub enum Anchor {
    /// Centered on a fixed point.
    Point(Vec2),
    /// Just above a body, following it.
    Above(BodyHandle),
}

impl Label {
    /// The point the bottom center of the text sits on, or `None` if its body is gone.
    pub fn position(&self, world: &World) -> Option<Vec2> {
        match self.anchor {
            Anchor::Point(point) => Some(point),
            Anchor::Above(body) => {
                let body = world.get_body(body)?;
                let top = body
                    .colliders()
                    .iter()
                    .map(|&collider| world[collider].aabb().max.y)
                    .fold(body.position().y, f32::max);
                Some(vec2(body.position().x, top + 0.15))
            }
        }
    }
}

/// The labels of a scene.
#[derive(Default)]
pub struct Labels(Vec<Label>);

impl Labels {
    /// Writes `text` centered on `point`.
    pub fn at(&mut self, point: Vec2, text: impl Into<String>) {
        self.0.push(Label { anchor: Anchor::Point(point), text: text.into() });
    }

    /// Writes `text` above `body`, wherever it goes.
    pub fn above(&mut self, body: BodyHandle, text: impl Into<String>) {
        self.0.push(Label { anchor: Anchor::Above(body), text: text.into() });
    }

    pub fn iter(&self) -> impl Iterator<Item = &Label> {
        self.0.iter()
    }
}

/// Adds a fixed ground `width` wide, centered on `x = 0`, whose top surface is at `y = 0`.
fn add_ground(world: &mut World, width: f32) -> BodyHandle {
    world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)).collider(ColliderDesc::rectangle(width, 1.0)))
}

/// Adds a small fixed pin marking a pivot at `position`; a sensor, so things swing through it.
fn add_pin(world: &mut World, position: Vec2) -> BodyHandle {
    world.add_body(BodyDesc::fixed().position(position).collider(ColliderDesc::circle(0.12).sensor()))
}

/// Spreads `i = 0, 1, 2…` evenly over `0..1`, never twice on the same spot: golden ratio steps.
fn scatter(i: usize) -> f32 {
    (i as f32 * 0.618_034).fract()
}

/// Adds a body at a regular pace and removes the oldest ones, so a scene can run forever.
pub struct Spawner {
    period: f32,
    capacity: usize,
    /// The bodies still in the world with their number, oldest first.
    bodies: VecDeque<(usize, BodyHandle)>,
    spawned: usize,
}

impl Spawner {
    /// A new body every `period` seconds, at most `capacity` at once.
    pub fn new(period: f32, capacity: usize) -> Self {
        Self { period, capacity, bodies: VecDeque::new(), spawned: 0 }
    }

    /// When it is time for a new body, adds the one `make` describes from its number.
    pub fn update(&mut self, world: &mut World, time: f32, make: impl FnOnce(usize) -> BodyDesc) {
        if time < self.period * self.spawned as f32 {
            return;
        }
        self.bodies.push_back((self.spawned, world.add_body(make(self.spawned))));
        self.spawned += 1;
        if self.bodies.len() > self.capacity
            && let Some((_, oldest)) = self.bodies.pop_front()
        {
            world.remove_body(oldest);
        }
    }

    /// The bodies still in the world, with their number.
    pub fn bodies(&self) -> impl Iterator<Item = (usize, BodyHandle)> + '_ {
        self.bodies.iter().copied()
    }
}

impl Scene {
    /// Builds a fresh world for the scene, with its labels and its per-step update if it has one.
    pub fn build_world(&self) -> (World, Labels, Option<Box<dyn SceneUpdate>>) {
        let mut world = World::new();
        let mut labels = Labels::default();
        let update = match self.build {
            Build::Static(build) => {
                build(&mut world, &mut labels);
                None
            }
            Build::Animated(build) => Some(build(&mut world, &mut labels)),
        };
        (world, labels, update)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The interface keeps room for a few lines only.
    #[test]
    fn descriptions_are_short() {
        for scene in SCENES {
            assert!(scene.description.chars().count() <= 160, "{}: description too long", scene.name);
        }
    }

    /// Every scene builds and runs for a few seconds without anything exploding.
    #[test]
    fn every_scene_runs() {
        for scene in SCENES {
            let (mut world, mut labels, mut update) = scene.build_world();
            for step in 0..180 {
                let frame = Frame { time: step as f32 / 60.0, mouse: scene.camera_center, ..Frame::default() };
                if let Some(update) = &mut update {
                    update.update(&mut world, &frame);
                    update.labels(&world, &frame, &mut labels);
                }
                world.step(1.0 / 60.0);
            }
            for (_, body) in world.bodies() {
                let position = body.position();
                assert!(
                    position.is_finite() && position.length() < 1000.0,
                    "{}: a body flew to {position}",
                    scene.name
                );
            }
        }
    }
}
