//! Demonstration scenes, each showing one engine concept. Add a scene by writing a module and
//! listing it in [`SCENES`].

mod body_types;
mod bullets;
mod cloth;
mod contacts;
mod distance_joints;
mod filtering;
mod forces;
mod gravity;
mod mass;
mod materials;
mod pendulum_wave;
mod prismatic_joints;
mod pyramid;
mod queries;
mod ragdolls;
mod revolute_joints;
mod ropes;
mod sensors;
mod shapes;
mod sleeping;
mod soft_bodies;
mod stacks;
mod weld_joints;
mod wheel_joints;

use std::collections::HashSet;

use kinetica::math::Vec2;
use kinetica::prelude::*;
use macroquad::input::KeyCode;

use kinetica_macroquad::DebugRenderer;

/// Every scene, in menu order; the command line selects them by position, from 1.
pub const SCENES: &[Scene] = &[
    body_types::SCENE,
    mass::SCENE,
    gravity::SCENE,
    forces::SCENE,
    shapes::SCENE,
    materials::SCENE,
    contacts::SCENE,
    stacks::SCENE,
    revolute_joints::SCENE,
    prismatic_joints::SCENE,
    distance_joints::SCENE,
    weld_joints::SCENE,
    wheel_joints::SCENE,
    ropes::SCENE,
    cloth::SCENE,
    soft_bodies::SCENE,
    ragdolls::SCENE,
    sleeping::SCENE,
    filtering::SCENE,
    sensors::SCENE,
    bullets::SCENE,
    queries::SCENE,
    pyramid::SCENE,
    pendulum_wave::SCENE,
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
