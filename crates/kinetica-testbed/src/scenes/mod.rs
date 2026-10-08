//! Demonstration scenes. Add a scene by writing a module and listing it in [`SCENES`].

mod body_types;
mod falling_shapes;

use kinetica::World;
use kinetica::math::Vec2;

/// Every scene, in the order of the number keys.
pub const SCENES: &[Scene] = &[falling_shapes::SCENE, body_types::SCENE];

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

impl Scene {
    pub fn build_world(&self) -> World {
        let mut world = World::new();
        (self.build)(&mut world);
        world
    }
}
