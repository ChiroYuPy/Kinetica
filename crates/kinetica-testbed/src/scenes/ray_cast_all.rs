use kinetica::CollisionLayers;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;

use super::queries::{FOUND_COLOR, NORMAL_COLOR, PROBE_COLOR, add_targets};
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Ray cast, all hits",
    category: Category::Queries,
    description: "A ray can also report every collider it crosses, from the nearest to the farthest. Move the \
                  mouse sideways to scan.",
    camera_center: vec2(0.0, 5.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

const TOP: f32 = 11.0;

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_targets(world, CollisionLayers::default());
    Box::new(Scanner)
}

struct Scanner;

impl SceneUpdate for Scanner {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        // Down to just above the ground.
        let (start, translation) = (vec2(frame.mouse.x, TOP), vec2(0.0, 0.1 - TOP));
        drawer.draw_segment(start, start + translation, PROBE_COLOR);
        for hit in world.cast_ray_all(start, translation, QueryFilter::new()) {
            drawer.draw_collider(world, hit.collider, FOUND_COLOR);
            drawer.draw_point(hit.point, NORMAL_COLOR);
        }
    }
}
