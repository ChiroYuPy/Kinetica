use kinetica::CollisionLayers;
use kinetica::prelude::*;
use kinetica::shape::Capsule;
use kinetica_macroquad::DebugRenderer;

use super::queries::{FOUND_COLOR, NORMAL_COLOR, PROBE_COLOR, add_targets};
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Shape cast",
    category: Category::Queries,
    description: "A shape cast moves a shape in a straight line and tells where it first touches something. The \
                  capsule is cast towards the mouse.",
    camera_center: vec2(0.0, 5.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

const ORIGIN: Vec2 = vec2(-7.0, 10.0);
const HALF_LENGTH: Vec2 = vec2(0.5, 0.0);
const RADIUS: f32 = 0.3;

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_targets(world, CollisionLayers::default());
    Box::new(Sweep)
}

struct Sweep;

impl SceneUpdate for Sweep {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        let capsule = Capsule::new(-HALF_LENGTH, HALF_LENGTH, RADIUS);
        let translation = frame.mouse - ORIGIN;
        drawer.draw_capsule(ORIGIN - HALF_LENGTH, ORIGIN + HALF_LENGTH, RADIUS, PROBE_COLOR);
        let Some(hit) = world.cast_shape(capsule, ORIGIN, translation, QueryFilter::new()) else {
            drawer.draw_segment(ORIGIN, frame.mouse, PROBE_COLOR);
            return;
        };
        let stop = ORIGIN + hit.fraction * translation;
        drawer.draw_segment(ORIGIN, stop, PROBE_COLOR);
        drawer.draw_capsule(stop - HALF_LENGTH, stop + HALF_LENGTH, RADIUS, FOUND_COLOR);
        drawer.draw_collider(world, hit.collider, FOUND_COLOR);
        drawer.draw_segment(hit.point, hit.point + 0.5 * hit.normal, NORMAL_COLOR);
    }
}
