use kinetica::CollisionLayers;
use kinetica::math::{Aabb, Rot};
use kinetica::prelude::*;
use kinetica::shape::Circle;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::queries::{FOUND_COLOR, PROBE_COLOR, add_targets};
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Overlap queries",
    category: Category::Queries,
    description: "Which colliders are here? Keys 1 to 3 pick what the mouse carries: a point, a circle, or a box, \
                  which only compares bounding boxes: cheaper, but approximate.",
    camera_center: vec2(0.0, 5.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

const RADIUS: f32 = 1.2;
const BOX_SIZE: Vec2 = vec2(4.0, 2.4);
const BOUNDS_COLOR: Color = Color::new(0.9, 0.3, 0.9, 1.0);

#[derive(Debug, Clone, Copy)]
enum Probe {
    Point,
    Circle,
    Box,
}

const KEYS: [(KeyCode, Probe); 3] =
    [(KeyCode::Key1, Probe::Point), (KeyCode::Key2, Probe::Circle), (KeyCode::Key3, Probe::Box)];

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_targets(world, CollisionLayers::default());
    Box::new(Overlaps { probe: Probe::Circle })
}

struct Overlaps {
    probe: Probe,
}

impl SceneUpdate for Overlaps {
    fn update(&mut self, _world: &mut World, frame: &Frame) {
        if let Some(&(_, probe)) = KEYS.iter().find(|(key, _)| frame.is_pressed(*key)) {
            self.probe = probe;
        }
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        let (mouse, filter) = (frame.mouse, QueryFilter::new());
        let found = match self.probe {
            Probe::Point => {
                drawer.draw_point(mouse, PROBE_COLOR);
                world.colliders_at_point(mouse, filter)
            }
            Probe::Circle => {
                drawer.draw_circle(mouse, RADIUS, Rot::IDENTITY, PROBE_COLOR);
                world.colliders_overlapping(Circle::new(RADIUS), mouse, filter)
            }
            Probe::Box => {
                let area = Aabb::new(mouse - BOX_SIZE / 2.0, mouse + BOX_SIZE / 2.0);
                drawer.draw_aabb(&area, PROBE_COLOR);
                let found = world.colliders_in_aabb(&area, filter);
                for &collider in &found {
                    drawer.draw_aabb(&world[collider].aabb(), BOUNDS_COLOR);
                }
                found
            }
        };
        for collider in found {
            drawer.draw_collider(world, collider, FOUND_COLOR);
        }
    }

    fn labels(&self, _world: &World, _frame: &Frame, labels: &mut Labels) {
        let text = match self.probe {
            Probe::Point => "1: colliders_at_point",
            Probe::Circle => "2: colliders_overlapping",
            Probe::Box => "3: colliders_in_aabb",
        };
        labels.at(vec2(0.0, 11.0), text);
    }
}
