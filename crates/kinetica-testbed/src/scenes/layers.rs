use kinetica::CollisionLayers;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, Spawner, add_ground, scatter};

pub const SCENE: Scene = Scene {
    name: "Collision layers",
    category: Category::GameFeatures,
    description: "Each collider says which layers it is in and which it collides with. Balls only collide with \
                  their own color, each shelf with one: they sort themselves.",
    camera_center: vec2(0.0, 6.0),
    view_height: 14.0,
    build: Build::Animated(build),
};

/// Walls, ground and shelves.
const WORLD: u32 = 1 << 0;
const RED: u32 = 1 << 1;
const GREEN: u32 = 1 << 2;
const BLUE: u32 = 1 << 3;
const COLORS: [(u32, Color); 3] = [
    (RED, Color::new(0.95, 0.35, 0.35, 1.0)),
    (GREEN, Color::new(0.4, 0.9, 0.45, 1.0)),
    (BLUE, Color::new(0.4, 0.6, 1.0, 1.0)),
];
/// Inside width of the box.
const WIDTH: f32 = 7.0;
const BALL_COUNT: usize = 120;

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 20.0);
    let level = world.add_body(BodyDesc::fixed());
    for x in [-WIDTH / 2.0, WIDTH / 2.0] {
        world.add_collider(level, ColliderDesc::rectangle(0.5, 12.0).offset(vec2(x, 6.0)));
    }
    // The red shelf stops red balls only, the green shelf green ones; blue ones reach the ground.
    let mut shelves = Vec::new();
    for (i, &(layer, color)) in COLORS[..2].iter().enumerate() {
        let shelf = ColliderDesc::rectangle(WIDTH - 0.5, 0.24)
            .offset(vec2(0.0, 7.5 - 3.5 * i as f32))
            .layers(CollisionLayers::new(WORLD, layer));
        shelves.push((world.add_collider(level, shelf), color));
    }
    Box::new(Sorter { shelves, balls: Spawner::new(0.1, BALL_COUNT) })
}

struct Sorter {
    shelves: Vec<(ColliderHandle, Color)>,
    balls: Spawner,
}

fn color_of(i: usize) -> (u32, Color) {
    COLORS[i % COLORS.len()]
}

impl SceneUpdate for Sorter {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        self.balls.update(world, frame.time, |i| {
            let layer = color_of(i).0;
            let shape = ColliderDesc::circle(0.22).restitution(0.2).layers(CollisionLayers::new(layer, layer | WORLD));
            BodyDesc::dynamic().position(vec2((WIDTH - 1.5) * (scatter(i) - 0.5), 12.0)).collider(shape)
        });
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for &(shelf, color) in &self.shelves {
            drawer.draw_collider(world, shelf, color);
        }
        for (i, ball) in self.balls.bodies() {
            drawer.draw_body(world, ball, color_of(i).1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn balls_end_up_sorted_by_color() {
        let mut world = World::new();
        let mut scene = build(&mut world, &mut Labels::default());
        for step in 0..16 * 60 {
            scene.update(&mut world, &Frame { time: step as f32 / 60.0, ..Frame::default() });
            world.step(1.0 / 60.0);
        }
        // Shelves at y = 7.5 (red) and 4 (green).
        let floors = [(RED, 7.5, 13.0), (GREEN, 4.0, 7.5), (BLUE, 0.0, 4.0)];
        // Balls keep raining: only those which landed are looked at.
        for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            if body.linear_velocity().length() > 0.5 {
                continue;
            }
            let layers = world[body.colliders()[0]].layers();
            let &(_, low, high) = floors.iter().find(|(layer, ..)| layers.memberships == *layer).unwrap();
            let y = body.position().y;
            assert!(low < y && y < high, "a ball of layer {} is at y = {y}", layers.memberships);
        }
    }
}
