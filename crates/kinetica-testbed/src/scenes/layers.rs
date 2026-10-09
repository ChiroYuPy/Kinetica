use kinetica::CollisionLayers;
use kinetica::prelude::*;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Collision layers",
    category: Category::GameFeatures,
    description: "Each shelf only stops balls of its color, and balls only collide with balls of the same \
                  color: they sort themselves out.",
    camera_center: vec2(0.0, 5.5),
    view_height: 13.0,
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
const BALL_COUNT: usize = 150;
const SPAWN_PERIOD: f32 = 0.08;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let frame = world.add_body(BodyDesc::fixed());
    let solid = CollisionLayers::new(WORLD, u32::MAX);
    for (width, height, center) in
        [(13.0, 0.5, vec2(0.0, -0.25)), (0.5, 12.0, vec2(-6.25, 6.0)), (0.5, 12.0, vec2(6.25, 6.0))]
    {
        world.add_collider(frame, ColliderDesc::rectangle(width, height).offset(center).layers(solid));
    }

    // Each shelf belongs to the world layer but only collides with one color.
    let mut shelves = Vec::new();
    for (i, &(layer, color)) in COLORS[..2].iter().enumerate() {
        let y = 7.0 - 3.5 * i as f32;
        let angle = if i % 2 == 0 { -0.08 } else { 0.08 };
        let shelf = ColliderDesc::rectangle(12.0, 0.24)
            .offset(vec2(0.0, y))
            .angle(angle)
            .layers(CollisionLayers::new(WORLD, layer));
        shelves.push((world.add_collider(frame, shelf), color));
    }
    labels.at(vec2(0.0, 7.6), "Red shelf: collides with red only");
    labels.at(vec2(0.0, 4.1), "Green shelf: collides with green only");
    labels.at(vec2(0.0, 11.2), "Balls collide with their own color and the walls");
    Box::new(Sorter { shelves, balls: Vec::new(), next_spawn: 0.0 })
}

struct Sorter {
    shelves: Vec<(ColliderHandle, Color)>,
    balls: Vec<(BodyHandle, Color)>,
    next_spawn: f32,
}

impl SceneUpdate for Sorter {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if self.balls.len() >= BALL_COUNT || frame.time < self.next_spawn {
            return;
        }
        self.next_spawn += SPAWN_PERIOD;
        let i = self.balls.len();
        let (layer, color) = COLORS[i % COLORS.len()];
        // Golden ratio steps scatter the balls evenly across the width, never twice at one spot.
        let x = 10.0 * (i as f32 * 0.618_034).fract() - 5.0;
        let shape = ColliderDesc::circle(0.22).restitution(0.2).layers(CollisionLayers::new(layer, layer | WORLD));
        let ball = world.add_body(BodyDesc::dynamic().position(vec2(x, 11.5)).collider(shape));
        self.balls.push((ball, color));
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for &(shelf, color) in &self.shelves {
            drawer.draw_collider(world, shelf, color);
        }
        for &(ball, color) in &self.balls {
            drawer.draw_body(world, ball, color);
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
        for step in 0..30 * 60 {
            let frame = Frame { time: step as f32 / 60.0, ..Frame::default() };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
        }
        // Shelves at y = 7 (red) and 3.5 (green), tilted by at most 0.08 rad over 6 m.
        let floors = [(RED, 7.0 - 0.6, 12.0), (GREEN, 3.5 - 0.6, 7.0 - 0.6), (BLUE, 0.0, 3.5 - 0.6)];
        for (_, body) in world.bodies() {
            let Some(&collider) = body.colliders().first() else { continue };
            let layers = world[collider].layers();
            let Some(&(_, low, high)) = floors.iter().find(|(layer, ..)| layers.memberships == *layer) else {
                continue;
            };
            let y = body.position().y;
            assert!(low < y && y < high, "a ball of layer {} is at y = {y}", layers.memberships);
        }
    }
}
