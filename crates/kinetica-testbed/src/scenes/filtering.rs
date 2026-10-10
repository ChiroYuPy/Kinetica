use std::collections::VecDeque;

use kinetica::CollisionLayers;
use kinetica::prelude::*;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Collision filtering",
    category: Category::GameFeatures,
    description: "Two ways to choose what collides. Left, collision layers: each shelf only stops balls of its \
                  color, and balls only collide with balls of the same color, so they sort themselves out. Right, \
                  one-way colliders only stop what comes from one side: balls shot up go through the platforms \
                  and land on them, and the wall only blocks from the left (drag a ball through it both ways). \
                  The decision is made when contact starts, so a body halfway through keeps going.",
    camera_center: vec2(0.5, 6.5),
    view_height: 17.0,
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
/// The sorting box: its middle and inside width.
const BOX_CENTER: f32 = -8.0;
const BOX_WIDTH: f32 = 7.0;
const BALL_COUNT: usize = 90;
const SPAWN_PERIOD: f32 = 0.1;
const SHOT_PERIOD: f32 = 0.5;
const MAX_SHOTS: usize = 30;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 60.0);
    let level = world.add_body(BodyDesc::fixed());

    // The sorting box; its walls belong to the world layer and stop every ball.
    let solid = CollisionLayers::new(WORLD, u32::MAX);
    for x in [BOX_CENTER - BOX_WIDTH / 2.0, BOX_CENTER + BOX_WIDTH / 2.0] {
        world.add_collider(level, ColliderDesc::rectangle(0.5, 12.0).offset(vec2(x, 6.0)).layers(solid));
    }
    // Each shelf belongs to the world layer but only collides with one color.
    let mut shelves = Vec::new();
    for (i, &(layer, color)) in COLORS[..2].iter().enumerate() {
        let y = 7.0 - 3.5 * i as f32;
        let angle = if i % 2 == 0 { -0.08 } else { 0.08 };
        let shelf = ColliderDesc::rectangle(BOX_WIDTH - 0.5, 0.24)
            .offset(vec2(BOX_CENTER, y))
            .angle(angle)
            .layers(CollisionLayers::new(WORLD, layer));
        shelves.push((world.add_collider(level, shelf), color));
    }
    labels.at(vec2(BOX_CENTER, 7.6), "Red shelf: red only");
    labels.at(vec2(BOX_CENTER, 4.1), "Green shelf: green only");
    labels.at(vec2(BOX_CENTER, 12.6), "Balls collide with their\nown color and the walls");

    // One-way platforms, blocking from above.
    for (center, width, angle) in [
        (vec2(-0.5, 3.0), 5.0, 0.0),
        (vec2(4.0, 5.0), 4.0, 0.0),
        (vec2(0.5, 7.5), 4.0, 0.2),
        (vec2(4.5, 9.5), 6.0, 0.0),
    ] {
        world.add_collider(level, ColliderDesc::rectangle(width, 0.2).offset(center).angle(angle).one_way(Vec2::Y));
    }
    labels.at(vec2(-0.5, 2.2), "Blocks from above only");
    // A wall that only blocks things coming from its left.
    world.add_collider(level, ColliderDesc::rectangle(0.2, 5.0).offset(vec2(10.0, 2.5)).one_way(-Vec2::X));
    labels.at(vec2(10.0, 5.6), "Blocks from\nthe left only");
    world.add_collider(level, ColliderDesc::rectangle(1.0, 20.0).offset(vec2(13.0, 10.0)));

    Box::new(Filtering { shelves, sorted: Vec::new(), shot: VecDeque::new(), shots: 0 })
}

struct Filtering {
    shelves: Vec<(ColliderHandle, Color)>,
    /// Balls dropped in the sorting box, with their color.
    sorted: Vec<(BodyHandle, Color)>,
    /// Balls shot up through the platforms, oldest first.
    shot: VecDeque<BodyHandle>,
    shots: usize,
}

impl Filtering {
    fn drop_colored_ball(&mut self, world: &mut World, frame: &Frame) {
        if self.sorted.len() >= BALL_COUNT || frame.time < SPAWN_PERIOD * self.sorted.len() as f32 {
            return;
        }
        let i = self.sorted.len();
        let (layer, color) = COLORS[i % COLORS.len()];
        // Golden ratio steps scatter the balls evenly across the width, never twice at one spot.
        let x = BOX_CENTER + (BOX_WIDTH - 1.5) * ((i as f32 * 0.618_034).fract() - 0.5);
        let shape = ColliderDesc::circle(0.22).restitution(0.2).layers(CollisionLayers::new(layer, layer | WORLD));
        let ball = world.add_body(BodyDesc::dynamic().position(vec2(x, 11.5)).collider(shape));
        self.sorted.push((ball, color));
    }

    fn shoot_ball(&mut self, world: &mut World, frame: &Frame) {
        if frame.time < SHOT_PERIOD * self.shots as f32 {
            return;
        }
        let scatter = (self.shots as f32 * 0.618_034).fract();
        self.shots += 1;
        let position = vec2(-2.5 + 9.0 * scatter, 0.5);
        let velocity = vec2(1.0 - 2.0 * scatter, 10.0 + 4.0 * scatter);
        let ball = BodyDesc::dynamic().position(position).linear_velocity(velocity).collider(ColliderDesc::circle(0.3));
        self.shot.push_back(world.add_body(ball));
        if self.shot.len() > MAX_SHOTS
            && let Some(oldest) = self.shot.pop_front()
        {
            world.remove_body(oldest);
        }
    }
}

impl SceneUpdate for Filtering {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        self.drop_colored_ball(world, frame);
        self.shoot_ball(world, frame);
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for &(shelf, color) in &self.shelves {
            drawer.draw_collider(world, shelf, color);
        }
        for &(ball, color) in &self.sorted {
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
        // Shelves at y = 7 (red) and 3.5 (green), tilted by at most 0.08 rad over 3.5 m.
        let floors = [(RED, 7.0 - 0.4, 12.0), (GREEN, 3.5 - 0.4, 7.0 - 0.4), (BLUE, 0.0, 3.5 - 0.4)];
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
