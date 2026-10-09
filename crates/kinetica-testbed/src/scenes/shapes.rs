use std::collections::VecDeque;
use std::f32::consts::TAU;

use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Shapes",
    category: Category::Collisions,
    description: "Every collider shape. Left, one of each kind of solid shape. Right, they rain on the ground \
                  shapes for levels: a chain (a line of segments, here wavy ground, a rock and a bin) collides on \
                  one side only, shown by its ticks; a segment is a two-sided line without thickness.",
    camera_center: vec2(0.0, 5.5),
    view_height: 20.5,
    build: Build::Animated(build),
};

const SPAWN_PERIOD: f32 = 0.25;
const MAX_BODIES: usize = 90;
/// The ground is flat left of this, where the shapes are on display, and wavy right of it.
const GALLERY_END: f32 = -2.0;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let level = world.add_body(BodyDesc::fixed());

    // The ground between two walls, listed left to right so it faces up.
    let mut ground = vec![vec2(-14.0, 8.0)];
    ground.extend((0..=56).map(|i| {
        let x = -14.0 + 0.5 * i as f32;
        let height = if x <= GALLERY_END { 0.0 } else { 0.5 * (0.8 * (x - GALLERY_END)).sin() };
        vec2(x, height)
    }));
    ground.push(vec2(14.0, 8.0));
    world.add_chain(level, &ChainDesc::new(ground).friction(0.7));
    labels.at(vec2(6.0, -1.3), "Chain: solid on the side of its ticks");

    // The display, behind a low wall that keeps the rain out.
    world.add_collider(level, ColliderDesc::segment(vec2(GALLERY_END - 0.5, 0.0), vec2(GALLERY_END - 0.5, 3.0)));
    let gallery = [
        (ColliderDesc::circle(0.5), "circle"),
        (ColliderDesc::rectangle(0.6, 0.4), "rectangle"),
        (ColliderDesc::regular_polygon(5, 0.55), "regular\npolygon"),
        (hull(&[vec2(-0.6, -0.4), vec2(0.7, -0.4), vec2(0.3, 0.5), vec2(-0.3, 0.6)]), "convex\nhull"),
        (ColliderDesc::capsule(vec2(-0.5, 0.0), vec2(0.5, 0.0), 0.3), "capsule"),
    ];
    for (i, (collider, name)) in gallery.into_iter().enumerate() {
        let bottom = -collider.shape.compute_aabb(&Transform::IDENTITY).min.y;
        let body = world.add_body(BodyDesc::dynamic().position(vec2(-12.5 + 2.2 * i as f32, bottom)));
        world.add_collider(body, collider);
        labels.above(body, name);
    }

    // A rock: listed clockwise, so it is solid from the outside.
    let rock: Vec<Vec2> = (0..9)
        .map(|i| {
            let angle = -TAU * i as f32 / 9.0;
            let radius = 1.2 + 0.35 * (3.0 * angle).cos();
            vec2(2.5, 6.0) + radius * Vec2::from_angle(angle)
        })
        .collect();
    world.add_chain(level, &ChainDesc::new(rock).closed(true));
    labels.at(vec2(2.5, 8.2), "Closed chain");

    // A thin two-sided shelf.
    world.add_collider(level, ColliderDesc::segment(vec2(-1.5, 9.0), vec2(0.5, 8.0)));
    labels.at(vec2(-1.5, 9.4), "Segment");

    // A bin: down the left side, along the bottom, up the right side, so it faces inward.
    let bin = [vec2(7.0, 6.0), vec2(7.0, 2.5), vec2(11.0, 2.5), vec2(11.0, 6.0)];
    world.add_chain(level, &ChainDesc::new(bin));
    labels.at(vec2(9.0, 6.6), "Open chain");

    Box::new(Rain { bodies: VecDeque::new(), spawned: 0, next_spawn: 0.0 })
}

fn hull(points: &[Vec2]) -> ColliderDesc {
    ColliderDesc::convex_hull(points).expect("the points enclose an area")
}

struct Rain {
    bodies: VecDeque<BodyHandle>,
    spawned: usize,
    next_spawn: f32,
}

impl SceneUpdate for Rain {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if frame.time < self.next_spawn {
            return;
        }
        self.next_spawn += SPAWN_PERIOD;
        let i = self.spawned;
        self.spawned += 1;

        let scatter = (i as f32 * 0.618_034).fract();
        let body = world.add_body(BodyDesc::dynamic().position(vec2(-1.0 + 14.0 * scatter, 15.0)).angle(3.0 * scatter));
        let size = 0.25 + 0.2 * ((i as f32 * 0.414_214).fract());
        let collider = match i % 5 {
            0 => ColliderDesc::capsule(vec2(-size, 0.0), vec2(size, 0.0), 0.5 * size),
            1 => ColliderDesc::circle(size),
            2 => ColliderDesc::rectangle(size, 0.7 * size),
            3 => ColliderDesc::regular_polygon(3 + i % 4, size),
            _ => hull(&[vec2(-size, -0.5 * size), vec2(size, -0.5 * size), vec2(0.3 * size, 0.8 * size)]),
        };
        world.add_collider(body, collider);
        self.bodies.push_back(body);
        if self.bodies.len() > MAX_BODIES
            && let Some(oldest) = self.bodies.pop_front()
        {
            world.remove_body(oldest);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_falls_through_the_level() {
        let mut world = World::new();
        let mut scene = build(&mut world, &mut Labels::default());
        for step in 0..30 * 60 {
            let frame = Frame { time: step as f32 / 60.0, ..Frame::default() };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
        }
        for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            let position = body.position();
            assert!(position.x.abs() < 14.0 && position.y > -0.8, "a body escaped to {position}");
        }
    }
}
