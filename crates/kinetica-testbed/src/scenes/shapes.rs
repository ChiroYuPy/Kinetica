use std::collections::VecDeque;
use std::f32::consts::TAU;

use kinetica::prelude::*;

use super::{Build, Category, Frame, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Capsules and chains",
    category: Category::Contacts,
    description: "Capsules, circles and boxes rain on wavy terrain made of a single chain, bounce off a rock \
                  (a closed chain), slide down thin segments and fill a U-shaped chain bin. Chains collide on one \
                  side only: the ticks show which.",
    camera_center: vec2(0.0, 6.0),
    view_height: 17.0,
    build: Build::Animated(build),
};

const SPAWN_PERIOD: f32 = 0.25;
const MAX_BODIES: usize = 90;

fn build(world: &mut World) -> Box<dyn SceneUpdate> {
    let level = world.add_body(BodyDesc::fixed());

    // Wavy ground between two walls, listed left to right so it faces up.
    let mut ground = vec![vec2(-14.0, 8.0)];
    ground.extend((0..=56).map(|i| {
        let x = -14.0 + 0.5 * i as f32;
        vec2(x, 0.5 * (0.8 * x).sin())
    }));
    ground.push(vec2(14.0, 8.0));
    world.add_chain(level, &ChainDesc::new(ground).friction(0.7));

    // A rock: listed clockwise, so it is solid from the outside.
    let rock: Vec<Vec2> = (0..9)
        .map(|i| {
            let angle = -TAU * i as f32 / 9.0;
            let radius = 1.2 + 0.35 * (3.0 * angle).cos();
            vec2(-5.0, 6.0) + radius * Vec2::from_angle(angle)
        })
        .collect();
    world.add_chain(level, &ChainDesc::new(rock).closed(true));

    // Thin two-sided shelves.
    world.add_collider(level, ColliderDesc::segment(vec2(0.0, 8.0), vec2(4.0, 7.0)));
    world.add_collider(level, ColliderDesc::segment(vec2(-1.0, 4.5), vec2(-4.0, 3.5)));

    // A bin: down the left side, along the bottom, up the right side, so it faces inward.
    let bin = [vec2(7.0, 6.0), vec2(7.0, 2.5), vec2(11.0, 2.5), vec2(11.0, 6.0)];
    world.add_chain(level, &ChainDesc::new(bin));

    Box::new(Rain { bodies: VecDeque::new(), spawned: 0, next_spawn: 0.0 })
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
        let body =
            world.add_body(BodyDesc::dynamic().position(vec2(-10.0 + 20.0 * scatter, 15.0)).angle(3.0 * scatter));
        let size = 0.25 + 0.2 * ((i as f32 * 0.414_214).fract());
        let collider = match i % 4 {
            0 | 1 => ColliderDesc::capsule(vec2(-size, 0.0), vec2(size, 0.0), 0.5 * size),
            2 => ColliderDesc::circle(size),
            _ => ColliderDesc::rectangle(size, 0.7 * size),
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
        let mut scene = build(&mut world);
        for step in 0..30 * 60 {
            let frame = Frame { time: step as f32 / 60.0, mouse: Vec2::ZERO };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
        }
        for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            let position = body.position();
            assert!(position.x.abs() < 14.0 && position.y > -0.8, "a body escaped to {position}");
        }
    }
}
