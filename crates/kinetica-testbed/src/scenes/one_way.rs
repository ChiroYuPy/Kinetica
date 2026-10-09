use std::collections::VecDeque;

use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "One-way platforms",
    category: Category::GameFeatures,
    description: "A one-way collider only stops what comes from one side. Balls are shot up through the platforms \
                  and land on top of them. The wall on the right only blocks from the left: drag a ball \
                  through it both ways. The decision is made when contact starts, so a body halfway through keeps \
                  going.",
    camera_center: vec2(0.0, 5.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

const SHOT_PERIOD: f32 = 0.5;
const MAX_BALLS: usize = 40;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    let level = world.add_body(BodyDesc::fixed());
    for (center, width, angle) in [
        (vec2(-4.0, 3.0), 5.0, 0.0),
        (vec2(0.5, 5.0), 4.0, 0.0),
        (vec2(-3.0, 7.5), 4.0, 0.2),
        (vec2(1.0, 9.5), 6.0, 0.0),
    ] {
        world.add_collider(level, ColliderDesc::rectangle(width, 0.2).offset(center).angle(angle).one_way(Vec2::Y));
    }
    labels.at(vec2(-4.0, 2.2), "Blocks from above only");

    // A wall that only blocks things coming from its left.
    world.add_collider(level, ColliderDesc::rectangle(0.2, 5.0).offset(vec2(6.5, 2.5)).one_way(-Vec2::X));
    labels.at(vec2(6.5, 5.6), "Blocks from the left only");
    for x in [-10.0, 10.0] {
        world.add_collider(level, ColliderDesc::rectangle(1.0, 20.0).offset(vec2(x, 10.0)));
    }

    Box::new(Launcher { balls: VecDeque::new(), shots: 0 })
}

struct Launcher {
    balls: VecDeque<BodyHandle>,
    shots: usize,
}

impl SceneUpdate for Launcher {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if frame.time < SHOT_PERIOD * self.shots as f32 {
            return;
        }
        let scatter = (self.shots as f32 * 0.618_034).fract();
        self.shots += 1;
        let position = vec2(-6.0 + 9.0 * scatter, 0.5);
        let velocity = vec2(1.0 - 2.0 * scatter, 10.0 + 4.0 * scatter);
        let ball = BodyDesc::dynamic()
            .position(position)
            .linear_velocity(velocity)
            .collider(ColliderDesc::circle(0.3).friction(0.6));
        let ball = world.add_body(ball);
        self.balls.push_back(ball);
        if self.balls.len() > MAX_BALLS
            && let Some(oldest) = self.balls.pop_front()
        {
            world.remove_body(oldest);
        }
    }
}
