use std::collections::VecDeque;

use kinetica::prelude::*;

use super::{Build, Category, Frame, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Sleeping",
    category: Category::GameFeatures,
    description: "Resting bodies fall asleep (drawn darker) and cost nothing. Every few seconds a ball drops \
                  on one stack and wakes only that stack: each group of touching bodies sleeps on its own.",
    camera_center: vec2(0.0, 5.0),
    view_height: 16.0,
    build: Build::Animated(build),
};

const STACK_XS: [f32; 5] = [-8.0, -4.0, 0.0, 4.0, 8.0];
const DROP_PERIOD: f32 = 2.5;
/// Old balls are removed, which wakes what they rested on.
const MAX_BALLS: usize = 4;

fn build(world: &mut World) -> Box<dyn SceneUpdate> {
    add_ground(world, 20.0);
    for x in STACK_XS {
        for level in 0..6 {
            let block = world.add_body(BodyDesc::dynamic().position(vec2(x, 0.4 + 0.8 * level as f32)));
            world.add_collider(block, ColliderDesc::rectangle(0.6, 0.4));
        }
    }
    Box::new(Dropper { next_drop: DROP_PERIOD, next_stack: 0, balls: VecDeque::new() })
}

struct Dropper {
    next_drop: f32,
    next_stack: usize,
    balls: VecDeque<BodyHandle>,
}

impl SceneUpdate for Dropper {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if frame.time < self.next_drop {
            return;
        }
        self.next_drop += DROP_PERIOD;
        let x = STACK_XS[self.next_stack] + 0.3;
        // Visit the stacks in a scattered order.
        self.next_stack = (self.next_stack + 2) % STACK_XS.len();

        let ball = world.add_body(BodyDesc::dynamic().position(vec2(x, 12.0)));
        world.add_collider(ball, ColliderDesc::circle(0.35).density(3.0));
        self.balls.push_back(ball);
        if self.balls.len() > MAX_BALLS
            && let Some(oldest) = self.balls.pop_front()
        {
            world.remove_body(oldest);
        }
    }
}
