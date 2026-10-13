use std::collections::VecDeque;

use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Sleeping",
    category: Category::GameFeatures,
    description: "Bodies at rest fall asleep, drawn darker, and cost nothing until something touches them. Each \
                  stack sleeps and wakes on its own.",
    camera_center: vec2(0.0, 5.0),
    view_height: 16.0,
    build: Build::Animated(build),
};

const STACK_XS: [f32; 5] = [-8.0, -4.0, 0.0, 4.0, 8.0];
const DROP_PERIOD: f32 = 2.5;
/// Old balls are removed, which wakes what they rested on.
const MAX_BALLS: usize = 4;

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    let stacks = STACK_XS.map(|x| {
        (0..6)
            .map(|level| {
                let position = vec2(x, 0.4 + 0.8 * level as f32);
                world.add_body(BodyDesc::dynamic().position(position).collider(ColliderDesc::rectangle(1.2, 0.8)))
            })
            .collect()
    });
    Box::new(Dropper { next_drop: DROP_PERIOD, next_stack: 0, balls: VecDeque::new(), stacks })
}

struct Dropper {
    next_drop: f32,
    next_stack: usize,
    balls: VecDeque<BodyHandle>,
    stacks: [Vec<BodyHandle>; STACK_XS.len()],
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

        let ball = BodyDesc::dynamic().position(vec2(x, 12.0)).collider(ColliderDesc::circle(0.35).density(3.0));
        self.balls.push_back(world.add_body(ball));
        if self.balls.len() > MAX_BALLS
            && let Some(oldest) = self.balls.pop_front()
        {
            world.remove_body(oldest);
        }
    }

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        for (x, stack) in STACK_XS.into_iter().zip(&self.stacks) {
            let asleep = stack.iter().all(|&block| world[block].is_sleeping());
            labels.at(vec2(x, 5.4), if asleep { "asleep" } else { "awake" });
        }
    }
}
