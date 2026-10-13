use std::collections::VecDeque;

use kinetica::prelude::*;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Bullets",
    category: Category::GameFeatures,
    description: "At 150 m/s a ball moves 2.5 m per step, and would jump over thin things. Fixed bodies always \
                  stop it; a bullet is also stopped by moving bodies.",
    camera_center: vec2(-1.2, 3.0),
    view_height: 14.0,
    build: Build::Animated(build),
};

const CANNON: Vec2 = vec2(-10.0, 2.0);
const SPEED: f32 = 150.0;
const FIRE_PERIOD: f32 = 0.5;
const MAX_BALLS: usize = 12;
const BULLET_COLOR: Color = Color::new(0.95, 0.3, 0.3, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    world.add_body(BodyDesc::fixed().position(vec2(8.0, 3.0)).collider(ColliderDesc::rectangle(0.1, 6.0)));

    // Thin plates hanging from hinges: they swing when hit, then fall back into place.
    for x in [-2.0, 1.0, 4.0] {
        let hinge = vec2(x, 4.0);
        let support = world.add_body(BodyDesc::fixed().position(hinge));
        let plate = world.add_body(
            BodyDesc::dynamic()
                .position(hinge - vec2(0.0, 1.5))
                .angular_damping(0.5)
                .collider(ColliderDesc::rectangle(0.08, 3.0).density(5.0)),
        );
        world.add_joint(RevoluteJointDesc::new(support, plate, hinge));
    }
    labels.at(vec2(1.0, 4.4), "Thin plates: only bullets, in red, hit them");
    Box::new(Cannon { next_shot: 0.5, shots: 0, balls: VecDeque::new() })
}

struct Cannon {
    next_shot: f32,
    shots: usize,
    balls: VecDeque<(BodyHandle, bool)>,
}

impl SceneUpdate for Cannon {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if frame.time < self.next_shot {
            return;
        }
        self.next_shot += FIRE_PERIOD;
        let bullet = self.shots % 2 == 1;
        self.shots += 1;
        let ball =
            BodyDesc::dynamic().position(CANNON).linear_velocity(vec2(SPEED, 1.0)).collider(ColliderDesc::circle(0.12));
        let ball = world.add_body(if bullet { ball.bullet() } else { ball });
        self.balls.push_back((ball, bullet));
        if self.balls.len() > MAX_BALLS
            && let Some((oldest, _)) = self.balls.pop_front()
        {
            world.remove_body(oldest);
        }
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        drawer.draw_polygon(
            &[CANNON + vec2(-0.8, -0.3), CANNON + vec2(0.0, -0.2), CANNON + vec2(0.0, 0.2), CANNON + vec2(-0.8, 0.3)],
            Color::new(0.7, 0.7, 0.75, 1.0),
        );
        for &(ball, bullet) in &self.balls {
            if bullet {
                drawer.draw_body(world, ball, BULLET_COLOR);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_passes_the_wall() {
        let mut world = World::new();
        let mut scene = build(&mut world, &mut Labels::default());
        for step in 0..10 * 60 {
            let frame = Frame { time: step as f32 / 60.0, ..Frame::default() };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
            for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
                assert!(body.position().x < 8.0, "a ball went through the wall: {}", body.position());
            }
        }
    }
}
