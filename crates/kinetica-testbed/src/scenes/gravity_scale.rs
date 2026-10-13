use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Gravity scale",
    category: Category::Bodies,
    description: "Each body can scale gravity for itself: 0 floats, a negative scale rises. Damping slows a body \
                  down, like air drag.",
    camera_center: vec2(0.0, 4.5),
    view_height: 12.0,
    build: Build::Animated(build),
};

const START_HEIGHT: f32 = 5.0;
/// The balls are released again this often, in seconds.
const PERIOD: f32 = 5.0;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 20.0);
    world.add_body(BodyDesc::fixed().position(vec2(0.0, 9.25)).collider(ColliderDesc::rectangle(20.0, 0.5)));

    let ball = |x: f32| BodyDesc::dynamic().position(vec2(x, START_HEIGHT)).collider(ColliderDesc::circle(0.5));
    let balls = [
        (ball(-6.0), "gravity × 1"),
        (ball(-2.0).gravity_scale(0.0), "gravity × 0"),
        (ball(2.0).gravity_scale(-0.3), "gravity × −0.3"),
        (ball(6.0).linear_damping(4.0), "damping 4"),
    ]
    .map(|(desc, text)| {
        let body = world.add_body(desc);
        labels.above(body, text);
        body
    });

    let mut releases = 1;
    Box::new(move |world: &mut World, frame: &Frame| {
        if frame.time >= PERIOD * releases as f32 {
            releases += 1;
            for ball in balls {
                let x = world[ball].position().x;
                world[ball].set_position(vec2(x, START_HEIGHT));
                world[ball].set_linear_velocity(Vec2::ZERO);
            }
        }
    })
}
