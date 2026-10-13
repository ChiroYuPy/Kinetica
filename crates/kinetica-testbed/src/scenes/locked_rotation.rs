use kinetica::math::Rot;
use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Locked rotation",
    category: Category::Bodies,
    description: "A body whose rotation is locked never turns, whatever hits it: a character stays upright on a \
                  slope where a free body falls over.",
    camera_center: vec2(0.0, 3.5),
    view_height: 11.0,
    build: Build::Static(build),
};

const RAMP_ANGLE: f32 = -0.5;
const RAMP_LENGTH: f32 = 6.0;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);
    // The same character at the top of two ramps.
    for (x, locked, text) in [(-5.5, false, "Free"), (2.5, true, "Rotation locked")] {
        let center = vec2(x, 2.2);
        let ramp = ColliderDesc::rectangle(RAMP_LENGTH, 0.4);
        world.add_body(BodyDesc::fixed().position(center).angle(RAMP_ANGLE).collider(ramp));
        // A stop at the bottom.
        world.add_body(BodyDesc::fixed().position(vec2(x + 4.8, 0.75)).collider(ColliderDesc::rectangle(0.4, 1.5)));

        let top = center - (RAMP_LENGTH / 2.0 - 1.0) * Rot::from_angle(RAMP_ANGLE).rotate(Vec2::X);
        let character = BodyDesc::dynamic()
            .position(top + vec2(0.0, 1.1))
            .collider(ColliderDesc::vertical_capsule(1.4, 0.3).friction(0.3));
        let character = world.add_body(if locked { character.lock_rotation() } else { character });
        labels.above(character, text);
    }
}
