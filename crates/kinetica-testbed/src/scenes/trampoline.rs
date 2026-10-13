use kinetica::prelude::*;

use super::rope::{add_beads, tie};
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, Spawner, add_ground, scatter};

pub const SCENE: Scene = Scene {
    name: "Trampoline",
    category: Category::SoftStructures,
    description: "The same line of beads, tied by springs instead of ropes and stretched between two posts, \
                  makes an elastic band.",
    camera_center: vec2(0.0, 4.5),
    view_height: 11.0,
    build: Build::Animated(build),
};

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let ground = add_ground(world, 40.0);
    // The springs are shorter than the gaps between beads, so the band is taut.
    let elastic = |desc: DistanceJointDesc, length: f32| desc.spring(15.0, 0.3).length(0.85 * length);
    let (left, right) = (vec2(-3.5, 3.0), vec2(3.5, 3.0));
    let band = add_beads(world, ground, left, right - vec2(0.25, 0.0), 27, elastic);
    tie(world, band[band.len() - 1], ground, right, elastic);
    for post in [left, right] {
        world.add_collider(ground, ColliderDesc::rectangle(0.2, 3.0).offset(post + vec2(0.0, -1.0)));
    }

    let mut balls = Spawner::new(2.0, 3);
    Box::new(move |world: &mut World, frame: &Frame| {
        balls.update(world, frame.time, |i| {
            let position = vec2(-1.5 + 3.0 * scatter(i), 9.0);
            BodyDesc::dynamic().position(position).collider(ColliderDesc::circle(0.4).density(2.0))
        });
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_band_holds_the_balls() {
        let mut world = World::new();
        let mut scene = build(&mut world, &mut Labels::default());
        for step in 0..12 * 60 {
            scene.update(&mut world, &Frame { time: step as f32 / 60.0, ..Frame::default() });
            world.step(1.0 / 60.0);
        }
        for (_, body) in world.bodies().filter(|(_, body)| body.mass() > 0.9) {
            assert!(body.position().y > 1.0, "the band let a ball down to {}", body.position());
        }
    }
}
