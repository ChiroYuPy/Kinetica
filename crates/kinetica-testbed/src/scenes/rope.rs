use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Rope",
    category: Category::SoftStructures,
    description: "A rope is a line of beads, each tied to the next by a distance joint used as a rope. One holds \
                  a crate, the other is whirled by a kinematic crank.",
    camera_center: vec2(0.0, 5.0),
    view_height: 12.0,
    build: Build::Static(build),
};

const BEAD_RADIUS: f32 = 0.08;

fn build(world: &mut World, _labels: &mut Labels) {
    add_ground(world, 40.0);
    let rope = |desc: DistanceJointDesc, length: f32| desc.rope(length);

    // Released horizontally, with a crate at the end.
    let ceiling = world.add_body(BodyDesc::fixed());
    let beads = add_beads(world, ceiling, vec2(-7.0, 9.0), vec2(-3.4, 9.0), 20, rope);
    let crate_body = world.add_body(BodyDesc::dynamic().position(vec2(-3.0, 8.7)).collider(ColliderDesc::square(0.6)));
    tie(world, beads[beads.len() - 1], crate_body, vec2(-3.0, 9.0), rope);

    let pivot = vec2(4.0, 5.5);
    let bar = ColliderDesc::rectangle(0.8, 0.1).offset(vec2(0.4, 0.0));
    let crank = world.add_body(BodyDesc::kinematic().position(pivot).angular_velocity(3.0).collider(bar));
    add_beads(world, crank, pivot + vec2(0.8, 0.0), pivot + vec2(3.3, 0.0), 12, rope);
}

/// Creates `count` beads from `start` (excluded, where `anchor` holds the first one) to `end`,
/// each tied to the previous one by a distance joint made by `kind`.
pub fn add_beads(
    world: &mut World,
    anchor: BodyHandle,
    start: Vec2,
    end: Vec2,
    count: usize,
    kind: impl Fn(DistanceJointDesc, f32) -> DistanceJointDesc,
) -> Vec<BodyHandle> {
    let step = (end - start) / count as f32;
    let mut previous = (anchor, start);
    let mut beads = Vec::with_capacity(count);
    for i in 1..=count {
        let position = start + step * i as f32;
        let ball = ColliderDesc::circle(BEAD_RADIUS).density(5.0);
        let bead = world.add_body(BodyDesc::dynamic().position(position).collider(ball));
        let desc = DistanceJointDesc::new(previous.0, bead, previous.1, position);
        world.add_joint(kind(desc, step.length()));
        beads.push(bead);
        previous = (bead, position);
    }
    beads
}

/// Ties `bead` to `point` on `body`.
pub fn tie(
    world: &mut World,
    bead: BodyHandle,
    body: BodyHandle,
    point: Vec2,
    kind: impl Fn(DistanceJointDesc, f32) -> DistanceJointDesc,
) {
    let bead_position = world[bead].position();
    let desc = DistanceJointDesc::new(bead, body, bead_position, point);
    world.add_joint(kind(desc, bead_position.distance(point)));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rope_links_do_not_stretch() {
        let mut world = World::new();
        build(&mut world, &mut Labels::default());
        for _ in 0..20 * 60 {
            world.step(1.0 / 60.0);
        }
        for (_, joint) in world.joints() {
            let Joint::Distance(distance) = joint else { continue };
            let (a, b) = (&world[joint.body_a()], &world[joint.body_b()]);
            let (anchor_a, anchor_b) = joint.local_anchors();
            let length = a.world_point(anchor_a).distance(b.world_point(anchor_b));
            assert!(length < distance.length() + 0.05, "a rope link stretched to {length}");
        }
    }
}
