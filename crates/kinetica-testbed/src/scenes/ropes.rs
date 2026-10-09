use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Ropes",
    category: Category::SoftStructures,
    description: "From left to right: a rope of beads holding a crate, a chain of hinged links, an elastic \
                  band catching a ball, and a rope whirled by a turning motor.",
    camera_center: vec2(0.5, 5.5),
    view_height: 16.0,
    build: Build::Static(build),
};

const BEAD_RADIUS: f32 = 0.08;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 20.0);
    let ceiling = world.add_body(BodyDesc::fixed());
    let rope = |desc: DistanceJointDesc, length: f32| desc.spring(0.0, 0.0).limits(0.0..=length);

    // A rope: beads that may get closer but never further apart, released horizontally.
    let beads = add_beads(world, ceiling, vec2(-9.5, 10.0), vec2(-5.9, 10.0), 20, rope);
    let crate_body = world.add_body(BodyDesc::dynamic().position(vec2(-5.5, 9.7)));
    world.add_collider(crate_body, ColliderDesc::rectangle(0.3, 0.3));
    tie(world, beads[beads.len() - 1], crate_body, vec2(-5.5, 10.0), rope);

    // A chain: small bars hinged end to end, ending with a ball.
    let link_length = 0.4;
    let start = vec2(-1.5, 10.0);
    let mut previous = ceiling;
    for i in 0..11 {
        let hinge = start + vec2(link_length * i as f32, 0.0);
        let link = world.add_body(BodyDesc::dynamic().position(hinge + vec2(link_length / 2.0, 0.0)));
        world.add_collider(link, ColliderDesc::rectangle(link_length / 2.0, 0.05));
        world.add_joint(RevoluteJointDesc::at_world_point(world, previous, link, hinge));
        previous = link;
    }
    let end = start + vec2(11.0 * link_length, 0.0);
    let ball = world.add_body(BodyDesc::dynamic().position(end + vec2(0.3, 0.0)));
    world.add_collider(ball, ColliderDesc::circle(0.3).density(3.0));
    world.add_joint(RevoluteJointDesc::at_world_point(world, previous, ball, end));

    // An elastic band: beads tied by springs between two posts, and a ball dropped on it. The
    // springs are shorter than the gaps, so the band is stretched like a trampoline.
    let elastic = |desc: DistanceJointDesc, length: f32| desc.spring(15.0, 0.3).length(0.85 * length);
    let (left, right) = (vec2(3.0, 3.0), vec2(8.0, 3.0));
    let band = add_beads(world, ceiling, left, right - vec2(0.25, 0.0), 19, elastic);
    tie(world, band[band.len() - 1], ceiling, right, elastic);
    for post in [left, right] {
        world.add_collider(ceiling, ColliderDesc::offset_rectangle(0.1, 1.5, post - vec2(0.0, 1.5), 0.0));
    }
    let falling = world.add_body(BodyDesc::dynamic().position(vec2(5.5, 8.0)));
    world.add_collider(falling, ColliderDesc::circle(0.4).density(2.0));

    // A rope whirled around: tied to the end of a turning kinematic crank.
    let pivot = vec2(9.5, 8.0);
    let crank = world.add_body(BodyDesc::kinematic().position(pivot).angular_velocity(3.0));
    world.add_collider(crank, ColliderDesc::offset_rectangle(0.4, 0.05, vec2(0.4, 0.0), 0.0));
    add_beads(world, crank, pivot + vec2(0.8, 0.0), pivot + vec2(3.3, 0.0), 12, rope);

    labels.at(vec2(-7.5, 10.5), "Rope: beads joined by\ndistance joints, slack allowed");
    labels.at(vec2(0.7, 10.5), "Chain: bars joined\nby revolute joints");
    labels.at(vec2(5.5, 4.6), "Elastic band: beads\njoined by springs");
    labels.at(pivot + vec2(0.0, 0.6), "Rope on a turning\nkinematic crank");
}

/// Creates `count` beads from `start` (excluded, where `anchor` holds the first one) to `end`,
/// each tied to the previous one by a distance joint made by `kind`.
fn add_beads(
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
        let bead = world.add_body(BodyDesc::dynamic().position(position));
        world.add_collider(bead, ColliderDesc::circle(BEAD_RADIUS).density(5.0));
        let desc = DistanceJointDesc::between_world_points(world, previous.0, bead, previous.1, position);
        world.add_joint(kind(desc, step.length()));
        beads.push(bead);
        previous = (bead, position);
    }
    beads
}

/// Ties `bead` to `point` on `body`.
fn tie(
    world: &mut World,
    bead: BodyHandle,
    body: BodyHandle,
    point: Vec2,
    kind: impl Fn(DistanceJointDesc, f32) -> DistanceJointDesc,
) {
    let bead_position = world.body(bead).map_or(point, kinetica::Body::position);
    let desc = DistanceJointDesc::between_world_points(world, bead, body, bead_position, point);
    let length = bead_position.distance(point);
    world.add_joint(kind(desc, length));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ropes_stay_together() {
        let mut world = World::new();
        build(&mut world, &mut Labels::default());
        for _ in 0..20 * 60 {
            world.step(1.0 / 60.0);
            for (_, body) in world.bodies() {
                assert!(body.position().is_finite() && body.linear_velocity().length() < 50.0, "{body:?}");
            }
        }
        // The ball dropped on the elastic band bounces, then rests on it, above the ground.
        let ball = world
            .bodies()
            .find(|(_, body)| body.mass() > 0.9 && body.mass() < 1.1)
            .map(|(_, body)| body.position())
            .unwrap();
        assert!(ball.y > 1.0, "the elastic band let the ball down to {ball}");

        // Rope links never get much longer than their length.
        for (_, joint) in world.joints() {
            if let JointKind::Distance(distance) = joint.kind()
                && distance.spring().is_some_and(|spring| spring.hertz == 0.0)
            {
                let (a, b) = (world.body(joint.body_a()).unwrap(), world.body(joint.body_b()).unwrap());
                let (anchor_a, anchor_b) = joint.local_anchors();
                let length = a.world_point(anchor_a).distance(b.world_point(anchor_b));
                assert!(length < distance.length() + 0.05, "a rope link stretched to {length}");
            }
        }
    }
}
