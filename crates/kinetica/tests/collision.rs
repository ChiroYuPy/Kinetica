//! Contacts and collision events seen through the `World`.

use kinetica::prelude::*;
use kinetica::{BodyType, Contact};

const DT: f32 = 1.0 / 60.0;

struct Scene {
    world: World,
    ground: BodyHandle,
    ball: BodyHandle,
}

/// A ball dropped from above a thin fixed ground.
fn ball_above_ground() -> Scene {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed());
    world.add_collider(ground, ColliderDesc::rectangle(5.0, 0.1));
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 1.0)));
    world.add_collider(ball, ColliderDesc::circle(0.25));
    Scene { world, ground, ball }
}

#[test]
fn ball_lands_then_leaves_the_ground() {
    let Scene { mut world, ground, ball } = ball_above_ground();
    let mut history = Vec::new();
    let mut record_events = |world: &World| {
        for event in world.collision_events() {
            assert!(event.pair().involves_body(ground) && event.pair().involves_body(ball));
            history.push(matches!(event, CollisionEvent::Started(_)));
        }
    };

    for _ in 0..120 {
        world.step(DT);
        record_events(&world);
    }
    world.body_mut(ball).unwrap().set_linear_velocity(vec2(0.0, 5.0));
    for _ in 0..30 {
        world.step(DT);
        record_events(&world);
    }

    assert_eq!(history, [true, false], "expected exactly one Started then one Stopped");
}

#[test]
fn contact_normal_points_from_a_to_b() {
    let Scene { mut world, ground, .. } = ball_above_ground();

    let contact = step_until_contact(&mut world);
    let pair = contact.pair();
    let ground_to_ball = if pair.body_a == ground { Vec2::Y } else { -Vec2::Y };
    assert!(contact.manifold().normal().abs_diff_eq(ground_to_ball, 1e-5));
    assert_eq!(pair.other_body(ground), Some(if pair.body_a == ground { pair.body_b } else { pair.body_a }));
}

#[test]
fn body_contacts_lists_only_that_body() {
    let Scene { mut world, ground, ball } = ball_above_ground();
    let far_away = world.add_body(BodyDesc::dynamic().position(vec2(50.0, 0.0)));
    world.add_collider(far_away, ColliderDesc::circle(0.25));

    step_until_contact(&mut world);
    assert_eq!(world.body_contacts(ball).count(), 1);
    assert_eq!(world.body_contacts(ground).count(), 1);
    assert_eq!(world.body_contacts(far_away).count(), 0);
}

#[test]
fn colliders_of_the_same_body_never_touch() {
    let mut world = World::new();
    let body = world.add_body(BodyDesc::dynamic().gravity_scale(0.0));
    world.add_collider(body, ColliderDesc::circle(1.0));
    world.add_collider(body, ColliderDesc::rectangle(1.0, 1.0));

    for _ in 0..3 {
        world.step(DT);
    }
    assert_eq!(world.contacts().count(), 0);
}

#[test]
fn only_pairs_with_a_dynamic_body_make_contacts() {
    let types = [BodyType::Fixed, BodyType::Kinematic, BodyType::Dynamic];
    for first in types {
        for second in types {
            let mut world = World::new();
            world.set_gravity(Vec2::ZERO);
            for body_type in [first, second] {
                let body = world.add_body(BodyDesc::new(body_type));
                world.add_collider(body, ColliderDesc::circle(1.0));
            }
            world.step(DT);
            world.step(DT);

            let expected = usize::from(first == BodyType::Dynamic || second == BodyType::Dynamic);
            assert_eq!(world.contacts().count(), expected, "{first:?} vs {second:?}");
        }
    }
}

#[test]
fn removing_a_body_removes_its_contacts_silently() {
    let Scene { mut world, ball, .. } = ball_above_ground();
    step_until_contact(&mut world);

    world.remove_body(ball);
    world.step(DT);
    assert_eq!(world.contacts().count(), 0);
    assert_eq!(world.collision_events(), &[]);
}

#[test]
fn removing_a_collider_keeps_other_contacts() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let probe = world.add_body(BodyDesc::dynamic());
    world.add_collider(probe, ColliderDesc::circle(1.0));
    let target = world.add_body(BodyDesc::fixed());
    let left = world.add_collider(target, ColliderDesc::rectangle(0.5, 0.5));
    world.add_collider(target, ColliderDesc::circle(0.5));

    world.step(DT);
    world.step(DT);
    assert_eq!(world.contacts().count(), 2);

    world.remove_collider(left);
    world.step(DT);
    assert_eq!(world.contacts().count(), 1);
}

#[test]
fn many_bodies_find_exactly_the_overlapping_pairs() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    // A row of circles of radius 0.5 spaced by 0.9: each overlaps only its neighbours.
    let count = 30;
    for i in 0..count {
        let body = world.add_body(BodyDesc::dynamic().position(vec2(i as f32 * 0.9, 0.0)));
        world.add_collider(body, ColliderDesc::circle(0.5));
    }
    world.step(DT);
    world.step(DT);
    assert_eq!(world.contacts().count(), count - 1);
}

fn step_until_contact(world: &mut World) -> Contact {
    for _ in 0..240 {
        world.step(DT);
        if let Some(contact) = world.contacts().next() {
            return contact.clone();
        }
    }
    panic!("no contact happened");
}
