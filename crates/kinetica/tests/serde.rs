//! Saving and loading worlds and descriptions with serde.
#![cfg(feature = "serde")]

use kinetica::prelude::*;

/// A world with every kind of shape, joint and contact, mid-simulation.
fn busy_world() -> World {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed());
    world.add_chain(ground, ChainDesc::new([vec2(-10.0, 1.0), vec2(-3.0, 0.0), vec2(10.0, 0.0)]));
    world.add_collider(ground, ColliderDesc::square(2.0).sensor());
    for i in 0..12 {
        let body = world.add_body(BodyDesc::dynamic().position(vec2(i as f32 * 0.7 - 4.0, 2.0 + i as f32 * 0.5)));
        let collider = match i % 4 {
            0 => ColliderDesc::circle(0.3),
            1 => ColliderDesc::rectangle(0.6, 0.4),
            2 => ColliderDesc::capsule(vec2(-0.3, 0.0), vec2(0.3, 0.0), 0.15),
            _ => ColliderDesc::regular_polygon(5, 0.3).restitution(0.5),
        };
        world.add_collider(body, collider);
    }
    let pivot = world.add_body(BodyDesc::fixed().position(vec2(6.0, 5.0)));
    let bob = world.add_body(BodyDesc::dynamic().position(vec2(8.0, 5.0)).collider(ColliderDesc::circle(0.4)));
    world.add_joint(RevoluteJointDesc::new(pivot, bob, vec2(6.0, 5.0)));
    for _ in 0..90 {
        world.step(1.0 / 60.0);
    }
    world
}

fn positions(world: &World) -> Vec<Transform> {
    world.bodies().map(|(_, body)| *body.transform()).collect()
}

#[test]
fn a_loaded_world_continues_exactly_like_the_original() {
    let mut original = busy_world();
    let json = serde_json::to_string(&original).unwrap();
    let mut loaded: World = serde_json::from_str(&json).unwrap();
    assert!(original.contacts().count() > 0, "the test needs contacts to warm start");

    for _ in 0..120 {
        original.step(1.0 / 60.0);
        loaded.step(1.0 / 60.0);
    }
    assert_eq!(positions(&original), positions(&loaded));
    assert_eq!(original.collision_events(), loaded.collision_events());
}

#[test]
fn descriptions_round_trip() {
    let desc = ColliderDesc::capsule(vec2(0.0, 0.0), vec2(1.0, 0.0), 0.2).friction(0.3).sensor();
    let loaded: ColliderDesc = serde_json::from_str(&serde_json::to_string(&desc).unwrap()).unwrap();
    assert_eq!(desc, loaded);

    let body = BodyDesc::dynamic().position(vec2(1.0, 2.0)).angle(0.5).bullet();
    let loaded: BodyDesc = serde_json::from_str(&serde_json::to_string(&body).unwrap()).unwrap();
    assert_eq!(body, loaded);
}
