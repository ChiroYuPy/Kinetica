//! Sleeping: resting bodies stop being simulated, and wake up when something happens to them.

use kinetica::prelude::*;

const DT: f32 = 1.0 / 60.0;
const SECOND: usize = 60;

fn world_with_ground() -> World {
    let mut world = World::new();
    world.add_body(BodyDesc::fixed().position(vec2(0.0, -0.5)).collider(ColliderDesc::rectangle(100.0, 1.0)));
    world
}

fn run(world: &mut World, steps: usize) {
    for _ in 0..steps {
        world.step(DT);
    }
}

fn add_box(world: &mut World, position: Vec2) -> BodyHandle {
    world.add_body(BodyDesc::dynamic().position(position).collider(ColliderDesc::square(1.0)))
}

fn is_sleeping(world: &World, body: BodyHandle) -> bool {
    world[body].is_sleeping()
}

/// A stack of boxes standing on the ground at `x`, bottom first.
fn add_stack(world: &mut World, x: f32, height: usize) -> Vec<BodyHandle> {
    (0..height).map(|i| add_box(world, vec2(x, 0.5 + i as f32))).collect()
}

#[test]
fn setting_unchanged_values_does_not_wake_a_body() {
    let mut world = world_with_ground();
    let block = add_box(&mut world, vec2(0.0, 0.5));
    run(&mut world, 2 * SECOND);
    assert!(is_sleeping(&world, block));

    let body = &mut world[block];
    let (position, velocity) = (body.position(), body.linear_velocity());
    body.set_position(position);
    body.set_linear_velocity(velocity);
    body.set_angular_velocity(body.angular_velocity());
    body.set_gravity_scale(1.0);
    assert!(body.is_sleeping(), "unchanged values woke the body");

    body.set_linear_velocity(vec2(1.0, 0.0));
    assert!(!body.is_sleeping());
}

#[test]
fn resting_box_falls_asleep() {
    let mut world = world_with_ground();
    let block = add_box(&mut world, vec2(0.0, 2.0));
    run(&mut world, SECOND / 2);
    assert!(!is_sleeping(&world, block), "asleep while still falling");

    run(&mut world, 2 * SECOND);
    let body = &world[block];
    assert!(body.is_sleeping());
    assert_eq!(body.linear_velocity(), Vec2::ZERO);
}

#[test]
fn pyramid_falls_asleep() {
    let mut world = world_with_ground();
    let mut boxes = Vec::new();
    for row in 0..10 {
        for column in 0..10 - row {
            let x = column as f32 - (10 - row) as f32 / 2.0;
            boxes.push(add_box(&mut world, vec2(x, 0.5 + row as f32)));
        }
    }
    run(&mut world, 5 * SECOND);
    let awake = boxes.iter().filter(|&&block| !is_sleeping(&world, block)).count();
    assert_eq!(awake, 0, "{awake} boxes still awake");
}

#[test]
fn sleeping_bodies_do_not_move() {
    let mut world = world_with_ground();
    let stack = add_stack(&mut world, 0.0, 3);
    run(&mut world, 3 * SECOND);
    let before: Vec<_> = stack.iter().map(|&block| *world[block].transform()).collect();
    run(&mut world, 3 * SECOND);
    let after: Vec<_> = stack.iter().map(|&block| *world[block].transform()).collect();
    assert_eq!(before, after);
}

#[test]
fn falling_box_wakes_the_whole_stack() {
    let mut world = world_with_ground();
    let stack = add_stack(&mut world, 0.0, 3);
    run(&mut world, 3 * SECOND);
    assert!(stack.iter().all(|&block| is_sleeping(&world, block)));

    let falling = add_box(&mut world, vec2(0.3, 6.0));
    let mut woke = false;
    for _ in 0..2 * SECOND {
        world.step(DT);
        let touching = world.body_contacts(falling).any(|contact| contact.pair().involves_body(stack[2]));
        if touching {
            assert!(stack.iter().all(|&block| !is_sleeping(&world, block)), "the stack must wake up as a whole");
            woke = true;
            break;
        }
    }
    assert!(woke, "the falling box never reached the stack");
}

#[test]
fn islands_sleep_and_wake_independently() {
    let mut world = world_with_ground();
    let left = add_stack(&mut world, -5.0, 2);
    let right = add_stack(&mut world, 5.0, 2);
    run(&mut world, 3 * SECOND);

    world[left[0]].apply_linear_impulse(vec2(0.0, 0.01));
    world.step(DT);
    assert!(left.iter().all(|&block| !is_sleeping(&world, block)), "the touched stack wakes up");
    assert!(right.iter().all(|&block| is_sleeping(&world, block)), "the other stack keeps sleeping");
}

#[test]
fn removing_the_support_wakes_what_rested_on_it() {
    let mut world = world_with_ground();
    let stack = add_stack(&mut world, 0.0, 2);
    run(&mut world, 3 * SECOND);
    assert!(is_sleeping(&world, stack[1]));

    world.remove_body(stack[0]);
    assert!(!is_sleeping(&world, stack[1]));
    run(&mut world, 2 * SECOND);
    let top = &world[stack[1]];
    assert!((top.position().y - 0.5).abs() < 0.02, "the top box should have fallen to the ground: {}", top.position());
}

#[test]
fn joints_link_islands() {
    let mut world = world_with_ground();
    let a = add_box(&mut world, vec2(-2.0, 0.5));
    let b = add_box(&mut world, vec2(2.0, 0.5));
    world.add_joint(DistanceJointDesc::new(a, b, vec2(-2.0, 0.5), vec2(2.0, 0.5)));
    run(&mut world, 3 * SECOND);
    assert!(is_sleeping(&world, a) && is_sleeping(&world, b));

    world[a].apply_linear_impulse(vec2(1.0, 0.0));
    world.step(DT);
    assert!(!is_sleeping(&world, b), "a joined body wakes up with its partner");
}

#[test]
fn moving_kinematic_body_keeps_its_passengers_awake() {
    let mut world = world_with_ground();
    let platform =
        world.add_body(BodyDesc::kinematic().position(vec2(0.0, 2.0)).collider(ColliderDesc::rectangle(6.0, 0.2)));
    let passenger = add_box(&mut world, vec2(0.0, 2.6));
    run(&mut world, 3 * SECOND);
    assert!(is_sleeping(&world, passenger));

    world[platform].set_linear_velocity(vec2(0.02, 0.0));
    for _ in 0..3 * SECOND {
        world.step(DT);
        assert!(!is_sleeping(&world, passenger), "the passenger fell asleep on a moving platform");
    }
    assert!(world[passenger].position().x > 0.04, "the passenger should ride along");
}

#[test]
fn bodies_that_cannot_sleep_stay_awake() {
    let mut world = world_with_ground();
    let block =
        world.add_body(BodyDesc::dynamic().position(vec2(0.0, 0.5)).keep_awake().collider(ColliderDesc::square(1.0)));
    run(&mut world, 3 * SECOND);
    assert!(!is_sleeping(&world, block));
}

#[test]
fn disabling_sleep_wakes_everything() {
    let mut world = world_with_ground();
    let stack = add_stack(&mut world, 0.0, 3);
    run(&mut world, 3 * SECOND);
    world.set_sleeping_enabled(false);
    assert!(stack.iter().all(|&block| !is_sleeping(&world, block)));
    run(&mut world, 3 * SECOND);
    assert!(stack.iter().all(|&block| !is_sleeping(&world, block)));
}

#[test]
fn sleeping_world_stays_deterministic() {
    let mut world = world_with_ground();
    add_stack(&mut world, 0.0, 4);
    run(&mut world, SECOND / 2);
    let mut copy = world.clone();
    run(&mut world, 3 * SECOND);
    run(&mut copy, 3 * SECOND);
    let positions = |world: &World| world.bodies().map(|(_, body)| *body.transform()).collect::<Vec<_>>();
    assert_eq!(positions(&world), positions(&copy));
}
