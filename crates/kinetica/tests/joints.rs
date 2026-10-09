//! Physical behaviour of joints.

use std::f32::consts::{FRAC_PI_2, PI};

use kinetica::prelude::*;

const DT: f32 = 1.0 / 60.0;
/// Number of steps in one simulated second.
const SECOND: usize = 60;

fn run(world: &mut World, steps: usize) {
    for _ in 0..steps {
        world.step(DT);
    }
}

fn ball(world: &mut World, position: Vec2, radius: f32) -> BodyHandle {
    world.add_body(BodyDesc::dynamic().position(position).collider(ColliderDesc::circle(radius)))
}

#[test]
fn pendulum_keeps_its_length_and_period() {
    let mut world = World::new();
    let pivot = world.add_body(BodyDesc::fixed());
    let length = 2.0;
    let bob = ball(&mut world, vec2(length, 0.0), 0.1);
    world.add_joint(RevoluteJointDesc::new(pivot, bob, Vec2::ZERO));

    // Released horizontally; the period is the time between two crossings of the vertical
    // in the same direction.
    let mut crossings = Vec::new();
    let mut previous_x = length;
    for step in 0..10 * SECOND {
        world.step(DT);
        let position = world[bob].position();
        assert!((position.length() - length).abs() < 0.01, "rod stretched to {}", position.length());
        if previous_x < 0.0 && position.x >= 0.0 {
            crossings.push(step as f32 * DT);
        }
        previous_x = position.x;
    }

    // Large-amplitude (90°) period of a simple pendulum: 1.18034 × 2π√(L/g).
    let body = &world[bob];
    let pivot_inertia = body.inertia() + body.mass() * length * length;
    let small_angle_period = 2.0 * PI * (pivot_inertia / (body.mass() * 9.81 * length)).sqrt();
    let expected = 1.18034 * small_angle_period;
    let measured = crossings[1] - crossings[0];
    assert!((measured - expected).abs() < 0.03 * expected, "period {measured}, expected {expected}");
}

#[test]
fn pendulum_does_not_gain_energy() {
    let mut world = World::new();
    let pivot = world.add_body(BodyDesc::fixed());
    let bob = ball(&mut world, vec2(2.0, 0.0), 0.2);
    world.add_joint(RevoluteJointDesc::new(pivot, bob, Vec2::ZERO));

    let mut highest = f32::NEG_INFINITY;
    for _ in 0..20 * SECOND {
        world.step(DT);
        highest = highest.max(world[bob].position().y);
    }
    assert!(highest < 0.01, "the bob rose above its release height: {highest}");
}

/// A horizontal chain of 1 m links hanging from a pivot at the origin.
fn chain(world: &mut World, link_count: usize, damping: f32) -> Vec<BodyHandle> {
    let ceiling = world.add_body(BodyDesc::fixed());
    let mut previous = ceiling;
    let mut links = Vec::new();
    for i in 0..link_count {
        let x = 0.5 + i as f32;
        let link = world.add_body(
            BodyDesc::dynamic()
                .position(vec2(x, 0.0))
                .linear_damping(damping)
                .collider(ColliderDesc::rectangle(1.0, 0.2)),
        );
        world.add_joint(RevoluteJointDesc::new(previous, link, vec2(x - 0.5, 0.0)));
        previous = link;
        links.push(link);
    }
    links
}

/// Largest distance between the two anchors of any joint.
fn largest_joint_gap(world: &World) -> f32 {
    world
        .joints()
        .map(|(_, joint)| {
            let (local_a, local_b) = joint.local_anchors();
            let a = world[joint.body_a()].world_point(local_a);
            let b = world[joint.body_b()].world_point(local_b);
            a.distance(b)
        })
        .fold(0.0, f32::max)
}

#[test]
fn swinging_chain_stays_connected() {
    let mut world = World::new();
    let links = chain(&mut world, 30, 0.0);

    // Released at rest at height 0: kinetic plus potential energy starts at zero and can only
    // be lost. An unstable solver would create energy.
    let energy = |world: &World| -> f32 {
        links
            .iter()
            .map(|&link| {
                let body = &world[link];
                0.5 * body.mass() * body.linear_velocity().length_squared()
                    + 0.5 * body.inertia() * body.angular_velocity().powi(2)
                    + body.mass() * 9.81 * body.center_of_mass().y
            })
            .sum()
    };
    let total_mass: f32 = links.iter().map(|&link| world[link].mass()).sum();
    // Scale of the energies involved: the whole chain dropping by half its length.
    let energy_scale = total_mass * 9.81 * 15.0;

    for _ in 0..15 * SECOND {
        world.step(DT);
        // Joints are soft: at the peak of the whip (about 30 m/s) they stretch by up to 4 cm
        // with 4 substeps, 2.4 cm with 8.
        assert!(largest_joint_gap(&world) < 0.05, "a joint opened by {}", largest_joint_gap(&world));
        assert!(energy(&world) < 0.01 * energy_scale, "energy grew to {} J", energy(&world));
    }
}

#[test]
fn damped_chain_comes_to_rest_hanging_down() {
    let mut world = World::new();
    let links = chain(&mut world, 30, 1.0);

    run(&mut world, 30 * SECOND);

    let last = &world[*links.last().unwrap()];
    assert!(last.position().x.abs() < 0.1, "the chain should hang straight down, last link at {:?}", last.position());
    assert!((last.position().y + 29.5).abs() < 0.1, "last link at {:?}", last.position());
    assert!(last.linear_velocity().length() < 0.01);
}

#[test]
fn newtons_cradle_passes_momentum_to_the_last_ball() {
    let mut world = World::new();
    // A 5 cm gap makes the impacts sequential; touching balls would share the momentum.
    let spacing = 1.05;
    let balls: Vec<_> = (0..5)
        .map(|i| {
            let pivot_position = vec2((i as f32 - 2.0) * spacing, 9.0);
            let pivot = world.add_body(BodyDesc::fixed().position(pivot_position));
            let angle: f32 = if i == 0 { -1.0 } else { 0.0 };
            let position = pivot_position + 4.0 * vec2(angle.sin(), -angle.cos());
            let ball = world.add_body(
                BodyDesc::dynamic()
                    .position(position)
                    .collider(ColliderDesc::circle(0.5).restitution(1.0).friction(0.0)),
            );
            world.add_joint(DistanceJointDesc::new(pivot, ball, pivot_position, position));
            ball
        })
        .collect();

    // Swing until the last ball moves.
    let speed = |world: &World, ball: BodyHandle| world[ball].linear_velocity().length();
    for _ in 0..2 * SECOND {
        world.step(DT);
        if speed(&world, balls[4]) > 1.0 {
            break;
        }
    }

    // Released from 1 rad on a 4 m string: v = √(2gL(1 − cos 1)).
    let impact_speed = (2.0 * 9.81 * 4.0 * (1.0 - 1f32.cos())).sqrt();
    assert!((speed(&world, balls[4]) - impact_speed).abs() < 0.05 * impact_speed);
    for &ball in &balls[..4] {
        assert!(speed(&world, ball) < 0.1, "ball kept {} m/s", speed(&world, ball));
    }
}

#[test]
fn revolute_limits_are_respected() {
    let mut world = World::new();
    let wall = world.add_body(BodyDesc::fixed());
    let arm = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)).collider(ColliderDesc::rectangle(2.0, 0.2)));
    let limit = 0.5;
    let hinge = world.add_joint(RevoluteJointDesc::new(wall, arm, Vec2::ZERO).limits(-limit..=limit));

    run(&mut world, 3 * SECOND);

    let angle = world[arm].angle();
    assert!((angle + limit).abs() < 0.02, "the arm should rest on its lower limit, at {angle}");
    assert_eq!(world[hinge].limits(), Some(-limit..=limit));
}

#[test]
fn revolute_motor_spins_a_wheel_at_its_speed() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let axle = world.add_body(BodyDesc::fixed());
    let wheel = ball(&mut world, Vec2::ZERO, 0.5);
    let speed = 3.0;
    world.add_joint(RevoluteJointDesc::new(axle, wheel, Vec2::ZERO).motor(speed, 100.0));

    run(&mut world, SECOND);

    assert!((world[wheel].angular_velocity() - speed).abs() < 1e-3);
}

#[test]
fn weak_motor_cannot_lift_a_heavy_arm() {
    let mut world = World::new();
    let wall = world.add_body(BodyDesc::fixed());
    let arm = world.add_body(
        BodyDesc::dynamic().position(vec2(1.0, 0.0)).collider(ColliderDesc::rectangle(2.0, 0.2).density(10.0)),
    );
    let hinge = world.add_joint(RevoluteJointDesc::new(wall, arm, Vec2::ZERO).motor(1.0, 1.0));

    run(&mut world, SECOND);
    assert!(world[arm].angle() < -0.5, "a weak motor should lose against gravity");

    // A strong motor lifts it.
    world[hinge].set_motor(Some(Motor { speed: 1.0, max_force: 1000.0 }));
    let angle_before = world[arm].angle();
    run(&mut world, SECOND / 2);
    assert!(world[arm].angle() > angle_before + 0.3);
}

#[test]
fn rigid_distance_joint_keeps_its_length() {
    let mut world = World::new();
    let anchor = world.add_body(BodyDesc::fixed());
    let weight = ball(&mut world, vec2(1.5, 1.0), 0.2);
    world.add_joint(DistanceJointDesc::new(anchor, weight, Vec2::ZERO, vec2(1.5, 1.0)));
    let length = vec2(1.5, 1.0).length();

    for _ in 0..5 * SECOND {
        world.step(DT);
        let distance = world[weight].position().length();
        assert!((distance - length).abs() < 0.01, "length {distance}, expected {length}");
    }
}

#[test]
fn rope_goes_slack_but_never_stretches() {
    let mut world = World::new();
    let anchor = world.add_body(BodyDesc::fixed());
    let weight = ball(&mut world, vec2(0.0, -1.0), 0.2);
    let rope_length = 3.0;
    world.add_joint(DistanceJointDesc::new(anchor, weight, Vec2::ZERO, vec2(0.0, -1.0)).rope(rope_length));

    let mut farthest = 0.0_f32;
    for _ in 0..3 * SECOND {
        world.step(DT);
        farthest = farthest.max(world[weight].position().length());
    }
    assert!(farthest < rope_length + 0.02, "rope stretched to {farthest}");
    assert!(farthest > rope_length - 0.05, "the weight should have fallen to the end of the rope");
}

#[test]
fn spring_settles_where_force_balances_gravity() {
    let mut world = World::new();
    let anchor = world.add_body(BodyDesc::fixed());
    let weight = ball(&mut world, vec2(0.0, -1.0), 0.2);
    let hertz = 2.0;
    world.add_joint(DistanceJointDesc::new(anchor, weight, Vec2::ZERO, vec2(0.0, -1.0)).length(1.0).spring(hertz, 0.5));

    run(&mut world, 10 * SECOND);

    // A spring of frequency f on a mass m has stiffness k = m(2πf)²; it stretches by mg/k.
    let omega = 2.0 * PI * hertz;
    let expected_stretch = 9.81 / (omega * omega);
    let length = world[weight].position().length();
    assert!(
        (length - 1.0 - expected_stretch).abs() < 0.01,
        "stretched by {}, expected {expected_stretch}",
        length - 1.0
    );
}

#[test]
fn prismatic_joint_slides_along_its_axis_only() {
    let mut world = World::new();
    let rail = world.add_body(BodyDesc::fixed());
    let slider = world.add_body(BodyDesc::dynamic().collider(ColliderDesc::rectangle(1.0, 0.5)));
    // A 45° rail going down to the right, limited to 2 m of travel.
    let axis = vec2(1.0, -1.0).normalize();
    world.add_joint(PrismaticJointDesc::new(rail, slider, Vec2::ZERO, axis).limits(0.0..=2.0));

    run(&mut world, 3 * SECOND);

    let body = &world[slider];
    let position = body.position();
    assert!(position.perp_dot(axis).abs() < 0.01, "left the rail: {position:?}");
    assert!((position.dot(axis) - 2.0).abs() < 0.02, "should rest at the end of travel: {position:?}");
    assert!(body.angle().abs() < 0.01, "a prismatic joint forbids rotation");
}

#[test]
fn prismatic_motor_lifts_an_elevator() {
    let mut world = World::new();
    let shaft = world.add_body(BodyDesc::fixed());
    let cabin = world.add_body(BodyDesc::dynamic().collider(ColliderDesc::rectangle(2.0, 0.4)));
    world.add_joint(PrismaticJointDesc::new(shaft, cabin, Vec2::ZERO, Vec2::Y).motor(1.5, 1000.0));

    run(&mut world, SECOND);

    let velocity = world[cabin].linear_velocity();
    assert!((velocity.y - 1.5).abs() < 0.01 && velocity.x.abs() < 1e-3, "cabin moves at {velocity:?}");
}

#[test]
fn weld_holds_a_cantilever() {
    let mut world = World::new();
    let wall = world.add_body(BodyDesc::fixed());
    let beam = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)).collider(ColliderDesc::rectangle(2.0, 0.2)));
    world.add_joint(WeldJointDesc::new(wall, beam, Vec2::ZERO));

    run(&mut world, 3 * SECOND);

    let body = &world[beam];
    assert!(body.angle().abs() < 0.01, "the beam sagged to {}", body.angle());
    assert!(body.position().abs_diff_eq(vec2(1.0, 0.0), 0.01));
}

#[test]
fn soft_weld_bends_under_load_and_springs_back() {
    let mut world = World::new();
    let wall = world.add_body(BodyDesc::fixed());
    let beam = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)).collider(ColliderDesc::rectangle(2.0, 0.2)));
    world.add_joint(WeldJointDesc::new(wall, beam, Vec2::ZERO).angular_spring(2.0, 0.7));

    run(&mut world, 5 * SECOND);
    let sag = world[beam].angle();
    assert!(sag < -0.1, "a soft weld should bend under its own weight, angle {sag}");
    assert!(world[beam].angular_velocity().abs() < 0.01, "and settle");

    world.set_gravity(Vec2::ZERO);
    run(&mut world, 5 * SECOND);
    assert!(world[beam].angle().abs() < 0.01, "it should straighten without load");
}

#[test]
fn mouse_joint_drags_a_body_to_its_target() {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed());
    let crate_body = world.add_body(BodyDesc::dynamic().collider(ColliderDesc::square(1.0)));
    let mouse = world.add_joint(MouseJointDesc::new(ground, crate_body, Vec2::ZERO));

    let target = vec2(3.0, 2.0);
    world[mouse].set_target(target);
    run(&mut world, 3 * SECOND);

    // Gravity pulls the hanging crate slightly below the target, like a spring.
    let position = world[crate_body].position();
    assert!(position.distance(target) < 0.05, "crate at {position:?}");
}

#[test]
fn joined_bodies_do_not_collide_unless_asked() {
    for collide_connected in [false, true] {
        let mut world = World::new();
        world.set_gravity(Vec2::ZERO);
        let a = ball(&mut world, Vec2::ZERO, 1.0);
        let b = ball(&mut world, vec2(1.0, 0.0), 1.0);
        let desc = RevoluteJointDesc::new(a, b, vec2(0.5, 0.0));
        world.add_joint(if collide_connected { desc.collide_connected() } else { desc });
        run(&mut world, 2);
        assert_eq!(world.contacts().count(), usize::from(collide_connected), "collide_connected = {collide_connected}");
    }
}

#[test]
fn removing_a_joint_restores_collisions() {
    let mut world = World::new();
    world.set_gravity(Vec2::ZERO);
    let a = ball(&mut world, Vec2::ZERO, 1.0);
    let b = ball(&mut world, vec2(1.0, 0.0), 1.0);
    let joint = world.add_joint(RevoluteJointDesc::new(a, b, vec2(0.5, 0.0)));
    run(&mut world, 2);
    assert_eq!(world.contacts().count(), 0);

    world.remove_joint(joint);
    run(&mut world, 2);
    assert_eq!(world.contacts().count(), 1);
}

#[test]
fn removing_a_body_removes_its_joints() {
    let mut world = World::new();
    let ground = world.add_body(BodyDesc::fixed());
    let first = ball(&mut world, vec2(1.0, 0.0), 0.2);
    let second = ball(&mut world, vec2(2.0, 0.0), 0.2);
    let to_ground = world.add_joint(RevoluteJointDesc::new(ground, first, Vec2::ZERO));
    let between = world.add_joint(RevoluteJointDesc::new(first, second, vec2(1.5, 0.0)));

    world.remove_body(first);

    assert!(world.get_joint(to_ground).is_none() && world.get_joint(between).is_none());
    assert_eq!(world[ground].joints(), []);
    assert_eq!(world[second].joints(), []);
    run(&mut world, 2);
}

#[test]
fn point_query_finds_the_shapes_under_a_point() {
    let mut world = World::new();
    let circle = ball(&mut world, Vec2::ZERO, 1.0);
    let block = world.add_body(BodyDesc::dynamic().position(vec2(3.0, 0.0)).angle(FRAC_PI_2 / 2.0));
    let block_collider = world.add_collider(block, ColliderDesc::square(2.0));

    let bodies_at = |point: Vec2| -> Vec<BodyHandle> {
        world.colliders_at_point(point, QueryFilter::new()).into_iter().map(|collider| world[collider].body()).collect()
    };
    assert_eq!(bodies_at(vec2(0.5, 0.5)), [circle]);
    assert_eq!(bodies_at(vec2(3.0, 1.3)), [block], "inside the rotated square's top corner");
    assert!(bodies_at(vec2(4.0, 1.0)).is_empty(), "outside the rotated square");
    assert!(world.colliders_at_point(vec2(3.0, 0.0), QueryFilter::new()).contains(&block_collider));
}

#[test]
fn constraint_force_holds_a_hanging_weight() {
    let joints: [fn(&mut World, BodyHandle, BodyHandle) -> JointHandle; 4] = [
        |world, a, b| world.add_joint(RevoluteJointDesc::new(a, b, Vec2::ZERO)).untyped(),
        |world, a, b| world.add_joint(DistanceJointDesc::new(a, b, Vec2::ZERO, vec2(0.0, -1.0))).untyped(),
        |world, a, b| world.add_joint(DistanceJointDesc::new(a, b, Vec2::ZERO, vec2(0.0, -1.0)).rope(1.0)).untyped(),
        |world, a, b| world.add_joint(WeldJointDesc::new(a, b, Vec2::ZERO)).untyped(),
    ];
    for add_joint in joints {
        let mut world = World::new();
        let ceiling = world.add_body(BodyDesc::fixed());
        let weight = world
            .add_body(BodyDesc::dynamic().position(vec2(0.0, -1.0)).collider(ColliderDesc::circle(0.2).density(10.0)));
        let joint = add_joint(&mut world, ceiling, weight);
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let weight_force = world[weight].mass() * 9.81;
        let force = world[joint].constraint_force();
        assert!(
            force.abs_diff_eq(vec2(0.0, weight_force), 0.02 * weight_force),
            "{force} for a weight of {weight_force}"
        );
    }
}

#[test]
fn constraint_torque_holds_a_cantilever() {
    let mut world = World::new();
    let wall = world.add_body(BodyDesc::fixed());
    // A 2 m beam welded to the wall at its left end.
    let beam = world.add_body(BodyDesc::dynamic().position(vec2(1.0, 0.0)).collider(ColliderDesc::rectangle(2.0, 0.2)));
    let joint = world.add_joint(WeldJointDesc::new(wall, beam, Vec2::ZERO));
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let weight = world[beam].mass() * 9.81;
    let joint = &world[joint];
    // The weight acts 1 m from the weld: the joint must turn the beam back up.
    let expected_torque = weight * 1.0;
    assert!(
        (joint.constraint_torque() - expected_torque).abs() < 0.05 * expected_torque,
        "{}",
        joint.constraint_torque()
    );
    assert!((joint.constraint_force().y - weight).abs() < 0.02 * weight);
}
