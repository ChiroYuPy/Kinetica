//! The benchmark worlds. Each one stresses a different part of the engine.

use std::f32::consts::TAU;

use kinetica::math::{Rot, Transform};
use kinetica::prelude::*;
use kinetica::shape::{Circle, Shape};

use crate::{Benchmark, Iteration};

pub(crate) fn all() -> Vec<Benchmark> {
    vec![
        Benchmark::steps(
            "pyramid",
            "820 boxes in a 40-box base pyramid, sleeping off: deep stacks, contact solver.",
            pyramid,
            60,
            300,
        ),
        Benchmark::steps(
            "rain",
            "2000 mixed shapes falling into a pit and piling up: broad and narrow phase churn.",
            rain,
            0,
            600,
        ),
        Benchmark::steps(
            "tumbler",
            "1500 mixed shapes in a turning kinematic drum: many changing contacts.",
            tumbler,
            60,
            300,
        ),
        Benchmark::steps(
            "cloth",
            "A 50 x 50 cloth of rope joints draped over a ball, sleeping off: joint solver.",
            cloth,
            60,
            300,
        ),
        Benchmark::steps(
            "ragdolls",
            "200 ragdolls of 10 capsules and 9 limited revolute joints piling into a pit.",
            ragdolls,
            0,
            600,
        ),
        Benchmark::steps(
            "sleeping",
            "80 stacks of 12 boxes, all asleep: the cost of a world at rest.",
            sleeping,
            300,
            300,
        ),
        Benchmark {
            name: "queries",
            description: "1000 ray casts and 200 circle casts per iteration among 2000 fixed shapes.",
            build: scattered,
            warmup: 1,
            iterations: 300,
            iteration: Iteration::Custom(queries),
        },
    ]
}

/// A small deterministic random number generator (xorshift).
struct Random(u32);

impl Random {
    /// A number in `0.0..1.0`.
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        (self.0 >> 8) as f32 / (1 << 24) as f32
    }

    fn range(&mut self, low: f32, high: f32) -> f32 {
        low + (high - low) * self.next()
    }
}

/// A world without sleeping, so every body is simulated on every step.
fn awake_world() -> World {
    let mut world = World::new();
    world.set_sleeping_enabled(false);
    world
}

/// One of four shapes of about `size` meters, picked by `i`.
fn mixed_shape(i: usize, size: f32) -> ColliderDesc {
    match i % 4 {
        0 => ColliderDesc::circle(size),
        1 => ColliderDesc::rectangle(2.0 * size, 1.6 * size),
        2 => ColliderDesc::regular_polygon(3 + i / 4 % 4, 1.1 * size),
        _ => ColliderDesc::capsule(vec2(-0.6 * size, 0.0), vec2(0.6 * size, 0.0), 0.6 * size),
    }
}

/// A fixed box open at the top, `width` wide inside, with its floor at `y = 0`.
fn add_pit(world: &mut World, width: f32, height: f32) {
    let wall = |side: f32| ColliderDesc::rectangle(1.0, height).offset(vec2(side * (width + 1.0) / 2.0, height / 2.0));
    world.add_body(
        BodyDesc::fixed()
            .collider(ColliderDesc::rectangle(width + 2.0, 1.0).offset(vec2(0.0, -0.5)))
            .collider(wall(-1.0))
            .collider(wall(1.0)),
    );
}

fn pyramid() -> World {
    const BASE: usize = 40;
    let mut world = awake_world();
    add_pit(&mut world, 60.0, 1.0);
    for row in 0..BASE {
        let count = BASE - row;
        for column in 0..count {
            let x = (column as f32 - (count - 1) as f32 / 2.0) * 1.05;
            world.add_body(BodyDesc::dynamic().position(vec2(x, 0.5 + row as f32)).collider(ColliderDesc::square(1.0)));
        }
    }
    world
}

fn rain() -> World {
    let mut world = awake_world();
    add_pit(&mut world, 40.0, 40.0);
    let mut random = Random(7);
    for i in 0..2000 {
        let (column, row) = (i % 50, i / 50);
        let position = vec2(-19.6 + 0.8 * column as f32 + random.range(-0.1, 0.1), 2.0 + 0.8 * row as f32);
        world.add_body(
            BodyDesc::dynamic().position(position).angle(random.range(0.0, TAU)).collider(mixed_shape(i, 0.25)),
        );
    }
    world
}

fn tumbler() -> World {
    const SIZE: f32 = 24.0;
    const HALF_SIZE: f32 = SIZE / 2.0;
    let mut world = awake_world();
    let drum = world.add_body(BodyDesc::kinematic().angular_velocity(0.25));
    for (center, width, height) in [
        (vec2(0.0, -HALF_SIZE), SIZE, 0.6),
        (vec2(0.0, HALF_SIZE), SIZE, 0.6),
        (vec2(-HALF_SIZE, 0.0), 0.6, SIZE),
        (vec2(HALF_SIZE, 0.0), 0.6, SIZE),
    ] {
        world.add_collider(drum, ColliderDesc::rectangle(width, height).offset(center));
    }
    for i in 0..1500 {
        let (column, row) = (i % 40, i / 40);
        let position = vec2(-11.0 + 0.55 * column as f32, -11.0 + 0.55 * row as f32);
        world.add_body(BodyDesc::dynamic().position(position).collider(mixed_shape(i, 0.2)));
    }
    world
}

fn cloth() -> World {
    const SIZE: usize = 50;
    const SPACING: f32 = 0.3;
    let mut world = awake_world();
    add_pit(&mut world, 40.0, 1.0);
    world.add_body(BodyDesc::fixed().position(vec2(0.0, 5.0)).collider(ColliderDesc::circle(3.0)));

    let top_left = vec2(-SPACING * (SIZE - 1) as f32 / 2.0, 12.0);
    let mut particles = Vec::with_capacity(SIZE * SIZE);
    for row in 0..SIZE {
        for column in 0..SIZE {
            // Laid out flat, the cloth falls onto the ball, hanging from its two front corners.
            let position = top_left + SPACING * vec2(column as f32, -0.02 * row as f32);
            let pinned = row == 0 && (column == 0 || column == SIZE - 1);
            let desc = if pinned { BodyDesc::fixed() } else { BodyDesc::dynamic() };
            let particle = world.add_body(desc.position(position).collider(ColliderDesc::circle(0.08).density(20.0)));
            particles.push((particle, position));
        }
    }
    let mut link = |a: (BodyHandle, Vec2), b: (BodyHandle, Vec2)| {
        world.add_joint(DistanceJointDesc::new(a.0, b.0, a.1, b.1).rope(SPACING));
    };
    for row in 0..SIZE {
        for column in 0..SIZE {
            let here = particles[row * SIZE + column];
            if column + 1 < SIZE {
                link(here, particles[row * SIZE + column + 1]);
            }
            if row + 1 < SIZE {
                link(here, particles[(row + 1) * SIZE + column]);
            }
        }
    }
    world
}

fn ragdolls() -> World {
    let mut world = awake_world();
    add_pit(&mut world, 30.0, 80.0);
    for i in 0..200 {
        let (column, row) = (i % 20, i / 20);
        add_ragdoll(&mut world, vec2(-13.5 + 1.4 * column as f32, 1.0 + 2.5 * row as f32));
    }
    world
}

/// A ragdoll standing with its feet at `feet`, as in the testbed.
fn add_ragdoll(world: &mut World, feet: Vec2) {
    let mut part = |center: Vec2, collider: ColliderDesc| {
        world
            .add_body(BodyDesc::dynamic().position(feet + center).angular_damping(0.5).collider(collider.friction(0.6)))
    };
    let torso = part(vec2(0.0, 1.3), ColliderDesc::vertical_capsule(0.6, 0.15));
    let head = part(vec2(0.0, 1.82), ColliderDesc::circle(0.15));
    let limbs = [-1.0_f32, 1.0].map(|side| {
        let upper_arm = part(vec2(0.22 * side, 1.4), ColliderDesc::vertical_capsule(0.3, 0.05));
        let lower_arm = part(vec2(0.22 * side, 1.1), ColliderDesc::vertical_capsule(0.3, 0.045));
        let upper_leg = part(vec2(0.09 * side, 0.75), ColliderDesc::vertical_capsule(0.5, 0.07));
        let lower_leg = part(vec2(0.09 * side, 0.25), ColliderDesc::vertical_capsule(0.5, 0.06));
        (side, upper_arm, lower_arm, upper_leg, lower_leg)
    });
    let mut hinge = |a: BodyHandle, b: BodyHandle, point: Vec2, limits: std::ops::RangeInclusive<f32>| {
        world.add_joint(RevoluteJointDesc::new(a, b, feet + point).limits(limits));
    };
    hinge(torso, head, vec2(0.0, 1.65), -0.5..=0.5);
    for (side, upper_arm, lower_arm, upper_leg, lower_leg) in limbs {
        hinge(torso, upper_arm, vec2(0.22 * side, 1.55), -2.5..=2.5);
        hinge(upper_arm, lower_arm, vec2(0.22 * side, 1.25), 0.0..=2.4);
        hinge(torso, upper_leg, vec2(0.09 * side, 1.0), -0.6..=1.8);
        hinge(upper_leg, lower_leg, vec2(0.09 * side, 0.5), -2.2..=0.0);
    }
}

fn sleeping() -> World {
    let mut world = World::new();
    add_pit(&mut world, 180.0, 1.0);
    for stack in 0..80 {
        let x = -87.0 + 2.2 * stack as f32;
        for level in 0..12 {
            world.add_body(
                BodyDesc::dynamic()
                    .position(vec2(x, 0.4 + 0.8 * level as f32))
                    .collider(ColliderDesc::rectangle(1.2, 0.8)),
            );
        }
    }
    world
}

/// 2000 fixed shapes scattered over a 100 m square, for the query benchmark.
fn scattered() -> World {
    let mut world = World::new();
    let mut random = Random(11);
    for i in 0..2000 {
        let position = vec2(random.range(-50.0, 50.0), random.range(-50.0, 50.0));
        world.add_body(
            BodyDesc::fixed()
                .position(position)
                .angle(random.range(0.0, TAU))
                .collider(mixed_shape(i, random.range(0.2, 0.8))),
        );
    }
    world
}

fn queries(world: &mut World) -> u64 {
    let filter = QueryFilter::new();
    let mut random = Random(5);
    let mut checksum = 0_u64;
    for _ in 0..1000 {
        let origin = vec2(random.range(-50.0, 50.0), random.range(-50.0, 50.0));
        let translation = 30.0 * Vec2::from_angle(random.range(0.0, TAU));
        if let Some(hit) = world.cast_ray(origin, translation, filter) {
            checksum = checksum.rotate_left(5) ^ u64::from(hit.fraction.to_bits());
        }
    }
    let probe = Shape::from(Circle::new(0.3));
    for _ in 0..200 {
        let origin = vec2(random.range(-50.0, 50.0), random.range(-50.0, 50.0));
        let translation = 20.0 * Vec2::from_angle(random.range(0.0, TAU));
        let start = Transform::new(origin, Rot::IDENTITY);
        if let Some(hit) = world.cast_shape(probe.clone(), start, translation, filter) {
            checksum = checksum.rotate_left(5) ^ u64::from(hit.fraction.to_bits());
        }
    }
    checksum
}
