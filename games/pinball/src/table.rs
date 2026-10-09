//! The table: walls, flippers, the plunger, bumpers, slingshots, rollover lanes and the drain.
//!
//! The playfield spans x from -4.4 to 4.4; the shooter lane, where the plunger launches the ball,
//! runs up its right side. The top of the table is a half circle bringing launched balls round
//! to the left.

use std::f32::consts::PI;

use kinetica::prelude::*;

pub const BALL_RADIUS: f32 = 0.25;
/// A steel ball: heavy for its size.
const BALL_DENSITY: f32 = 8.0;

const PLAYFIELD_LEFT: f32 = -4.4;
const PLAYFIELD_RIGHT: f32 = 4.4;
/// The shooter lane is between the divider and the right wall.
const LANE_LEFT: f32 = 4.5;
const LANE_RIGHT: f32 = 5.2;
const LANE_CENTER: f32 = f32::midpoint(LANE_LEFT, LANE_RIGHT);
/// Where the divider between the playfield and the lane stops.
const DIVIDER_TOP: f32 = 13.5;
/// Height where the side walls meet the half circle at the top.
const ARC_START: f32 = 16.0;
/// The side walls slope down from this height to the flipper pivots.
const FUNNEL_TOP: f32 = 4.4;

/// Pivot of the left flipper; the right one is its mirror image.
const FLIPPER_PIVOT: Vec2 = vec2(-2.5, 2.8);
const FLIPPER_LENGTH: f32 = 1.8;
const FLIPPER_RADIUS: f32 = 0.18;
/// How far below the horizontal a flipper rests, and how far up it swings, in radians.
const FLIPPER_REST_ANGLE: f32 = 0.5;
const FLIPPER_SWING: f32 = 1.0;
/// Flipper speed while moving, in rad/s, and the torque driving it, in N·m.
const FLIPPER_SPEED: f32 = 20.0;
const FLIPPER_TORQUE: f32 = 3000.0;

const PLUNGER_SIZE: Vec2 = vec2(0.6, 0.4);
/// Height of the plunger top at rest.
const PLUNGER_TOP: f32 = 1.6;
/// How far the plunger can be pulled down, in meters.
const PLUNGER_TRAVEL: f32 = 1.2;
const PLUNGER_PULL_SPEED: f32 = 2.0;
const PLUNGER_PULL_FORCE: f32 = 5000.0;
/// The spring pushing the plunger back up. It is longer than its room at rest, so it holds the
/// plunger against its stop.
const PLUNGER_SPRING_HERTZ: f32 = 4.0;
const PLUNGER_SPRING_PRELOAD: f32 = 0.2;
/// Where the spring is attached, below the plunger.
const PLUNGER_SPRING_LENGTH: f32 = 3.0;

/// Where a new ball is placed: on the plunger.
pub const BALL_START: Vec2 = vec2(LANE_CENTER, PLUNGER_TOP + BALL_RADIUS);

pub const BUMPER_RADIUS: f32 = 0.6;
const BUMPER_CENTERS: [Vec2; 3] = [vec2(-1.6, 12.6), vec2(1.6, 12.6), vec2(0.0, 10.4)];
const BUMPER_KICK_SPEED: f32 = 9.0;
const SLINGSHOT_KICK_SPEED: f32 = 7.0;
pub const BUMPER_POINTS: u32 = 100;
pub const SLINGSHOT_POINTS: u32 = 50;

pub const ROLLOVER_RADIUS: f32 = 0.3;
const ROLLOVER_Y: f32 = 17.6;
const ROLLOVER_XS: [f32; 3] = [-1.2, 0.0, 1.2];

/// Which side of the table a mirrored part is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    /// Mirrors a point given for the left side.
    fn place(self, left_point: Vec2) -> Vec2 {
        match self {
            Self::Left => left_point,
            Self::Right => vec2(-left_point.x, left_point.y),
        }
    }

    /// `1` on the left, `-1` on the right: a left flipper swings up counter-clockwise.
    fn sign(self) -> f32 {
        match self {
            Self::Left => 1.0,
            Self::Right => -1.0,
        }
    }
}

/// A bat turning on a pivot, driven up while its button is held and back down otherwise.
#[derive(Debug, Clone)]
pub struct Flipper {
    pub body: BodyHandle,
    hinge: JointHandle<RevoluteJoint>,
    side: Side,
}

impl Flipper {
    fn new(world: &mut World, table: BodyHandle, side: Side) -> Self {
        let pivot = side.place(FLIPPER_PIVOT);
        let tip = vec2(side.sign() * FLIPPER_LENGTH, 0.0);
        let body = world.add_body(
            BodyDesc::dynamic()
                .position(pivot)
                .angle(-side.sign() * FLIPPER_REST_ANGLE)
                .bullet()
                .collider(ColliderDesc::capsule(Vec2::ZERO, tip, FLIPPER_RADIUS).density(3.0).friction(0.4)),
        );
        let swing = side.sign() * FLIPPER_SWING;
        let hinge = world.add_joint(RevoluteJointDesc::new(table, body, pivot).limits(swing.min(0.0)..=swing.max(0.0)));
        let flipper = Self { body, hinge, side };
        flipper.actuate(world, false);
        flipper
    }

    /// Swings up while `pressed`, back down otherwise.
    pub fn actuate(&self, world: &mut World, pressed: bool) {
        let direction = if pressed { self.side.sign() } else { -self.side.sign() };
        world[self.hinge].set_motor(Some(Motor { speed: direction * FLIPPER_SPEED, max_force: FLIPPER_TORQUE }));
    }

    /// Whether the flipper is up, near the end of its swing.
    #[cfg(test)]
    pub fn is_up(&self, world: &World) -> bool {
        let rest = -self.side.sign() * FLIPPER_REST_ANGLE;
        (world[self.body].angle() - rest).abs() > 0.9 * FLIPPER_SWING
    }
}

/// The spring-loaded rod launching the ball up the shooter lane.
#[derive(Debug, Clone)]
pub struct Plunger {
    pub body: BodyHandle,
    slide: JointHandle<PrismaticJoint>,
}

impl Plunger {
    fn new(world: &mut World, table: BodyHandle) -> Self {
        let center = vec2(LANE_CENTER, PLUNGER_TOP - PLUNGER_SIZE.y / 2.0);
        let body = world.add_body(
            BodyDesc::dynamic()
                .position(center)
                .collider(ColliderDesc::rectangle(PLUNGER_SIZE.x, PLUNGER_SIZE.y).density(20.0)),
        );
        let slide =
            world.add_joint(PrismaticJointDesc::new(table, body, center, Vec2::Y).limits(-PLUNGER_TRAVEL..=0.0));
        let spring_base = center - vec2(0.0, PLUNGER_SPRING_LENGTH);
        world.add_joint(
            DistanceJointDesc::new(table, body, spring_base, center)
                .spring(PLUNGER_SPRING_HERTZ, 0.1)
                .length(PLUNGER_SPRING_LENGTH + PLUNGER_SPRING_PRELOAD),
        );
        Self { body, slide }
    }

    /// Pulls the plunger down while `pulling`; once released, the spring sends it back up.
    pub fn pull(&self, world: &mut World, pulling: bool) {
        let motor = pulling.then_some(Motor { speed: -PLUNGER_PULL_SPEED, max_force: PLUNGER_PULL_FORCE });
        world[self.slide].set_motor(motor);
    }

    /// How far the plunger is pulled, from 0 (at rest) to 1 (all the way).
    pub fn tension(&self, world: &World) -> f32 {
        let top = world[self.body].position().y + PLUNGER_SIZE.y / 2.0;
        ((PLUNGER_TOP - top) / PLUNGER_TRAVEL).clamp(0.0, 1.0)
    }
}

/// How a kicker pushes the ball away.
#[derive(Debug, Clone, Copy)]
pub enum Push {
    /// Away from a point: pop bumpers.
    AwayFrom(Vec2),
    /// Along a fixed direction: the rubber face of a slingshot.
    Along(Vec2),
}

/// A part that kicks the ball when hit, and scores.
#[derive(Debug, Clone)]
pub struct Kicker {
    pub collider: ColliderHandle,
    push: Push,
    speed: f32,
    pub points: u32,
}

impl Kicker {
    /// Sends the ball away at the kicker speed at least.
    pub fn kick(&self, world: &mut World, ball: BodyHandle) {
        let ball = &mut world[ball];
        let direction = match self.push {
            Push::AwayFrom(center) => (ball.position() - center).normalize_or_zero(),
            Push::Along(direction) => direction,
        };
        let velocity = ball.linear_velocity();
        let missing_speed = (self.speed - velocity.dot(direction)).max(0.0);
        ball.set_linear_velocity(velocity + missing_speed * direction);
    }
}

#[derive(Debug, Clone)]
pub struct Table {
    /// The wall around the table, as a line, to draw it.
    pub outline: Vec<Vec2>,
    /// Solid parts inside the table: the lane divider and gate, slingshots and lane posts.
    pub walls: Vec<ColliderHandle>,
    pub flippers: [Flipper; 2],
    pub plunger: Plunger,
    /// Pop bumpers, then slingshots.
    pub kickers: Vec<Kicker>,
    /// Sensors across the lanes at the top: lighting all three scores a bonus.
    pub rollovers: [ColliderHandle; 3],
    /// A sensor below the flippers, catching lost balls.
    pub drain: ColliderHandle,
}

impl Table {
    pub fn new(world: &mut World) -> Self {
        let body = world.add_body(BodyDesc::fixed());
        let outline = outline();
        world.add_chain(body, ChainDesc::new(outline.clone()).friction(0.3).restitution(0.4));

        let mut walls = Vec::new();
        let divider = ColliderDesc::rectangle(LANE_LEFT - PLAYFIELD_RIGHT, DIVIDER_TOP)
            .offset(vec2(f32::midpoint(PLAYFIELD_RIGHT, LANE_LEFT), DIVIDER_TOP / 2.0));
        walls.push(world.add_collider(body, divider));
        walls.push(world.add_collider(body, lane_gate()));
        // Posts on both sides of each rollover, making three lanes.
        for x in [-1.8, -0.6, 0.6, 1.8] {
            let post = ColliderDesc::capsule(vec2(x, ROLLOVER_Y - 0.5), vec2(x, ROLLOVER_Y + 0.5), 0.08);
            walls.push(world.add_collider(body, post));
        }

        let mut kickers = Vec::new();
        for center in BUMPER_CENTERS {
            let bumper = ColliderDesc::circle(BUMPER_RADIUS).offset(center).restitution(0.5);
            let collider = world.add_collider(body, bumper);
            kickers.push(Kicker {
                collider,
                push: Push::AwayFrom(center),
                speed: BUMPER_KICK_SPEED,
                points: BUMPER_POINTS,
            });
        }
        for side in [Side::Left, Side::Right] {
            let (wall, kicker) = slingshot(world, body, side);
            walls.push(wall);
            kickers.push(kicker);
        }

        let rollovers = ROLLOVER_XS.map(|x| {
            world.add_collider(body, ColliderDesc::circle(ROLLOVER_RADIUS).offset(vec2(x, ROLLOVER_Y)).sensor())
        });
        let drain = world.add_collider(body, ColliderDesc::rectangle(12.0, 1.0).offset(vec2(0.0, -1.5)).sensor());

        let flippers = [Flipper::new(world, body, Side::Left), Flipper::new(world, body, Side::Right)];
        let plunger = Plunger::new(world, body);
        Self { outline, walls, flippers, plunger, kickers, rollovers, drain }
    }

    /// Adds a ball at `position`.
    pub fn add_ball(world: &mut World, position: Vec2) -> BodyHandle {
        let ball = ColliderDesc::circle(BALL_RADIUS).density(BALL_DENSITY).friction(0.2);
        world.add_body(BodyDesc::dynamic().position(position).bullet().collider(ball))
    }
}

/// The outer wall, counter-clockwise so that it faces inwards: from the right flipper up the
/// shooter lane, round the top and down to the left flipper. The gap between the flippers is
/// the drain.
fn outline() -> Vec<Vec2> {
    let mut points = vec![
        Side::Right.place(FLIPPER_PIVOT),
        vec2(PLAYFIELD_RIGHT, FUNNEL_TOP),
        // Down behind the divider to the bottom of the lane.
        vec2(PLAYFIELD_RIGHT, 0.0),
        vec2(LANE_RIGHT, 0.0),
    ];
    points.extend(top_arc());
    points.extend([vec2(PLAYFIELD_LEFT, FUNNEL_TOP), FLIPPER_PIVOT]);
    points
}

/// The half circle at the top of the table, counter-clockwise from the right wall to the left one.
fn top_arc() -> impl Iterator<Item = Vec2> {
    let center = vec2(f32::midpoint(PLAYFIELD_LEFT, LANE_RIGHT), ARC_START);
    let radius = (LANE_RIGHT - PLAYFIELD_LEFT) / 2.0;
    let segments = 24;
    (0..=segments).map(move |i| center + radius * Vec2::from_angle(PI * i as f32 / segments as f32))
}

/// The floor of the table as convex pieces, to draw it: the half circle at the top, the
/// rectangle below it, the bottom of the shooter lane, and the funnel down to the flippers.
pub fn floor() -> Vec<Vec<Vec2>> {
    let rectangle = |min: Vec2, max: Vec2| vec![min, vec2(max.x, min.y), max, vec2(min.x, max.y)];
    let funnel = vec![
        FLIPPER_PIVOT,
        Side::Right.place(FLIPPER_PIVOT),
        vec2(PLAYFIELD_RIGHT, FUNNEL_TOP),
        vec2(PLAYFIELD_LEFT, FUNNEL_TOP),
    ];
    vec![
        top_arc().collect(),
        rectangle(vec2(PLAYFIELD_LEFT, FUNNEL_TOP), vec2(LANE_RIGHT, ARC_START)),
        rectangle(vec2(PLAYFIELD_RIGHT, 0.0), vec2(LANE_RIGHT, FUNNEL_TOP)),
        funnel,
    ]
}

/// A one-way gate at the top of the shooter lane: launched balls go through it, and balls
/// falling back land on it and roll into the playfield.
fn lane_gate() -> ColliderDesc {
    let (low, high) = (vec2(PLAYFIELD_RIGHT, 14.0), vec2(LANE_RIGHT, 14.6));
    let along = high - low;
    let normal = along.perp().normalize();
    ColliderDesc::rectangle(along.length(), 0.08)
        .offset((low + high) / 2.0)
        .angle(along.y.atan2(along.x))
        .one_way(normal)
}

/// A triangle above a flipper whose rubber face, towards the middle, kicks the ball.
fn slingshot(world: &mut World, table: BodyHandle, side: Side) -> (ColliderHandle, Kicker) {
    let [bottom, top, inner] = [vec2(-3.7, 4.6), vec2(-3.7, 6.6), vec2(-2.8, 4.3)].map(|point| side.place(point));
    let triangle = ColliderDesc::convex_hull(&[bottom, top, inner]).expect("a triangle");
    let wall = world.add_collider(table, triangle);

    // The face from the top to the inner corner, facing up and towards the middle.
    let face = inner - top;
    let normal = side.sign() * face.perp().normalize();
    let rubber = ColliderDesc::segment(top + 0.02 * normal, inner + 0.02 * normal);
    let collider = world.add_collider(table, rubber);
    let kicker = Kicker { collider, push: Push::Along(normal), speed: SLINGSHOT_KICK_SPEED, points: SLINGSHOT_POINTS };
    (wall, kicker)
}
