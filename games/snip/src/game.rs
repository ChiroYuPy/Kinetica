//! Game rules, independent of input and drawing so they can be tested.

use kinetica::prelude::*;

use crate::level::{Level, levels};

pub const TIME_STEP: f32 = 1.0 / 60.0;
pub const SWEET_RADIUS: f32 = 0.45;
pub const STAR_RADIUS: f32 = 0.45;
/// Inside size of the basket, and the thickness of its sides.
pub const BASKET_SIZE: Vec2 = vec2(2.6, 1.4);
pub const BASKET_WALL: f32 = 0.2;
pub const PAD_THICKNESS: f32 = 0.3;
pub const PADDLE_THICKNESS: f32 = 0.35;
/// Beads in each rope.
const BEADS: usize = 15;
/// Falling below this height loses the sweet.
const LOST_HEIGHT: f32 = -2.0;
/// How long a lost or won level shows before moving on, in seconds.
pub const END_DELAY: f32 = 1.5;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    Playing,
    /// The sweet landed in the basket this many seconds ago.
    Won(f32),
    /// The sweet fell off this many seconds ago.
    Lost(f32),
}

/// What can be cut: a rope made of beads, or an elastic band.
#[derive(Debug, Clone)]
pub struct Strand {
    /// The pin, the beads in order, and the sweet: each link joins two consecutive ones.
    pub bodies: Vec<BodyHandle>,
    /// `None` once cut.
    pub links: Vec<Option<JointHandle<DistanceJoint>>>,
    pub elastic: bool,
}

impl Strand {
    /// The two ends of each link still in one piece.
    pub fn intact_links<'a>(&'a self, world: &'a World) -> impl Iterator<Item = (Vec2, Vec2)> + 'a {
        self.links
            .iter()
            .zip(self.bodies.windows(2))
            .filter_map(|(link, pair)| link.map(|_| (world[pair[0]].position(), world[pair[1]].position())))
    }
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub level_index: usize,
    pub level: Level,
    pub sweet: BodyHandle,
    pub strands: Vec<Strand>,
    pub basket: BodyHandle,
    goal: ColliderHandle,
    pub stars: Vec<BodyHandle>,
    pub stars_collected: usize,
    pub pads: Vec<BodyHandle>,
    pub windmills: Vec<BodyHandle>,
    pub state: State,
    pub time: f32,
}

impl Game {
    pub fn new(level_index: usize) -> Self {
        let level = levels().swap_remove(level_index);
        let mut world = World::new();
        let sweet = world.add_body(
            BodyDesc::dynamic()
                .position(level.sweet)
                .collider(ColliderDesc::circle(SWEET_RADIUS).density(2.0).restitution(0.2)),
        );
        let mut strands: Vec<Strand> =
            level.ropes.iter().map(|rope| add_rope(&mut world, rope.pin, sweet, rope.slack)).collect();
        for elastic in &level.elastics {
            let pin = world.add_body(BodyDesc::fixed().position(elastic.pin));
            let band = DistanceJointDesc::new(pin, sweet, elastic.pin, level.sweet).spring(elastic.hertz, 0.3);
            strands.push(Strand { bodies: vec![pin, sweet], links: vec![Some(world.add_joint(band))], elastic: true });
        }
        let (basket, goal) = add_basket(&mut world, level.basket);
        let stars = level
            .stars
            .iter()
            .map(|&center| {
                world.add_body(BodyDesc::fixed().position(center).collider(ColliderDesc::circle(STAR_RADIUS).sensor()))
            })
            .collect();
        let pads = level
            .pads
            .iter()
            .map(|pad| {
                let shape = ColliderDesc::rectangle(pad.width, PAD_THICKNESS).restitution(1.0).friction(0.2);
                world.add_body(BodyDesc::fixed().position(pad.center).angle(pad.angle).collider(shape))
            })
            .collect();
        let windmills = level
            .windmills
            .iter()
            .map(|windmill| {
                let blade = |angle: f32| ColliderDesc::rectangle(windmill.length, PADDLE_THICKNESS).angle(angle);
                let desc = BodyDesc::kinematic().position(windmill.center).angular_velocity(windmill.speed);
                world.add_body(desc.collider(blade(0.0)).collider(blade(std::f32::consts::FRAC_PI_2)))
            })
            .collect();
        Self {
            world,
            level_index,
            level,
            sweet,
            strands,
            basket,
            goal,
            stars,
            stars_collected: 0,
            pads,
            windmills,
            state: State::Playing,
            time: 0.0,
        }
    }

    pub fn level_count() -> usize {
        levels().len()
    }

    /// Cuts every link crossing the segment from `from` to `to`, such as a mouse stroke.
    pub fn cut(&mut self, from: Vec2, to: Vec2) {
        if self.state != State::Playing {
            return;
        }
        for strand in &mut self.strands {
            for (link, pair) in strand.links.iter_mut().zip(strand.bodies.windows(2)) {
                let (a, b) = (self.world[pair[0]].position(), self.world[pair[1]].position());
                if link.is_some() && segments_cross(from, to, a, b) {
                    self.world.remove_joint(link.take().expect("checked above"));
                }
            }
        }
    }

    pub fn step(&mut self) {
        self.world.step(TIME_STEP);
        self.time += TIME_STEP;
        let sweet = self.sweet;
        let entered: Vec<SensorPair> = self
            .world
            .sensor_events()
            .iter()
            .filter_map(|event| match event {
                SensorEvent::Entered(pair) if pair.visitor_body == sweet => Some(*pair),
                _ => None,
            })
            .collect();
        for pair in entered {
            if pair.sensor == self.goal && self.state == State::Playing {
                self.state = State::Won(0.0);
            } else if let Some(index) = self.stars.iter().position(|&star| star == pair.sensor_body) {
                self.world.remove_body(self.stars.remove(index));
                self.stars_collected += 1;
            }
        }
        self.state = match self.state {
            State::Playing if self.world[sweet].position().y < LOST_HEIGHT => State::Lost(0.0),
            State::Playing => State::Playing,
            State::Won(since) => State::Won(since + TIME_STEP),
            State::Lost(since) => State::Lost(since + TIME_STEP),
        };
    }
}

/// A rope of light beads from a pin to the sweet. The beads collide with nothing, so the rope
/// passes through everything.
fn add_rope(world: &mut World, pin_position: Vec2, sweet: BodyHandle, slack: f32) -> Strand {
    let end = world[sweet].position();
    let pin = world.add_body(BodyDesc::fixed().position(pin_position));
    let mut bodies = vec![pin];
    for i in 1..=BEADS {
        let position = pin_position.lerp(end, i as f32 / (BEADS + 1) as f32);
        let bead = ColliderDesc::circle(0.06).density(0.5).layers(CollisionLayers::NONE);
        bodies.push(world.add_body(BodyDesc::dynamic().position(position).linear_damping(0.1).collider(bead)));
    }
    bodies.push(sweet);
    let links = bodies
        .windows(2)
        .map(|pair| {
            let (a, b) = (world[pair[0]].position(), world[pair[1]].position());
            let rope = DistanceJointDesc::new(pair[0], pair[1], a, b).rope(slack * a.distance(b));
            Some(world.add_joint(rope))
        })
        .collect();
    Strand { bodies, links, elastic: false }
}

/// A basket open at the top, its floor centered on `floor`, with a sensor inside; returns the
/// basket and the sensor.
fn add_basket(world: &mut World, floor: Vec2) -> (BodyHandle, ColliderHandle) {
    let (inside, wall) = (BASKET_SIZE, BASKET_WALL);
    let side_x = f32::midpoint(inside.x, wall);
    let side = |x: f32| ColliderDesc::rectangle(wall, inside.y + wall).offset(vec2(x, (inside.y - wall) / 2.0));
    let basket = world.add_body(
        BodyDesc::fixed()
            .position(floor)
            .collider(ColliderDesc::rectangle(inside.x + 2.0 * wall, wall).offset(vec2(0.0, -wall / 2.0)))
            .collider(side(-side_x))
            .collider(side(side_x)),
    );
    let goal = world.add_collider(
        basket,
        ColliderDesc::rectangle(inside.x, inside.y / 2.0).offset(vec2(0.0, inside.y / 4.0)).sensor(),
    );
    (basket, goal)
}

/// Whether segments `a0 a1` and `b0 b1` cross.
fn segments_cross(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> bool {
    let side = |p: Vec2, q0: Vec2, q1: Vec2| (q1 - q0).perp_dot(p - q0);
    side(b0, a0, a1) * side(b1, a0, a1) < 0.0 && side(a0, b0, b1) * side(a1, b0, b1) < 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Whether strand `index` still holds the sweet.
    fn holds(game: &Game, index: usize) -> bool {
        game.strands[index].links.iter().all(Option::is_some)
    }

    /// Plays until the level ends or `seconds` pass, cutting each strand next to its pin at its
    /// given time.
    fn play(level: usize, cut_times: &[f32], seconds: f32) -> Game {
        let mut game = Game::new(level);
        while game.time < seconds && game.state == State::Playing {
            for (index, &time) in cut_times.iter().enumerate() {
                if game.time >= time && holds(&game, index) {
                    let link = game.strands[index].links[0].take().expect("the strand holds");
                    game.world.remove_joint(link);
                }
            }
            game.step();
        }
        game
    }

    #[test]
    fn the_sweet_hangs_still_until_cut() {
        for index in 0..Game::level_count() {
            let game = play(index, &[], 5.0);
            assert_eq!(game.state, State::Playing, "{}", game.level.name);
        }
    }

    /// Tries every cut time, in tenths of a second, for each strand in turn.
    fn find_winning_cuts(level: usize) -> Option<Vec<f32>> {
        let strands = Game::new(level).strands.len();
        let times: Vec<f32> = (0..40).map(|i| 0.5 + 0.1 * i as f32).collect();
        let mut schedules: Vec<Vec<f32>> = vec![Vec::new()];
        for _ in 0..strands {
            schedules = schedules
                .into_iter()
                .flat_map(|schedule| {
                    let last = schedule.last().copied().unwrap_or(0.0);
                    times.iter().filter(move |&&time| time >= last).map(move |&time| {
                        let mut next = schedule.clone();
                        next.push(time);
                        next
                    })
                })
                .collect();
        }
        schedules.into_iter().find(|cuts| matches!(play(level, cuts, 12.0).state, State::Won(_)))
    }

    #[test]
    fn every_level_can_be_won() {
        for index in 0..Game::level_count() {
            assert!(find_winning_cuts(index).is_some(), "{}: no cuts win", levels()[index].name);
        }
    }

    #[test]
    fn a_stroke_cuts_the_ropes_it_crosses() {
        let mut game = Game::new(1);
        // A horizontal stroke just below the pins crosses both ropes.
        game.cut(vec2(0.0, 11.4), vec2(24.0, 11.4));
        assert!(!holds(&game, 0) && !holds(&game, 1));
        let mut game = Game::new(1);
        game.cut(vec2(0.0, 11.4), vec2(5.0, 11.4));
        assert!(!holds(&game, 0) && holds(&game, 1), "only the left rope is under the stroke");
    }

    #[test]
    fn stars_are_collected_on_the_way_down() {
        let game = play(0, &[0.5], 12.0);
        assert!(matches!(game.state, State::Won(_)));
        assert_eq!(game.stars_collected, 3);
    }
}
