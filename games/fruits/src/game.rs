//! Game rules, independent of input and drawing so they can be tested.

use kinetica::prelude::*;

pub const TIME_STEP: f32 = 1.0 / 60.0;
/// Radius of each kind of fruit, from the smallest. Two alike merge into the next one.
pub const RADII: [f32; 9] = [0.28, 0.38, 0.5, 0.62, 0.76, 0.92, 1.1, 1.32, 1.6];
/// Points for making each kind of fruit, from the second: bigger ones are worth more.
const POINTS: [u32; 8] = [1, 3, 6, 10, 15, 21, 28, 36];
/// Only the smallest kinds are dropped; the others are made by merging.
const DROPPED_KINDS: usize = 4;
/// Inside size of the jar, and the thickness of its sides.
pub const JAR_SIZE: Vec2 = vec2(7.0, 9.0);
pub const JAR_WALL: f32 = 0.4;
/// Fruits are dropped from this height.
pub const DROP_HEIGHT: f32 = 10.2;
/// A fruit resting above this line for [`OVERFLOW_TIME`] ends the game.
pub const TOP_LINE: f32 = JAR_SIZE.y;
const OVERFLOW_TIME: f32 = 2.0;
/// A fruit just dropped is still falling: it does not count as overflowing for this long.
const FALL_TIME: f32 = 1.0;
/// Time between two drops, in seconds.
const DROP_DELAY: f32 = 0.5;

#[derive(Debug, Clone, Copy)]
pub struct Fruit {
    pub body: BodyHandle,
    /// Index in [`RADII`].
    pub kind: usize,
    /// Seconds since it appeared.
    age: f32,
}

#[derive(Clone)]
pub struct Game {
    pub world: World,
    pub fruits: Vec<Fruit>,
    pub jar: BodyHandle,
    /// The kind of fruit held, and the one after it.
    pub next: usize,
    pub after: usize,
    pub score: u32,
    pub game_over: bool,
    /// How long fruits have been above the top line, in seconds.
    pub overflow: f32,
    drop_cooldown: f32,
    random: u32,
}

impl Game {
    pub fn new(seed: u32) -> Self {
        let mut world = World::new();
        let (size, wall) = (JAR_SIZE, JAR_WALL);
        let side = |x: f32| ColliderDesc::rectangle(wall, size.y + wall).offset(vec2(x, (size.y - wall) / 2.0));
        let jar = world.add_body(
            BodyDesc::fixed()
                .collider(ColliderDesc::rectangle(size.x + 2.0 * wall, wall).offset(vec2(0.0, -wall / 2.0)))
                .collider(side(-f32::midpoint(size.x, wall)))
                .collider(side(f32::midpoint(size.x, wall))),
        );
        let mut game = Self {
            world,
            fruits: Vec::new(),
            jar,
            next: 0,
            after: 0,
            score: 0,
            game_over: false,
            overflow: 0.0,
            drop_cooldown: 0.0,
            random: seed | 1,
        };
        game.next = game.random_kind();
        game.after = game.random_kind();
        game
    }

    /// A random kind among the dropped ones (xorshift: simple and repeatable).
    fn random_kind(&mut self) -> usize {
        self.random ^= self.random << 13;
        self.random ^= self.random >> 17;
        self.random ^= self.random << 5;
        self.random as usize % DROPPED_KINDS
    }

    pub fn can_drop(&self) -> bool {
        !self.game_over && self.drop_cooldown <= 0.0
    }

    /// Where a fruit dropped at `x` actually starts: inside the jar.
    pub fn drop_x(&self, x: f32) -> f32 {
        let limit = JAR_SIZE.x / 2.0 - RADII[self.next];
        x.clamp(-limit, limit)
    }

    /// Drops the held fruit above `x`, if the last one had time to get going.
    pub fn drop(&mut self, x: f32) {
        if !self.can_drop() {
            return;
        }
        let position = vec2(self.drop_x(x), DROP_HEIGHT);
        let kind = self.next;
        self.add_fruit(kind, position, Vec2::ZERO);
        self.next = self.after;
        self.after = self.random_kind();
        self.drop_cooldown = DROP_DELAY;
    }

    fn add_fruit(&mut self, kind: usize, position: Vec2, velocity: Vec2) {
        let shape = ColliderDesc::circle(RADII[kind]).friction(0.4).restitution(0.1);
        let body =
            self.world.add_body(BodyDesc::dynamic().position(position).linear_velocity(velocity).collider(shape));
        self.fruits.push(Fruit { body, kind, age: 0.0 });
    }

    pub fn step(&mut self) {
        if self.game_over {
            return;
        }
        self.world.step(TIME_STEP);
        self.drop_cooldown -= TIME_STEP;
        for fruit in &mut self.fruits {
            fruit.age += TIME_STEP;
        }
        self.merge();

        let world = &self.world;
        let overflowing = self
            .fruits
            .iter()
            .any(|fruit| fruit.age > FALL_TIME && world[fruit.body].position().y + RADII[fruit.kind] > TOP_LINE);
        self.overflow = if overflowing { self.overflow + TIME_STEP } else { 0.0 };
        self.game_over = self.overflow > OVERFLOW_TIME;
    }

    /// Replaces every two touching fruits of the same kind by one of the next kind, between them.
    fn merge(&mut self) {
        let kind_of = |body: BodyHandle| self.fruits.iter().position(|fruit| fruit.body == body);
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for contact in self.world.contacts() {
            let pair = contact.pair();
            let (Some(a), Some(b)) = (kind_of(pair.body_a), kind_of(pair.body_b)) else { continue };
            let kind = self.fruits[a].kind;
            let taken = pairs.iter().any(|&(c, d)| [a, b].iter().any(|&i| i == c || i == d));
            if kind == self.fruits[b].kind && kind + 1 < RADII.len() && !taken {
                pairs.push((a, b));
            }
        }
        let mut gone = Vec::new();
        for (a, b) in pairs {
            let (a, b) = (self.fruits[a], self.fruits[b]);
            let (body_a, body_b) = (&self.world[a.body], &self.world[b.body]);
            let position = (body_a.position() + body_b.position()) / 2.0;
            let velocity = (body_a.linear_velocity() + body_b.linear_velocity()) / 2.0;
            self.world.remove_body(a.body);
            self.world.remove_body(b.body);
            gone.extend([a.body, b.body]);
            self.add_fruit(a.kind + 1, position, velocity);
            self.score += POINTS[a.kind];
        }
        self.fruits.retain(|fruit| !gone.contains(&fruit.body));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(game: &mut Game, steps: usize) {
        for _ in 0..steps {
            game.step();
        }
    }

    #[test]
    fn two_alike_fruits_merge_into_the_next_kind() {
        let mut game = Game::new(1);
        game.next = 2;
        game.after = 2;
        game.drop(0.0);
        run(&mut game, 60);
        game.drop(0.0);
        run(&mut game, 120);
        assert_eq!(game.fruits.len(), 1);
        assert_eq!(game.fruits[0].kind, 3);
        assert_eq!(game.score, 6);
        // The new fruit rests on the floor of the jar.
        let y = game.world[game.fruits[0].body].position().y;
        assert!((y - RADII[3]).abs() < 0.05, "the merged fruit is at y = {y}");
    }

    /// Dropping fruits anywhere: they stay in the jar, merge now and then, and the jar ends
    /// up full.
    #[test]
    fn a_careless_player_fills_the_jar() {
        let mut game = Game::new(7);
        let mut drops = 0;
        while !game.game_over && drops < 400 {
            // Golden ratio steps spread the drops across the jar.
            let x = JAR_SIZE.x * ((drops as f32 * 0.618_034).fract() - 0.5);
            if game.can_drop() {
                game.drop(x);
                drops += 1;
            }
            game.step();
            for fruit in &game.fruits {
                let position = game.world[fruit.body].position();
                assert!(position.x.abs() < JAR_SIZE.x / 2.0 && position.y > 0.0, "a fruit got out: {position}");
            }
        }
        assert!(game.game_over, "the jar never filled up");
        assert!(game.score > 100, "only {} points", game.score);
        assert!(game.fruits.iter().any(|fruit| fruit.kind >= 5), "nothing big was ever made");
    }
}
