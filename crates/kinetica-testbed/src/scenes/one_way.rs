use kinetica::prelude::*;
use macroquad::input::{KeyCode, is_key_down};

use super::{Build, Category, Frame, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "One-way platforms",
    category: Category::GameFeatures,
    description: "Left/Right to run, Up to jump. Platforms only block from above: jump through them from \
                  below. A sensor under the feet tells whether the character stands on something.",
    camera_center: vec2(0.0, 4.0),
    view_height: 12.0,
    build: Build::Animated(build),
};

const RUN_SPEED: f32 = 6.0;
/// How quickly the horizontal speed reaches the target, in 1/s.
const RUN_RESPONSE: f32 = 12.0;
const JUMP_SPEED: f32 = 9.0;

fn build(world: &mut World) -> Box<dyn SceneUpdate> {
    add_ground(world, 20.0);
    let level = world.add_body(BodyDesc::fixed());
    for (center, half_width, angle) in [
        (vec2(-4.0, 2.0), 2.0, 0.0),
        (vec2(1.5, 3.5), 2.0, 0.0),
        (vec2(-2.0, 5.5), 1.5, 0.0),
        (vec2(4.5, 6.5), 2.0, 0.15),
        (vec2(0.0, 8.5), 3.0, 0.0),
    ] {
        world.add_collider(level, ColliderDesc::offset_rectangle(half_width, 0.1, center, angle).one_way(Vec2::Y));
    }
    for x in [-20.0, 20.0] {
        world.add_collider(level, ColliderDesc::offset_rectangle(0.5, 10.0, vec2(x, 10.0), 0.0));
    }
    // A few crates resting on the platforms.
    for position in [vec2(-4.5, 3.0), vec2(1.0, 4.5), vec2(0.5, 9.5)] {
        let crate_body = world.add_body(BodyDesc::dynamic().position(position));
        world.add_collider(crate_body, ColliderDesc::rectangle(0.35, 0.35));
    }

    Box::new(add_player(world, vec2(-8.0, 0.6)))
}

fn add_player(world: &mut World, position: Vec2) -> Player {
    let player = world.add_body(BodyDesc::dynamic().position(position).fixed_rotation(true));
    // No friction, so the character does not stick to walls; running sets the speed directly.
    // A capsule: its round bottom glides over edges instead of catching on them.
    world.add_collider(player, ColliderDesc::vertical_capsule(0.25, 0.3).friction(0.0));
    let feet =
        world.add_collider(player, ColliderDesc::offset_rectangle(0.25, 0.08, vec2(0.0, -0.55), 0.0).sensor(true));
    Player { body: player, feet }
}

struct Player {
    body: BodyHandle,
    feet: ColliderHandle,
}

impl Player {
    /// Runs left (`-1`), right (`1`) or stops (`0`), and jumps if asked while standing.
    fn control(&self, world: &mut World, run: f32, jump: bool) {
        let on_ground = world.sensor_overlaps(self.feet).next().is_some();
        let Some(body) = world.body_mut(self.body) else { return };
        let mut velocity = body.linear_velocity();
        velocity.x += (run * RUN_SPEED - velocity.x) * (RUN_RESPONSE / 60.0).min(1.0);
        // Only jump when standing, not while passing up through a platform.
        if jump && on_ground && velocity.y <= 0.5 {
            velocity.y = JUMP_SPEED;
        }
        if velocity != body.linear_velocity() {
            body.set_linear_velocity(velocity);
        }
    }
}

impl SceneUpdate for Player {
    fn update(&mut self, world: &mut World, _frame: &Frame) {
        let run = f32::from(i8::from(is_key_down(KeyCode::Right)) - i8::from(is_key_down(KeyCode::Left)));
        self.control(world, run, is_key_down(KeyCode::Up));
    }

    fn camera_target(&self, world: &World) -> Option<Vec2> {
        world.body(self.body).map(|body| body.position() + vec2(0.0, 2.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(world: &mut World, player: &Player, steps: usize, run: f32, jump: bool) {
        for _ in 0..steps {
            player.control(world, run, jump);
            world.step(1.0 / 60.0);
        }
    }

    fn position(world: &World, player: &Player) -> Vec2 {
        world.body(player.body).unwrap().position()
    }

    #[test]
    fn player_jumps_through_a_platform_and_lands_on_it() {
        let mut world = World::new();
        build(&mut world);
        // Under the right end of the first platform, whose top is at y = 2.1.
        let player = add_player(&mut world, vec2(-3.0, 0.6));
        run(&mut world, &player, 30, 0.0, false);
        run(&mut world, &player, 1, 0.0, true);
        run(&mut world, &player, 120, 0.0, false);
        let y = position(&world, &player).y;
        assert!((y - 2.65).abs() < 0.03, "the player should stand on the platform, at y = {y}");
    }

    #[test]
    fn player_cannot_jump_in_the_air() {
        let mut world = World::new();
        build(&mut world);
        let player = add_player(&mut world, vec2(8.0, 5.0));
        run(&mut world, &player, 20, 0.0, true);
        assert!(world.body(player.body).unwrap().linear_velocity().y < 0.0, "the player jumped mid-air");
    }

    #[test]
    fn player_runs_off_a_platform_and_falls() {
        let mut world = World::new();
        build(&mut world);
        let player = add_player(&mut world, vec2(-3.0, 2.7));
        run(&mut world, &player, 120, 1.0, false);
        let position = position(&world, &player);
        assert!(position.x > 0.0 && position.y < 0.7, "the player should have dropped to the ground: {position}");
    }
}
