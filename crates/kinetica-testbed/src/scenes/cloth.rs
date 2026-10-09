//! Cloth made of particles linked by ropes, and the scene showing it.

use kinetica::prelude::*;
use kinetica::{Color, DebugDraw};

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};
use kinetica_macroquad::DebugRenderer;

pub const SCENE: Scene = Scene {
    name: "Cloth",
    category: Category::SoftStructures,
    description: "Cloth is a grid of small balls linked by rope joints: it resists stretching but folds freely. \
                  Left, a curtain in the wind. Right, a hammock catching whatever falls on it.",
    camera_center: vec2(0.0, 6.0),
    view_height: 16.0,
    build: Build::Animated(build),
};

/// Particles are heavy for their size, so the cloth is not crushed by what lands on it.
const PARTICLE_DENSITY: f32 = 20.0;
const CURTAIN_COLOR: Color = Color::rgb(0.85, 0.35, 0.45);
const HAMMOCK_COLOR: Color = Color::rgb(0.35, 0.7, 0.9);
const DROP_PERIOD: f32 = 1.5;
const MAX_DROPS: usize = 8;

/// A rectangular grid of particles; neighbors are linked by ropes that may be torn.
pub struct Cloth {
    columns: usize,
    rows: usize,
    /// Longest distance allowed between neighbors.
    link_length: f32,
    /// Row by row, the first row at the top.
    particles: Vec<BodyHandle>,
    /// Links between a particle and its right neighbor, row by row; `None` once torn.
    horizontal: Vec<Option<JointHandle>>,
    /// Links between a particle and the one below it, row by row.
    vertical: Vec<Option<JointHandle>>,
    color: Color,
}

impl Cloth {
    /// Builds a cloth whose top-left particle is at `top_left`, with particles `spacing` apart.
    /// Links may stretch to `slack` times the spacing before pulling, so a cloth pinned on
    /// both sides can sag. `pinned(column, row)` tells which particles are fixed in place.
    pub fn new(
        world: &mut World,
        top_left: Vec2,
        (columns, rows): (usize, usize),
        (spacing, slack): (f32, f32),
        pinned: impl Fn(usize, usize) -> bool,
        color: Color,
    ) -> Self {
        let mut particles = Vec::with_capacity(columns * rows);
        for row in 0..rows {
            for column in 0..columns {
                let position = top_left + spacing * vec2(column as f32, -(row as f32));
                let desc = if pinned(column, row) { BodyDesc::fixed() } else { BodyDesc::dynamic() };
                let particle = world.add_body(desc.position(position).linear_damping(0.1));
                world.add_collider(
                    particle,
                    ColliderDesc::circle(0.3 * spacing).density(PARTICLE_DENSITY).friction(0.3),
                );
                particles.push(particle);
            }
        }
        let mut cloth = Self {
            columns,
            rows,
            link_length: slack * spacing,
            particles,
            horizontal: Vec::new(),
            vertical: Vec::new(),
            color,
        };
        for row in 0..rows {
            for column in 0..columns {
                if column + 1 < columns {
                    let link = cloth.link(world, (column, row), (column + 1, row));
                    cloth.horizontal.push(link);
                }
                if row + 1 < rows {
                    let link = cloth.link(world, (column, row), (column, row + 1));
                    cloth.vertical.push(link);
                }
            }
        }
        cloth
    }

    fn particle(&self, column: usize, row: usize) -> BodyHandle {
        self.particles[row * self.columns + column]
    }

    /// A rope: the particles may get closer, never further than the link length.
    fn link(&self, world: &mut World, a: (usize, usize), b: (usize, usize)) -> Option<JointHandle> {
        let (a, b) = (self.particle(a.0, a.1), self.particle(b.0, b.1));
        let (position_a, position_b) = (world.body(a)?.position(), world.body(b)?.position());
        let rope = DistanceJointDesc::between_world_points(world, a, b, position_a, position_b)
            .spring(0.0, 0.0)
            .limits(0.0..=self.link_length);
        Some(world.add_joint(rope))
    }

    pub fn particles(&self) -> &[BodyHandle] {
        &self.particles
    }

    /// The particles of the bottom row, from left to right.
    pub fn bottom_row(&self) -> &[BodyHandle] {
        &self.particles[(self.rows - 1) * self.columns..]
    }

    /// Breaks the links that pulled harder than `max_force` (in newtons) during the last step.
    /// Links to pinned particles are reinforced and never break.
    pub fn tear(&mut self, world: &mut World, max_force: f32) {
        let is_pinned = |body| world.body(body).is_some_and(|body| body.body_type() == BodyType::Fixed);
        let mut torn = Vec::new();
        for link in self.horizontal.iter_mut().chain(&mut self.vertical) {
            if let Some(joint) = link.and_then(|joint| world.joint(joint))
                && !is_pinned(joint.body_a())
                && !is_pinned(joint.body_b())
                && joint.constraint_force().length() > max_force
            {
                torn.extend(link.take());
            }
        }
        for joint in torn {
            world.remove_joint(joint);
        }
    }

    /// Cuts the links passing within `radius` of `point`.
    pub fn cut(&mut self, world: &mut World, point: Vec2, radius: f32) {
        let mut cut = Vec::new();
        for link in self.horizontal.iter_mut().chain(&mut self.vertical) {
            let Some(joint) = link.and_then(|joint| world.joint(joint)) else { continue };
            let (Some(a), Some(b)) = (world.body(joint.body_a()), world.body(joint.body_b())) else { continue };
            if distance_to_segment(point, a.position(), b.position()) < radius {
                cut.extend(link.take());
            }
        }
        for joint in cut {
            world.remove_joint(joint);
        }
    }

    /// Fills every cell whose four sides are still linked.
    pub fn draw(&self, world: &World, drawer: &mut DebugRenderer) {
        let position = |column, row| world.body(self.particle(column, row)).map(kinetica::Body::position);
        for row in 0..self.rows - 1 {
            for column in 0..self.columns - 1 {
                let top = self.horizontal[row * (self.columns - 1) + column];
                let bottom = self.horizontal[(row + 1) * (self.columns - 1) + column];
                let left = self.vertical[row * self.columns + column];
                let right = self.vertical[row * self.columns + column + 1];
                if [top, bottom, left, right].iter().any(Option::is_none) {
                    continue;
                }
                let corners = [
                    position(column, row),
                    position(column, row + 1),
                    position(column + 1, row + 1),
                    position(column + 1, row),
                ];
                if let [Some(a), Some(b), Some(c), Some(d)] = corners {
                    drawer.draw_polygon(&[a, b, c, d], self.color);
                }
            }
        }
    }
}

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 30.0);
    let curtain = Cloth::new(
        world,
        vec2(-10.5, 13.0),
        (22, 18),
        (0.3, 1.0),
        |column, row| row == 0 && column % 3 == 0,
        CURTAIN_COLOR,
    );
    let hammock =
        Cloth::new(world, vec2(1.0, 7.0), (34, 4), (0.3, 1.15), |column, _| column == 0 || column == 33, HAMMOCK_COLOR);
    Box::new(ClothDemo { curtain, hammock, drops: Vec::new(), next_drop: 1.0 })
}

struct ClothDemo {
    curtain: Cloth,
    hammock: Cloth,
    drops: Vec<BodyHandle>,
    next_drop: f32,
}

impl SceneUpdate for ClothDemo {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        // Gusts of wind blowing the curtain to the right.
        let wind = 5.0 + 3.0 * (0.7 * frame.time).sin() + 1.5 * (2.3 * frame.time).sin();
        for &particle in self.curtain.particles() {
            if let Some(body) = world.body_mut(particle) {
                let force = vec2(wind * body.mass(), 0.0);
                body.apply_force(force);
            }
        }

        if frame.time >= self.next_drop && self.drops.len() < MAX_DROPS {
            self.next_drop += DROP_PERIOD;
            let i = self.drops.len();
            let x = 4.0 + 4.0 * (i as f32 * 0.618_034).fract();
            let body = world.add_body(BodyDesc::dynamic().position(vec2(x, 12.0)).angular_velocity(1.0));
            let collider =
                if i.is_multiple_of(2) { ColliderDesc::circle(0.5) } else { ColliderDesc::rectangle(0.45, 0.45) };
            world.add_collider(body, collider.density(2.0));
            self.drops.push(body);
        }
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        self.curtain.draw(world, drawer);
        self.hammock.draw(world, drawer);
    }
}

fn distance_to_segment(point: Vec2, start: Vec2, end: Vec2) -> f32 {
    let along = end - start;
    let t = ((point - start).dot(along) / along.length_squared().max(f32::EPSILON)).clamp(0.0, 1.0);
    point.distance(start + t * along)
}
