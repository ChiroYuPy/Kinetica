//! Cloth made of particles linked by ropes that can tear, and a scene showing it.

use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, Spawner, add_ground, scatter};

pub const SCENE: Scene = Scene {
    name: "Cloth",
    category: Category::SoftStructures,
    description: "Cloth is a grid of small balls linked by ropes: it resists stretching but folds freely. This \
                  hammock is fixed at both ends.",
    camera_center: vec2(0.0, 5.0),
    view_height: 12.0,
    build: Build::Animated(build),
};

/// Particles are heavy for their size, so the cloth is not crushed by what lands on it.
const PARTICLE_DENSITY: f32 = 20.0;
const HAMMOCK_COLOR: Color = Color::new(0.35, 0.7, 0.9, 1.0);

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    // Links 15 % longer than the spacing, so the hammock sags.
    let pinned = |column, _| column == 0 || column == 33;
    let hammock = Cloth::new(world, vec2(-4.95, 7.0), (34, 4), (0.3, 1.15), pinned, HAMMOCK_COLOR);
    Box::new(Hammock { hammock, drops: Spawner::new(1.5, 8) })
}

struct Hammock {
    hammock: Cloth,
    drops: Spawner,
}

impl SceneUpdate for Hammock {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        self.drops.update(world, frame.time, |i| {
            let shape = if i.is_multiple_of(2) { ColliderDesc::circle(0.5) } else { ColliderDesc::square(0.9) };
            let position = vec2(-2.0 + 4.0 * scatter(i), 12.0);
            BodyDesc::dynamic().position(position).angular_velocity(1.0).collider(shape.density(2.0))
        });
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        self.hammock.draw(world, drawer);
    }
}

/// A rectangular grid of particles; neighbors are linked by ropes that may be torn.
pub struct Cloth {
    columns: usize,
    rows: usize,
    /// Longest distance allowed between neighbors.
    link_length: f32,
    /// Row by row, the first row at the top.
    particles: Vec<BodyHandle>,
    /// Links between a particle and its right neighbor, row by row; `None` once torn.
    horizontal: Vec<Option<JointHandle<DistanceJoint>>>,
    /// Links between a particle and the one below it, row by row.
    vertical: Vec<Option<JointHandle<DistanceJoint>>>,
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
                let ball = ColliderDesc::circle(0.3 * spacing).density(PARTICLE_DENSITY).friction(0.3);
                particles.push(world.add_body(desc.position(position).linear_damping(0.1).collider(ball)));
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
                    cloth.horizontal.push(Some(link));
                }
                if row + 1 < rows {
                    let link = cloth.link(world, (column, row), (column, row + 1));
                    cloth.vertical.push(Some(link));
                }
            }
        }
        cloth
    }

    fn particle(&self, column: usize, row: usize) -> BodyHandle {
        self.particles[row * self.columns + column]
    }

    /// A rope: the particles may get closer, never further than the link length.
    fn link(&self, world: &mut World, a: (usize, usize), b: (usize, usize)) -> JointHandle<DistanceJoint> {
        let (a, b) = (self.particle(a.0, a.1), self.particle(b.0, b.1));
        let rope = DistanceJointDesc::new(a, b, world[a].position(), world[b].position()).rope(self.link_length);
        world.add_joint(rope)
    }

    /// The particles of the bottom row, from left to right.
    pub fn bottom_row(&self) -> &[BodyHandle] {
        &self.particles[(self.rows - 1) * self.columns..]
    }

    /// Breaks the links that pulled harder than `max_force` (in newtons) during the last step.
    /// Links to pinned particles are reinforced and never break.
    pub fn tear(&mut self, world: &mut World, max_force: f32) {
        let is_pinned = |body: BodyHandle| world[body].body_type() == BodyType::Fixed;
        let mut torn = Vec::new();
        for link in self.horizontal.iter_mut().chain(&mut self.vertical) {
            if let Some(joint) = link.map(|joint| &world[joint])
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
            let Some(joint) = link.map(|joint| &world[joint]) else { continue };
            let (a, b) = (world[joint.body_a()].position(), world[joint.body_b()].position());
            if distance_to_segment(point, a, b) < radius {
                cut.extend(link.take());
            }
        }
        for joint in cut {
            world.remove_joint(joint);
        }
    }

    /// Fills every cell whose four sides are still linked.
    pub fn draw(&self, world: &World, drawer: &mut DebugRenderer) {
        let position = |column, row| world[self.particle(column, row)].position();
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
                drawer.draw_polygon(&corners, self.color);
            }
        }
    }
}

fn distance_to_segment(point: Vec2, start: Vec2, end: Vec2) -> f32 {
    let along = end - start;
    let t = ((point - start).dot(along) / along.length_squared().max(f32::EPSILON)).clamp(0.0, 1.0);
    point.distance(start + t * along)
}
