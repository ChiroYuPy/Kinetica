//! The levels: towers of wood and stone protecting targets.

use kinetica::prelude::*;

/// What a block is made of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    Wood,
    Stone,
}

impl Material {
    fn collider(self, width: f32, height: f32) -> ColliderDesc {
        let desc = ColliderDesc::rectangle(width, height);
        match self {
            Self::Wood => desc.density(0.6).friction(0.7),
            Self::Stone => desc.density(2.4).friction(0.8),
        }
    }
}

/// The bodies of a level, as the game needs to know them.
#[derive(Debug, Clone, Default)]
pub struct Level {
    pub ground: Option<BodyHandle>,
    pub blocks: Vec<(BodyHandle, Material)>,
    pub targets: Vec<BodyHandle>,
    pub shots: u32,
}

/// Number of levels.
pub const LEVEL_COUNT: usize = 3;

/// Radius of the targets.
pub const TARGET_RADIUS: f32 = 0.35;

/// Thickness of posts and beams.
const THICKNESS: f32 = 0.24;

/// Builds level `index` into an empty world.
pub fn build(world: &mut World, index: usize) -> Level {
    let mut builder = Builder { world, level: Level::default() };
    builder.ground();
    match index {
        0 => builder.first(),
        1 => builder.second(),
        _ => builder.third(),
    }
    builder.level
}

struct Builder<'a> {
    world: &'a mut World,
    level: Level,
}

impl Builder<'_> {
    fn ground(&mut self) {
        let ground =
            BodyDesc::fixed().position(vec2(15.0, -1.0)).collider(ColliderDesc::rectangle(80.0, 2.0).friction(0.9));
        self.level.ground = Some(self.world.add_body(ground));
    }

    fn block(&mut self, material: Material, center: Vec2, width: f32, height: f32) {
        let body = self.world.add_body(BodyDesc::dynamic().position(center).collider(material.collider(width, height)));
        self.level.blocks.push((body, material));
    }

    /// A vertical post standing on `floor`.
    fn post(&mut self, material: Material, x: f32, floor: f32, height: f32) {
        self.block(material, vec2(x, floor + height / 2.0), THICKNESS, height);
    }

    /// A horizontal beam lying on `floor`.
    fn beam(&mut self, material: Material, x: f32, floor: f32, width: f32) {
        self.block(material, vec2(x, floor + THICKNESS / 2.0), width, THICKNESS);
    }

    fn target(&mut self, x: f32, floor: f32) {
        let ball = ColliderDesc::circle(TARGET_RADIUS).density(0.8).friction(0.8);
        let body = self.world.add_body(BodyDesc::dynamic().position(vec2(x, floor + TARGET_RADIUS)).collider(ball));
        self.level.targets.push(body);
    }

    /// A one-storey hut of posts and a beam, with a target inside.
    fn hut(&mut self, material: Material, x: f32, floor: f32) -> f32 {
        self.post(material, x - 0.9, floor, 1.6);
        self.post(material, x + 0.9, floor, 1.6);
        self.beam(material, x, floor + 1.6, 2.3);
        self.target(x, floor);
        floor + 1.6 + THICKNESS
    }

    /// One hut with a target on its roof.
    fn first(&mut self) {
        let roof = self.hut(Material::Wood, 14.0, 0.0);
        self.target(14.0, roof);
        self.level.shots = 3;
    }

    /// Two towers of two storeys.
    fn second(&mut self) {
        for (x, material) in [(12.0, Material::Wood), (17.0, Material::Wood)] {
            let first = self.hut(material, x, 0.0);
            let second = self.hut(material, x, first);
            self.target(x, second);
        }
        self.level.shots = 4;
    }

    /// A stone bunker under a wooden tower.
    fn third(&mut self) {
        let floor = self.hut(Material::Stone, 15.0, 0.0);
        let floor = self.hut(Material::Wood, 15.0, floor);
        self.hut(Material::Wood, 15.0, floor);
        for x in [11.5, 18.5] {
            self.block(Material::Stone, vec2(x, 0.5), 1.0, 1.0);
            self.target(x, 1.0);
        }
        self.level.shots = 5;
    }
}
