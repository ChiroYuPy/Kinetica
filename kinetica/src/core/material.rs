#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub friction: f32,
    pub restitution: f32,
}

impl Material {
    pub const fn new(friction: f32, restitution: f32) -> Self {
        Self { friction, restitution }
    }
}

impl Default for Material {
    fn default() -> Self {
        Self {
            friction: 0.3,
            restitution: 0.5,
        }
    }
}

impl Material {
    pub const WOOD: Self = Self::new(0.4, 0.2);
    pub const METAL: Self = Self::new(0.3, 0.5);
    pub const RUBBER: Self = Self::new(0.9, 0.8);
    pub const ICE: Self = Self::new(0.05, 0.1);
    pub const BOUNCE: Self = Self::new(0.3, 0.95);
}
