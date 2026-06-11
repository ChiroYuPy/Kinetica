use crate::math::{Vec2, AABB};

#[derive(Clone, Debug)]
pub struct RigidBody {
    pub transform: Transform,
    pub motion: Motion,
    pub mass: Mass,
    pub inertia: Inertia,
    pub material: Material,
    pub body_type: BodyType,
    pub state: BodyState,
    pub shape: Shape,
}

impl RigidBody {
    pub fn new(position: Vec2, mass: f32, shape: Shape) -> Self {
        let (inv_mass, body_type) = Self::compute_mass(mass);
        let (inertia, inv_inertia) = Self::compute_inertia(mass, &shape);

        Self {
            transform: Transform {
                position,
                rotation: 0.0,
            },

            motion: Motion {
                linear_velocity: Vec2::ZERO,
                angular_velocity: 0.0,
                accumulated_force: Vec2::ZERO,
                accumulated_torque: 0.0,
            },

            mass: Mass {
                mass,
                inverse_mass: inv_mass,
            },

            inertia: Inertia {
                inertia,
                inverse_inertia: inv_inertia,
            },

            material: Material {
                friction: 0.5,
                restitution: 0.5
            },

            body_type,
            state: BodyState {
                is_awake: true,
                sleep_time: 0.0,
            },

            shape,
        }
    }

    pub fn static_body(position: Vec2, shape: Shape) -> Self {
        Self {
            transform: Transform {
                position,
                rotation: 0.0,
            },

            motion: Motion {
                linear_velocity: Vec2::ZERO,
                angular_velocity: 0.0,
                accumulated_force: Vec2::ZERO,
                accumulated_torque: 0.0,
            },

            mass: Mass {
                mass: 0.0,
                inverse_mass: 0.0,
            },

            inertia: Inertia {
                inertia: 0.0,
                inverse_inertia: 0.0,
            },

            material: Material {
                friction: 0.5,
                restitution: 0.5
            },

            body_type: BodyType::Static,

            state: BodyState {
                is_awake: false,
                sleep_time: 0.0,
            },

            shape,
        }
    }

    pub fn with_material(mut self, material: Material) -> Self {
        self.material = material;
        self
    }

    pub fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    pub fn wake_up(&mut self) {
        self.state.is_awake = true;
        self.state.sleep_time = 0.0;
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        if self.mass.inverse_mass == 0.0 {
            return;
        }

        self.motion.linear_velocity += impulse * self.mass.inverse_mass;
    }

    pub fn apply_angular_impulse(&mut self, impulse: f32) {
        if self.inertia.inverse_inertia == 0.0 {
            return;
        }

        self.motion.angular_velocity += impulse * self.inertia.inverse_inertia;
    }

    fn compute_mass(mass: f32) -> (f32, BodyType) {
        if mass == 0.0 {
            (0.0, BodyType::Static)
        } else {
            (1.0 / mass, BodyType::Dynamic)
        }
    }

    fn compute_inertia(mass: f32, shape: &Shape) -> (f32, f32) {
        if mass == 0.0 {
            return (0.0, 0.0);
        }

        let inertia = match shape {
            Shape::Circle(radius) => 0.5 * mass * radius * radius,
            Shape::Rectangle(size) => mass * (size.x * size.x + size.y * size.y) / 12.0,
        };

        let inv = if inertia == 0.0 { 0.0 } else { 1.0 / inertia };
        (inertia, inv)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BodyState {
    pub is_awake: bool,
    pub sleep_time: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Circle(f32),
    Rectangle(Vec2),
}

impl Shape {
    pub fn compute_aabb(&self, position: Vec2) -> AABB {
        match self {
            Shape::Circle(radius) => AABB::new(position, Vec2::splat(*radius)),
            Shape::Rectangle(extents) => AABB::new(position, *extents / 2.0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub position: Vec2,
    pub rotation: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,

    pub accumulated_force: Vec2,
    pub accumulated_torque: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mass {
    pub mass: f32,
    pub inverse_mass: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertia {
    pub inertia: f32,
    pub inverse_inertia: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub friction: f32,
    pub restitution: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyType {
    Static,
    Dynamic,
}