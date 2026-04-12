use crate::core::material::Material;
use crate::core::shape::Shape;
use crate::math::Vec2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyType {
    Static,
    Dynamic,
#[doc(hidden)]
    Kinematic, // pour plus tard
}

#[derive(Clone, Debug)]
pub struct RigidBody {
    // État dynamique
    pub state: BodyState,
    // Propriétés physiques
    pub props: BodyProps,
    // Forme
    pub shape: Shape,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyState {
    // Position
    pub position: Vec2,
    pub velocity: Vec2,
    pub force: Vec2,

    // Rotation
    pub angle: f32,
    pub angular_velocity: f32,
    pub torque: f32,

    // Activité
    pub is_awake: bool,
    pub sleep_time: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyProps {
    pub body_type: BodyType,
    pub mass: f32,
    pub inv_mass: f32,
    pub inertia: f32,
    pub inv_inertia: f32,
    pub material: Material,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
}

impl RigidBody {
    pub fn new(position: Vec2, mass: f32, shape: Shape) -> Self {
        let (inv_mass, body_type) = Self::compute_mass(mass);
        let (inertia, inv_inertia) = Self::compute_inertia(mass, &shape);

        Self {
            state: BodyState {
                position,
                velocity: Vec2::ZERO,
                force: Vec2::ZERO,
                angle: 0.0,
                angular_velocity: 0.0,
                torque: 0.0,
                is_awake: true,
                sleep_time: 0.0,
            },
            props: BodyProps {
                body_type,
                mass,
                inv_mass,
                inertia,
                inv_inertia,
                material: Material::default(),
                linear_damping: 0.01,
                angular_damping: 0.01,
                gravity_scale: 1.0,
            },
            shape,
        }
    }

    pub fn static_body(position: Vec2, shape: Shape) -> Self {
        Self {
            state: BodyState {
                position,
                velocity: Vec2::ZERO,
                force: Vec2::ZERO,
                angle: 0.0,
                angular_velocity: 0.0,
                torque: 0.0,
                is_awake: false,
                sleep_time: 0.0,
            },
            props: BodyProps {
                body_type: BodyType::Static,
                mass: 0.0,
                inv_mass: 0.0,
                inertia: 0.0,
                inv_inertia: 0.0,
                material: Material::default(),
                linear_damping: 0.0,
                angular_damping: 0.0,
                gravity_scale: 0.0,
            },
            shape,
        }
    }

    // Builder
    pub fn with_material(mut self, material: Material) -> Self {
        self.props.material = material;
        self
    }

    pub fn with_damping(mut self, linear: f32, angular: f32) -> Self {
        self.props.linear_damping = linear;
        self.props.angular_damping = angular;
        self
    }

    pub fn with_gravity_scale(mut self, scale: f32) -> Self {
        self.props.gravity_scale = scale;
        self
    }

    // Méthodes utiles
    pub fn is_static(&self) -> bool {
        self.props.body_type == BodyType::Static
    }

    pub fn is_dynamic(&self) -> bool {
        self.props.body_type == BodyType::Dynamic
    }

    pub fn wake_up(&mut self) {
        self.state.is_awake = true;
        self.state.sleep_time = 0.0;
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        self.state.velocity += impulse * self.props.inv_mass;
    }

    pub fn apply_angular_impulse(&mut self, impulse: f32) {
        self.state.angular_velocity += impulse * self.props.inv_inertia;
    }

    // Internes
    fn compute_mass(mass: f32) -> (f32, BodyType) {
        let inv_mass = if mass == 0.0 { 0.0 } else { 1.0 / mass };
        let body_type = if mass == 0.0 { BodyType::Static } else { BodyType::Dynamic };
        (inv_mass, body_type)
    }

    fn compute_inertia(mass: f32, shape: &Shape) -> (f32, f32) {
        let inertia = match shape {
            Shape::Circle(radius) => 0.5 * mass * radius * radius,
            Shape::Rectangle(size) => mass * (size.x * size.x + size.y * size.y) / 12.0,
        };
        let inv_inertia = if inertia == 0.0 { 0.0 } else { 1.0 / inertia };
        (inertia, inv_inertia)
    }
}
