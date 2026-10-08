use std::f32::consts::TAU;
use std::fmt;

use super::MassProperties;
use crate::math::{Aabb, Transform, Vec2, vec2};

/// Maximum number of vertices of a [`Polygon`].
///
/// Small polygons keep collision detection fast; larger shapes should be built from several
/// colliders.
pub const MAX_POLYGON_VERTICES: usize = 8;

/// Input points closer than this distance are merged into a single vertex.
const WELD_DISTANCE: f32 = 0.0025;

/// A solid convex polygon with counter-clockwise vertices.
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    vertices: Vec<Vec2>,
    normals: Vec<Vec2>,
    centroid: Vec2,
}

impl Polygon {
    /// Creates the convex hull of the given points.
    ///
    /// Points do not need to be ordered. Duplicate points and points lying inside the hull are
    /// discarded.
    ///
    /// # Errors
    ///
    /// Returns an error if the points are not finite, if there are fewer than three distinct
    /// points, if they are all aligned, or if the hull has more than [`MAX_POLYGON_VERTICES`]
    /// vertices.
    pub fn new(points: &[Vec2]) -> Result<Self, PolygonError> {
        if !points.iter().all(|p| p.is_finite()) {
            return Err(PolygonError::NonFinitePoint);
        }
        let hull = convex_hull(&weld_points(points));
        if hull.len() < 3 {
            return Err(PolygonError::Degenerate);
        }
        if hull.len() > MAX_POLYGON_VERTICES {
            return Err(PolygonError::TooManyVertices(hull.len()));
        }
        Ok(Self::from_hull(hull))
    }

    /// Creates a rectangle centered on the body origin.
    ///
    /// # Panics
    ///
    /// Panics if a half extent is not strictly positive and finite.
    #[must_use]
    pub fn rectangle(half_width: f32, half_height: f32) -> Self {
        assert!(
            half_width.is_finite() && half_width > 0.0 && half_height.is_finite() && half_height > 0.0,
            "rectangle half extents must be positive, got {half_width} x {half_height}"
        );
        Self::from_hull(vec![
            vec2(-half_width, -half_height),
            vec2(half_width, -half_height),
            vec2(half_width, half_height),
            vec2(-half_width, half_height),
        ])
    }

    /// Creates a regular polygon centered on the body origin, with a vertex on the +X axis.
    ///
    /// # Panics
    ///
    /// Panics if `sides` is not between 3 and [`MAX_POLYGON_VERTICES`], or if `radius` (the
    /// distance from the center to each vertex) is not strictly positive and finite.
    #[must_use]
    pub fn regular(sides: usize, radius: f32) -> Self {
        assert!(
            (3..=MAX_POLYGON_VERTICES).contains(&sides),
            "a regular polygon needs 3 to {MAX_POLYGON_VERTICES} sides, got {sides}"
        );
        assert!(radius.is_finite() && radius > 0.0, "regular polygon radius must be positive, got {radius}");
        let vertices = (0..sides).map(|i| radius * Vec2::from_angle(TAU * i as f32 / sides as f32)).collect();
        Self::from_hull(vertices)
    }

    /// Vertices in counter-clockwise order, in the local frame.
    #[must_use]
    pub fn vertices(&self) -> &[Vec2] {
        &self.vertices
    }

    /// Outward unit normals; `normals()[i]` belongs to the edge from vertex `i` to vertex `i + 1`.
    #[must_use]
    pub fn normals(&self) -> &[Vec2] {
        &self.normals
    }

    /// Centroid (center of area), in the local frame.
    #[must_use]
    pub fn centroid(&self) -> Vec2 {
        self.centroid
    }

    /// Builds a polygon from vertices already known to form a counter-clockwise convex hull.
    fn from_hull(vertices: Vec<Vec2>) -> Self {
        let normals = edges(&vertices)
            .map(|(a, b)| {
                let edge = b - a;
                vec2(edge.y, -edge.x).normalize()
            })
            .collect();
        let mut polygon = Self { vertices, normals, centroid: Vec2::ZERO };
        polygon.centroid = polygon.mass_properties(1.0).center;
        polygon
    }

    pub(crate) fn mass_properties(&self, density: f32) -> MassProperties {
        // Split the polygon into a fan of triangles sharing the first vertex. Working relative to
        // that vertex keeps the numbers small and the result accurate far from the origin.
        let origin = self.vertices[0];
        let mut area = 0.0;
        let mut weighted_center = Vec2::ZERO;
        let mut origin_inertia = 0.0;

        for window in self.vertices[1..].windows(2) {
            let e1 = window[0] - origin;
            let e2 = window[1] - origin;
            let cross = e1.perp_dot(e2);
            let triangle_area = 0.5 * cross;
            area += triangle_area;
            weighted_center += triangle_area * (e1 + e2) / 3.0;

            let integral_x2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
            let integral_y2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
            origin_inertia += cross / 12.0 * (integral_x2 + integral_y2);
        }

        let mass = density * area;
        let center = weighted_center / area;
        // Parallel axis theorem: move the inertia from the first vertex to the center of mass.
        let inertia = density * origin_inertia - mass * center.length_squared();
        MassProperties { mass, center: origin + center, inertia }
    }

    pub(crate) fn compute_aabb(&self, transform: &Transform) -> Aabb {
        Aabb::from_points(self.vertices.iter().map(|&v| transform.transform_point(v)))
            .expect("a polygon has at least three vertices")
    }
}

/// Reason why a [`Polygon`] could not be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolygonError {
    /// A point has a NaN or infinite coordinate.
    NonFinitePoint,
    /// Fewer than three distinct points, or all points are aligned.
    Degenerate,
    /// The convex hull has more than [`MAX_POLYGON_VERTICES`] vertices.
    TooManyVertices(usize),
}

impl fmt::Display for PolygonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinitePoint => write!(f, "polygon points must be finite"),
            Self::Degenerate => write!(f, "polygon needs at least three points that are not aligned"),
            Self::TooManyVertices(count) => {
                write!(f, "polygon hull has {count} vertices, the maximum is {MAX_POLYGON_VERTICES}")
            }
        }
    }
}

impl std::error::Error for PolygonError {}

/// Iterates over the edges of a closed polygon as `(start, end)` pairs.
fn edges(vertices: &[Vec2]) -> impl Iterator<Item = (Vec2, Vec2)> + '_ {
    vertices.iter().copied().zip(vertices.iter().copied().cycle().skip(1))
}

/// Removes points that are too close to a point seen earlier.
fn weld_points(points: &[Vec2]) -> Vec<Vec2> {
    let mut welded: Vec<Vec2> = Vec::with_capacity(points.len());
    for &point in points {
        if welded.iter().all(|kept| kept.distance_squared(point) > WELD_DISTANCE * WELD_DISTANCE) {
            welded.push(point);
        }
    }
    welded
}

/// Andrew's monotone chain: returns the convex hull in counter-clockwise order, without
/// collinear vertices.
fn convex_hull(points: &[Vec2]) -> Vec<Vec2> {
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));

    // `true` when going a -> b -> c does not turn left.
    let is_not_left_turn = |a: Vec2, b: Vec2, c: Vec2| (b - a).perp_dot(c - a) <= 0.0;

    let mut hull: Vec<Vec2> = Vec::with_capacity(sorted.len() + 1);
    // Lower hull, left to right.
    for &point in &sorted {
        while hull.len() >= 2 && is_not_left_turn(hull[hull.len() - 2], hull[hull.len() - 1], point) {
            hull.pop();
        }
        hull.push(point);
    }
    // Upper hull, right to left; it must not pop into the lower hull.
    let lower_len = hull.len() + 1;
    for &point in sorted.iter().rev().skip(1) {
        while hull.len() >= lower_len && is_not_left_turn(hull[hull.len() - 2], hull[hull.len() - 1], point) {
            hull.pop();
        }
        hull.push(point);
    }
    // The last point is the first one again.
    hull.pop();
    hull
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hull_discards_interior_and_collinear_points() {
        let points = [
            vec2(0.0, 0.0),
            vec2(1.0, 1.0), // interior
            vec2(2.0, 0.0),
            vec2(1.0, 0.0), // collinear with the bottom edge
            vec2(2.0, 2.0),
            vec2(0.0, 2.0),
        ];
        let polygon = Polygon::new(&points).unwrap();
        assert_eq!(polygon.vertices().len(), 4);
    }

    #[test]
    fn vertices_are_counter_clockwise() {
        let polygon = Polygon::new(&[vec2(0.0, 0.0), vec2(0.0, 1.0), vec2(1.0, 0.0)]).unwrap();
        let v = polygon.vertices();
        assert!((v[1] - v[0]).perp_dot(v[2] - v[0]) > 0.0);
    }

    #[test]
    fn normals_point_outward() {
        let polygon = Polygon::rectangle(1.0, 1.0);
        for (vertex, normal) in polygon.vertices().iter().zip(polygon.normals()) {
            assert!((*vertex - polygon.centroid()).dot(*normal) > 0.0);
        }
    }

    #[test]
    fn regular_polygon_is_centered() {
        let hexagon = Polygon::regular(6, 2.0);
        assert_eq!(hexagon.vertices().len(), 6);
        assert!(hexagon.centroid().abs_diff_eq(Vec2::ZERO, 1e-5));
        assert!(hexagon.vertices().iter().all(|v| (v.length() - 2.0).abs() < 1e-5));
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        let aligned = [vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(2.0, 2.0)];
        assert_eq!(Polygon::new(&aligned), Err(PolygonError::Degenerate));
        assert_eq!(Polygon::new(&[Vec2::ZERO, Vec2::X]), Err(PolygonError::Degenerate));
        assert_eq!(Polygon::new(&[Vec2::ZERO, Vec2::X, Vec2::NAN]), Err(PolygonError::NonFinitePoint));

        let circle: Vec<_> = (0..12).map(|i| Vec2::from_angle(i as f32 * 0.5)).collect();
        assert_eq!(Polygon::new(&circle), Err(PolygonError::TooManyVertices(12)));
    }

    #[test]
    fn rectangle_mass_properties_match_closed_form() {
        let (width, height, density) = (4.0, 2.0, 3.0);
        let offset = vec2(5.0, -7.0);
        let corners = [vec2(0.0, 0.0), vec2(width, 0.0), vec2(width, height), vec2(0.0, height)];
        let polygon = Polygon::new(&corners.map(|c| c + offset)).unwrap();

        let props = polygon.mass_properties(density);
        let mass = density * width * height;
        assert!((props.mass - mass).abs() < 1e-4);
        assert!(props.center.abs_diff_eq(offset + vec2(width, height) / 2.0, 1e-4));
        let inertia = mass * (width * width + height * height) / 12.0;
        assert!((props.inertia - inertia).abs() < 1e-3, "{} != {inertia}", props.inertia);
    }
}
