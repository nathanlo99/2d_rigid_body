use glam::*;

use super::intersects::{self, ConvexPolygon, *};

pub trait Shape {
    type Corners;
    fn center(&self, position: DVec2, angle: f64) -> DVec2;
    fn inv_mass(&self) -> f64;
    fn inv_moment_of_inertia(&self) -> f64;
    fn corners(&self, position: DVec2, angle: f64) -> Self::Corners;
}

#[derive(Copy, Clone)]
pub struct RectangleShape {
    pub width: f64,
    pub height: f64,
    pub density: f64,

    pub inv_mass: f64,
    pub inv_moment_of_inertia: f64,

    pub colour: [f32; 4],
}

impl RectangleShape {
    pub fn new(width: f64, height: f64, density: f64, colour: [f32; 4]) -> RectangleShape {
        let mass = width * height * density;
        let moment_of_inertia = mass * (width * width + height * height) / 12.0;
        RectangleShape {
            width,
            height,
            density,
            inv_mass: 1.0 / mass,
            inv_moment_of_inertia: 1.0 / moment_of_inertia,
            colour,
        }
    }
}

impl Shape for RectangleShape {
    type Corners = ConvexPolygon;
    fn center(&self, position: DVec2, _angle: f64) -> DVec2 {
        position
    }

    fn inv_mass(&self) -> f64 {
        self.inv_mass
    }

    fn inv_moment_of_inertia(&self) -> f64 {
        self.inv_moment_of_inertia
    }

    fn corners(&self, position: DVec2, angle: f64) -> Self::Corners {
        let corners = [
            dvec2(-self.width / 2.0, -self.height / 2.0),
            dvec2(self.width / 2.0, -self.height / 2.0),
            dvec2(self.width / 2.0, self.height / 2.0),
            dvec2(-self.width / 2.0, self.height / 2.0),
        ];

        let rotate_vector = dvec2(angle.cos(), angle.sin());
        let vertices = corners
            .map(|corner| position + corner.rotate(rotate_vector))
            .to_vec();
        ConvexPolygon { vertices }
    }
}

pub struct RigidBody {
    pub position: DVec2,
    pub velocity: DVec2,
    pub force: DVec2,

    pub angle: f64,
    pub angular_velocity: f64,
    pub torque: f64,

    pub shape: RectangleShape,
    pub fixed: bool,
}

impl RigidBody {
    pub fn new(shape: RectangleShape, position: DVec2, angle: f64, fixed: bool) -> RigidBody {
        RigidBody {
            position,
            velocity: DVec2::ZERO,
            force: DVec2::ZERO,
            angle,
            angular_velocity: 0.0,
            torque: 0.0,
            shape,
            fixed,
        }
    }

    pub fn center(&self) -> DVec2 {
        self.shape.center(self.position, self.angle)
    }

    pub fn corners(&self) -> ConvexPolygon {
        self.shape.corners(self.position, self.angle)
    }

    pub fn intersects(&self, other: &Self) -> bool {
        let my_corners = self.corners();
        let other_corners = other.corners();
        my_corners
            .bounding_box()
            .intersects(&other_corners.bounding_box())
            && intersects::intersects(&my_corners, &other_corners).is_some()
    }

    pub fn clear_forces(&mut self) {
        self.force = DVec2::ZERO;
        self.torque = 0.0;
    }
}
