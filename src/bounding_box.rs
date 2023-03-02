use glam::*;

pub struct BoundingBox {
    pub min: DVec2,
    pub max: DVec2,
}

impl BoundingBox {
    pub fn intersects(&self, other: &Self) -> bool {
        let intersection_min = self.min.max(other.min);
        let intersection_max = self.max.min(other.max);
        intersection_min.x <= intersection_max.x && intersection_min.y <= intersection_max.y
    }

    pub fn contains(&self, other: &Self) -> bool {
        self.min.cmple(other.min).all() && self.max.cmpge(other.max).all()
    }
}
