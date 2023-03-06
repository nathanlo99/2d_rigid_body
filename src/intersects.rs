use glam::*;

pub trait Support {
    fn support_point(&self, direction: DVec2) -> DVec2;
}

pub struct ConvexPolygon {
    pub vertices: Vec<DVec2>,
}

impl Support for ConvexPolygon {
    fn support_point(&self, direction: DVec2) -> DVec2 {
        let mut best_vertex = self.vertices[0];
        let mut best_dot_product = best_vertex.dot(direction);
        for vertex in &self.vertices[1..] {
            let current_dot_product = vertex.dot(direction);
            if current_dot_product > best_dot_product {
                best_dot_product = current_dot_product;
                best_vertex = *vertex;
            }
        }
        best_vertex
    }
}

fn segment_normal_towards_origin(p1: DVec2, p2: DVec2) -> DVec2 {
    // TODO: Make this cleaner once we have a naive solution working
    let p1 = DVec3::new(p1.x, p1.y, 0.0);
    let p2 = DVec3::new(p2.x, p2.y, 0.0);
    let ab = (p2 - p1).normalize_or_zero();
    let ao = (-p1).normalize_or_zero();
    ab.cross(ao).cross(ab).xy()
}

fn nearest_triangle(triangle: (DVec2, DVec2, DVec2)) -> Option<(DVec2, DVec2, DVec2)> {
    let (c, b, a) = triangle;
    let ac = dvec3(c.x - a.x, c.y - a.y, 0.0);
    let ab = dvec3(b.x - a.x, b.y - a.y, 0.0);
    let ao = -a;
    let ab_perp = ac.cross(ab).cross(ab).xy();
    if ab_perp.dot(ao) > 0.0 {
        return Some((b, a, ab_perp));
    }
    let ac_perp = ab.cross(ac).cross(ac).xy();
    if ac_perp.dot(ao) > 0.0 {
        return Some((c, a, ac_perp));
    }
    None
}

pub fn intersects<T1: Support, T2: Support>(
    poly1: &T1,
    poly2: &T2,
) -> Option<(DVec2, DVec2, DVec2)> {
    let d = DVec2::X;
    let p1 = poly1.support_point(d) - poly2.support_point(-d);
    let d = -p1;
    let p2 = poly1.support_point(d) - poly2.support_point(-d);
    if p2.dot(d) < 0.0 {
        return None;
    }
    let d = segment_normal_towards_origin(p1, p2);
    let p3 = poly1.support_point(d) - poly2.support_point(-d);

    let mut triangle = (p1, p2, p3);

    while let Some((c, b, d)) = nearest_triangle(triangle) {
        let new_point = poly1.support_point(d) - poly2.support_point(-d);
        if new_point.dot(d) < 0.0 {
            return None;
        }
        triangle = (c, b, new_point);
    }
    Some(triangle)
}

pub fn intersection_direction<T1: Support, T2: Support>(poly1: &T1, poly2: &T2) -> Option<DVec2> {
    let (p1, p2, p3) = intersects(poly1, poly2)?;
    // The triangle (a, b, c) contains the origin

    let mut points: Vec<DVec2> = [p1, p2, p3].to_vec();
    loop {
        let num_points = points.len();

        // 1. Find the edge closest to the origin
        let mut closest_idx = 0;
        let mut closest_distance = f64::INFINITY;
        let mut closest_normal = DVec2::ZERO;
        for i in 0..num_points {
            let j = (i + 1) % num_points;
            let normal = -segment_normal_towards_origin(points[i], points[j]).normalize_or_zero();
            let distance = normal.dot(points[i]);
            if distance < closest_distance {
                closest_idx = j;
                closest_distance = distance;
                closest_normal = normal;
            }
        }

        // 2. Try and expand it
        let new_point = poly1.support_point(closest_normal) - poly2.support_point(-closest_normal);
        let new_distance = new_point.dot(closest_normal);
        if new_distance - closest_distance < 0.0001 {
            return Some(closest_normal * new_distance);
        }

        points.insert(closest_idx % num_points, new_point);
    }
}

// Under the assumption that the polygons are colliding with vertex-edge, find the contact point
pub fn contact_point(poly1: ConvexPolygon, poly2: ConvexPolygon) -> DVec2 {
    let mut closest_point: DVec2 = poly2.vertices[0];
    let mut closest_distance = f64::INFINITY;

    let vertices1 = poly1.vertices;
    let vertices2 = poly2.vertices;

    let n = vertices1.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let normal = (vertices1[j] - vertices1[i])
            .rotate(dvec2(0.0, -1.0))
            .normalize_or_zero();

        for point in &vertices2 {
            let this_distance = (*point - vertices1[i]).dot(normal).abs();
            if this_distance < closest_distance {
                closest_point = *point;
                closest_distance = this_distance;
            }
        }
    }

    let n = vertices2.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let normal = (vertices2[j] - vertices2[i])
            .rotate(dvec2(0.0, -1.0))
            .normalize_or_zero();

        for point in &vertices1 {
            let this_distance = (*point - vertices2[i]).dot(normal).abs();
            if this_distance < closest_distance {
                closest_point = *point;
                closest_distance = this_distance;
            }
        }
    }

    println!("Closest distance: {closest_distance}");

    closest_point
}
