use std::ops::{Add, Div, Sub};

pub fn f_inf_sqrt(x: f32) -> f32 {
    const THREEHALVES: f32 = 1.5;
    let x2 = x * 0.5;
    let mut i = x.to_bits();
    i = 0x5f3759df - (i >> 1);
    let mut y = f32::from_bits(i);
    y = y * (THREEHALVES - (x2 * y * y));
    y = y * (THREEHALVES - (x2 * y * y));
    y
}

#[derive(Clone, Copy, Debug)]
pub struct Vec2 {
    x: f32,
    y: f32,
}
impl Vec2 {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
    pub fn dot(&self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }
    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    pub fn normalize(&self) -> Self {
        let len = self.length();
        Self {
            x: self.x / len,
            y: self.y / len,
        }
    }
    pub fn normalize_f(&self) -> Self {
        let x = self.x * self.x + self.y * self.y;
        let len = f_inf_sqrt(x);
        Self {
            x: self.x / len,
            y: self.y / len,
        }
    }
}
impl Sub<Vec2> for Vec2 {
    type Output = Vec2;

    fn sub(self, rhs: Vec2) -> Self::Output {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}
impl Add<Vec2> for Vec2 {
    type Output = Vec2;

    fn add(self, rhs: Vec2) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}
impl Div<f32> for Vec2 {
    type Output = Vec2;

    fn div(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

fn reorder_vertices(mut r: [Vec2; 4]) -> [Vec2; 4] {
    let v01 = r[1] - r[0];
    let v02 = r[2] - r[0];
    let v03 = r[3] - r[0];
    if v01.dot(v03).abs() > 1e-8 {
        if v01.dot(v02).abs() > 1e-8 {
            r.swap(1, 2);
        } else {
            r.swap(3, 2);
        }
    }
    r
}

fn project(ax: Vec2, point: Vec2) -> f32 {
    let ax = ax.normalize();
    point.dot(ax)
}

fn project_f(ax: Vec2, point: Vec2) -> f32 {
    let ax = ax.normalize_f();
    point.dot(ax)
}

/**
 * 0--------1
 * |        |
 * |        |
 * |        |
 * 3--------2
 */
pub fn sat2d(b1: [Vec2; 4], b2: [Vec2; 4]) -> bool {
    let center1 = (b1[0] + b1[1] + b1[2] + b1[3]) / 4.0;
    let center2 = (b2[0] + b2[1] + b2[2] + b2[3]) / 4.0;
    let r1 = (b1[0] - center1).length();
    let r2 = (b2[0] - center2).length();
    let dist = (center1 - center2).length();
    if dist > r1 + r2 {
        // If the bounding boxes are far away from each other, they can't collide
        return false;
    }

    let b1 = reorder_vertices(b1);
    let b2 = reorder_vertices(b2);

    let _w = (b1[1] - b1[0]).dot(b1[3] - b1[0]).abs() > 1e-8;
    assert!(
        (b1[1] - b1[0]).dot(b1[3] - b1[0]).abs() <= 1e-8,
        "Edges of bounding box 1 are not orthogonal"
    );
    assert!(
        (b2[1] - b2[0]).dot(b2[3] - b2[0]).abs() <= 1e-8,
        "Edges of bounding box 2 are not orthogonal"
    );

    let axes = [b1[1] - b1[0], b1[3] - b1[0], b2[1] - b2[0], b2[3] - b2[0]];

    for ax in axes {
        let ax_b1p0 = project(ax, b1[0]);
        let ax_b1p1 = project(ax, b1[1]);
        let ax_b1p2 = project(ax, b1[2]);
        let ax_b1p3 = project(ax, b1[3]);
        let ax_b1_min = ax_b1p0.min(ax_b1p1).min(ax_b1p2).min(ax_b1p3);
        let ax_b1_max = ax_b1p0.max(ax_b1p1).max(ax_b1p2).max(ax_b1p3);

        let ax_b2p0 = project(ax, b2[0]);
        let ax_b2p1 = project(ax, b2[1]);
        let ax_b2p2 = project(ax, b2[2]);
        let ax_b2p3 = project(ax, b2[3]);
        let ax_b2_min = ax_b2p0.min(ax_b2p1).min(ax_b2p2).min(ax_b2p3);
        let ax_b2_max = ax_b2p0.max(ax_b2p1).max(ax_b2p2).max(ax_b2p3);

        if ax_b1_min.max(ax_b2_min) >= ax_b1_max.min(ax_b2_max) {
            return false;
        }
    }
    true
}

pub fn sat2d_f(b1: [Vec2; 4], b2: [Vec2; 4]) -> bool {
    let center1 = (b1[0] + b1[1] + b1[2] + b1[3]) / 4.0;
    let center2 = (b2[0] + b2[1] + b2[2] + b2[3]) / 4.0;
    let r1 = (b1[0] - center1).length();
    let r2 = (b2[0] - center2).length();
    let dist = (center1 - center2).length();
    if dist > r1 + r2 {
        // If the bounding boxes are far away from each other, they can't collide
        return false;
    }

    let b1 = reorder_vertices(b1);
    let b2 = reorder_vertices(b2);

    let _w = (b1[1] - b1[0]).dot(b1[3] - b1[0]).abs() > 1e-8;
    assert!(
        (b1[1] - b1[0]).dot(b1[3] - b1[0]).abs() <= 1e-8,
        "Edges of bounding box 1 are not orthogonal"
    );
    assert!(
        (b2[1] - b2[0]).dot(b2[3] - b2[0]).abs() <= 1e-8,
        "Edges of bounding box 2 are not orthogonal"
    );

    let axes = [b1[1] - b1[0], b1[3] - b1[0], b2[1] - b2[0], b2[3] - b2[0]];

    for ax in axes {
        let ax_b1p0 = project_f(ax, b1[0]);
        let ax_b1p1 = project_f(ax, b1[1]);
        let ax_b1p2 = project_f(ax, b1[2]);
        let ax_b1p3 = project_f(ax, b1[3]);
        let ax_b1_min = ax_b1p0.min(ax_b1p1).min(ax_b1p2).min(ax_b1p3);
        let ax_b1_max = ax_b1p0.max(ax_b1p1).max(ax_b1p2).max(ax_b1p3);

        let ax_b2p0 = project_f(ax, b2[0]);
        let ax_b2p1 = project_f(ax, b2[1]);
        let ax_b2p2 = project_f(ax, b2[2]);
        let ax_b2p3 = project_f(ax, b2[3]);
        let ax_b2_min = ax_b2p0.min(ax_b2p1).min(ax_b2p2).min(ax_b2p3);
        let ax_b2_max = ax_b2p0.max(ax_b2p1).max(ax_b2p2).max(ax_b2p3);

        if ax_b1_min.max(ax_b2_min) >= ax_b1_max.min(ax_b2_max) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod sat_tests {
    use super::Vec2;

    fn permute_test(b1: [Vec2; 4], b2: [Vec2; 4], intersect: bool) {
        let sb1 = [
            [b1[0], b1[1], b1[2], b1[3]],
            [b1[0], b1[2], b1[1], b1[3]],
            [b1[0], b1[1], b1[3], b1[2]],
            [b1[0], b1[3], b1[1], b1[2]],
            [b1[0], b1[3], b1[2], b1[1]],
            [b1[1], b1[2], b1[3], b1[0]],
            [b1[1], b1[3], b1[2], b1[0]],
            [b1[1], b1[2], b1[0], b1[3]],
            [b1[1], b1[0], b1[2], b1[3]],
            [b1[1], b1[0], b1[3], b1[2]],
            [b1[2], b1[3], b1[0], b1[1]],
            [b1[2], b1[0], b1[3], b1[1]],
            [b1[2], b1[3], b1[1], b1[0]],
            [b1[2], b1[1], b1[3], b1[0]],
            [b1[2], b1[1], b1[0], b1[3]],
            [b1[3], b1[0], b1[1], b1[2]],
            [b1[3], b1[1], b1[0], b1[2]],
            [b1[3], b1[0], b1[2], b1[1]],
            [b1[3], b1[2], b1[0], b1[1]],
            [b1[3], b1[2], b1[1], b1[0]],
        ];
        let sb2 = [
            [b2[0], b2[1], b2[2], b2[3]],
            [b2[0], b2[2], b2[1], b2[3]],
            [b2[0], b2[1], b2[3], b2[2]],
            [b2[0], b2[3], b2[1], b2[2]],
            [b2[0], b2[3], b2[2], b2[1]],
            [b2[0], b2[2], b2[3], b2[1]],
            [b2[1], b2[2], b2[3], b2[0]],
            [b2[1], b2[3], b2[2], b2[0]],
            [b2[1], b2[2], b2[0], b2[3]],
            [b2[1], b2[0], b2[2], b2[3]],
            [b2[1], b2[0], b2[3], b2[2]],
            [b2[1], b2[3], b2[0], b2[2]],
            [b2[2], b2[3], b2[0], b2[1]],
            [b2[2], b2[0], b2[3], b2[1]],
            [b2[2], b2[3], b2[1], b2[0]],
            [b2[2], b2[1], b2[3], b2[0]],
            [b2[2], b2[1], b2[0], b2[3]],
            [b2[2], b2[0], b2[1], b2[3]],
            [b2[3], b2[0], b2[1], b2[2]],
            [b2[3], b2[1], b2[0], b2[2]],
            [b2[3], b2[0], b2[2], b2[1]],
            [b2[3], b2[2], b2[0], b2[1]],
            [b2[3], b2[2], b2[1], b2[0]],
            [b2[3], b2[1], b2[0], b2[0]],
        ];

        for (b1, b2) in sb1.into_iter().zip(sb2.into_iter()) {
            assert!(
                super::sat2d(b1, b2) == intersect,
                "b1: {:?}, b2: {:?}",
                b1,
                b2
            );
        }
    }

    #[test]
    fn test_sat1() {
        let b1 = [
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        let b2 = [
            Vec2::new(1.5, 0.5),
            Vec2::new(2.5, 0.5),
            Vec2::new(2.5, 1.5),
            Vec2::new(1.5, 1.5),
        ];
        permute_test(b1, b2, false);
    }

    #[test]
    fn test_sat2() {
        let b1 = [
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        let b2 = [
            Vec2::new(0.5, 0.5),
            Vec2::new(1.5, 0.5),
            Vec2::new(1.5, 1.5),
            Vec2::new(0.5, 1.5),
        ];
        permute_test(b1, b2, true);
    }

    #[test]
    fn test_sat3() {
        let b1 = [
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        let b2 = [
            Vec2::new(0.5, 0.5),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.5, 0.5),
            Vec2::new(1.0, 0.0),
        ];
        permute_test(b1, b2, true);
    }

    #[test]
    fn test_sat4() {
        let b1 = [
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let b2 = [
            Vec2::new(0.5, 0.5),
            Vec2::new(1.5, 0.5),
            Vec2::new(1.5, 1.5),
            Vec2::new(0.5, 1.5),
        ];
        permute_test(b1, b2, true);
    }

    #[test]
    fn test_sat5() {
        let b1 = [
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 4.0),
            Vec2::new(0.0, 4.0),
        ];
        let b2 = [
            Vec2::new(1.5, 0.5),
            Vec2::new(2.5, 0.5),
            Vec2::new(2.5, 4.5),
            Vec2::new(1.5, 4.5),
        ];
        permute_test(b1, b2, false);
    }
}
