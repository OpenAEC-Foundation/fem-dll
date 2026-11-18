//! A mesh of cells, as finite element meshes and simulation results hold them, and the outer
//! faces it is drawn by.
//!
//! The cells keep their corner nodes only: a quadratic cell is drawn as the linear one of its
//! corners. The nodes of a volume cell are in the usual order of the corner nodes: a tetrahedron
//! 0 1 2 3; a pyramid with the base 0 1 2 3 and the top 4; a wedge with the triangles 0 1 2 and
//! 3 4 5, 3 over 0; a hexahedron with the bottom 0 1 2 3 and the top 4 5 6 7, 4 over 0. Which
//! way a file turns its faces does not matter: [`Mesh::surfaces`] turns every outer face outward.

/// The kinds of cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CellKind {
    /// A point.
    Vertex,
    /// A line through two or more nodes.
    Line,
    Triangle,
    Quad,
    /// A face of more than four corners, in order around it.
    Polygon,
    Tetra,
    Pyramid,
    Wedge,
    Hexa,
}

impl CellKind {
    /// 0 for a point, 1 for a line, 2 for a face, 3 for a volume.
    pub fn dimension(self) -> u8 {
        match self {
            CellKind::Vertex => 0,
            CellKind::Line => 1,
            CellKind::Triangle | CellKind::Quad | CellKind::Polygon => 2,
            _ => 3,
        }
    }

    /// The number of corner nodes, `None` where any number goes (a line, a polygon).
    pub fn corners(self) -> Option<usize> {
        Some(match self {
            CellKind::Vertex => 1,
            CellKind::Triangle => 3,
            CellKind::Quad | CellKind::Tetra => 4,
            CellKind::Pyramid => 5,
            CellKind::Wedge => 6,
            CellKind::Hexa => 8,
            CellKind::Line | CellKind::Polygon => return None,
        })
    }

    /// The faces of a volume cell, each as its nodes' places in the cell, in order around it.
    pub fn faces(self) -> &'static [&'static [usize]] {
        match self {
            CellKind::Tetra => &[&[0, 1, 2], &[0, 1, 3], &[1, 2, 3], &[0, 2, 3]],
            CellKind::Pyramid => &[&[0, 1, 2, 3], &[0, 1, 4], &[1, 2, 4], &[2, 3, 4], &[3, 0, 4]],
            CellKind::Wedge => &[&[0, 1, 2], &[3, 4, 5], &[0, 1, 4, 3], &[1, 2, 5, 4], &[2, 0, 3, 5]],
            CellKind::Hexa => &[&[0, 1, 2, 3], &[4, 5, 6, 7], &[0, 1, 5, 4], &[1, 2, 6, 5], &[2, 3, 7, 6], &[3, 0, 4, 7]],
            _ => &[],
        }
    }
}

/// The cells of a mesh, stored flat: cell `k` has the nodes `nodes[start[k]..start[k + 1]]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Cells {
    pub kind: Vec<CellKind>,
    pub start: Vec<u32>,
    /// Indices into [`Mesh::points`].
    pub nodes: Vec<u32>,
    /// Index into [`Mesh::groups`].
    pub group: Vec<u32>,
}

impl Default for Cells {
    fn default() -> Self {
        Self { kind: Vec::new(), start: vec![0], nodes: Vec::new(), group: Vec::new() }
    }
}

impl Cells {
    pub fn len(&self) -> usize {
        self.kind.len()
    }

    pub fn is_empty(&self) -> bool {
        self.kind.is_empty()
    }

    /// Adds a cell of the corner nodes `nodes` in group `group`.
    pub fn push(&mut self, kind: CellKind, nodes: &[u32], group: u32) {
        self.kind.push(kind);
        self.nodes.extend_from_slice(nodes);
        self.start.push(self.nodes.len() as u32);
        self.group.push(group);
    }

    /// The nodes of cell `k`.
    pub fn nodes(&self, k: usize) -> &[u32] {
        &self.nodes[self.start[k] as usize..self.start[k + 1] as usize]
    }
}

/// Where the values of a field are given.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldAt {
    /// One value (of `components` numbers) per point.
    Point,
    /// One per cell.
    Cell,
}

/// Values the file gives at the points or cells: a temperature, a displacement, a stress.
#[derive(Clone, Debug, PartialEq)]
pub struct Field {
    pub name: String,
    pub at: FieldAt,
    /// 1 for a scalar, 3 for a vector, 6 or 9 for a tensor.
    pub components: usize,
    /// `components` numbers per point or cell.
    pub values: Vec<f32>,
}

impl Field {
    /// The value of point or cell `k`: the number itself, or for several components its length.
    pub fn magnitude(&self, k: usize) -> f32 {
        let c = self.components.max(1);
        let v = self.values.get(k * c..k * c + c).unwrap_or(&[]);
        match v {
            [one] => *one,
            _ => v.iter().map(|x| x * x).sum::<f32>().sqrt(),
        }
    }

    /// Component `c` of point or cell `k`.
    pub fn component(&self, k: usize, c: usize) -> f32 {
        self.values.get(k * self.components.max(1) + c).copied().unwrap_or(f32::NAN)
    }

    /// How many points or cells the field gives values for.
    pub fn count(&self) -> usize {
        self.values.len() / self.components.max(1)
    }
}

/// The fields of one moment: of a time series one step, of a single file the only one.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Step {
    /// The time the file gives the step, if any.
    pub time: Option<f64>,
    pub fields: Vec<Field>,
}

/// A mesh with its fields.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    pub name: String,
    /// In the file's own units and axes.
    pub points: Vec<[f64; 3]>,
    pub cells: Cells,
    /// The names of the groups the cells are in (element sets, physical groups, pieces); at least
    /// one when there are cells.
    pub groups: Vec<String>,
    /// At least one step when the file gives fields.
    pub steps: Vec<Step>,
}

/// The outer faces of one group of a mesh, ready to draw: every face has vertices of its own,
/// so a vertex belongs to one cell.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Surface {
    /// Index into [`Mesh::groups`].
    pub group: u32,
    pub positions: Vec<[f64; 3]>,
    /// Of the face the vertex belongs to, unit length (zero for a vertex of a line or a point).
    pub normals: Vec<[f32; 3]>,
    /// The point and the cell of the mesh each vertex comes from, for colouring by a field.
    pub point: Vec<u32>,
    pub cell: Vec<u32>,
    /// Triangles, three vertex indices each.
    pub triangles: Vec<u32>,
    /// Line segments, two vertex indices each; their vertices are not used by `triangles`.
    pub lines: Vec<u32>,
    /// Vertices that are points of their own (point cells).
    pub dots: Vec<u32>,
}

impl Surface {
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty() && self.lines.is_empty() && self.dots.is_empty()
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The normal of a polygon (Newell's), not unit length.
fn newell(corners: &[[f64; 3]]) -> [f64; 3] {
    let mut n = [0.0; 3];
    for (k, a) in corners.iter().enumerate() {
        let b = corners[(k + 1) % corners.len()];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    n
}

fn centroid(points: impl Iterator<Item = [f64; 3]>) -> [f64; 3] {
    let (mut c, mut n) = ([0.0; 3], 0.0);
    for p in points {
        c = [c[0] + p[0], c[1] + p[1], c[2] + p[2]];
        n += 1.0;
    }
    if n > 0.0 { [c[0] / n, c[1] / n, c[2] / n] } else { c }
}

impl Mesh {
    /// Adds the points, cells, groups and fields of `other` after its own (the pieces of a
    /// parallel file, the blocks of a multiblock file). A field only one of the two has gets no
    /// value (NaN) at the points or cells of the other.
    pub fn append(&mut self, other: Mesh) {
        let (points, cells, groups) = (self.points.len(), self.cells.len(), self.groups.len() as u32);
        let (their_points, their_cells) = (other.points.len(), other.cells.len());
        let count = |at: FieldAt, p: usize, c: usize| if at == FieldAt::Point { p } else { c };
        let steps = self.steps.len().max(other.steps.len());
        for k in 0..steps {
            if self.steps.len() <= k {
                self.steps.push(Step { time: other.steps.get(k).and_then(|s| s.time), fields: Vec::new() });
            }
            let theirs: &[Field] = other.steps.get(k).map_or(&[], |s| &s.fields);
            let mine = &mut self.steps[k].fields;
            for f in mine.iter_mut() {
                match theirs.iter().find(|t| t.name == f.name && t.at == f.at && t.components == f.components) {
                    Some(t) => f.values.extend_from_slice(&t.values),
                    None => f.values.extend(std::iter::repeat(f32::NAN).take(count(f.at, their_points, their_cells) * f.components)),
                }
            }
            for t in theirs {
                if !mine.iter().any(|f| f.name == t.name && f.at == t.at && f.components == t.components) {
                    let mut values = vec![f32::NAN; count(t.at, points, cells) * t.components];
                    values.extend_from_slice(&t.values);
                    mine.push(Field { values, ..t.clone() });
                }
            }
        }
        self.points.extend_from_slice(&other.points);
        let mut nodes = Vec::new();
        for k in 0..their_cells {
            nodes.clear();
            nodes.extend(other.cells.nodes(k).iter().map(|n| n + points as u32));
            self.cells.push(other.cells.kind[k], &nodes, other.cells.group[k] + groups);
        }
        if other.groups.is_empty() && their_cells > 0 {
            self.groups.push(other.name);
        } else {
            self.groups.extend(other.groups);
        }
    }

    /// The smallest and largest point, `None` for a mesh without points.
    pub fn bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        let mut it = self.points.iter().filter(|p| p.iter().all(|v| v.is_finite()));
        let first = *it.next()?;
        Some(it.fold((first, first), |(lo, hi), p| ([lo[0].min(p[0]), lo[1].min(p[1]), lo[2].min(p[2])], [hi[0].max(p[0]), hi[1].max(p[1]), hi[2].max(p[2])])))
    }

    /// The cells' outer faces per group, in the order of the groups; groups with nothing to draw
    /// are left out. A face of a volume cell is outer when no other volume cell has it; faces,
    /// lines and points are drawn as they are.
    pub fn surfaces(&self) -> Vec<Surface> {
        let cells = &self.cells;
        let point = |i: u32| self.points.get(i as usize).copied().unwrap_or([f64::NAN; 3]);
        // every face of every volume cell, by its sorted nodes: the outer ones occur once
        let mut faces: Vec<([u32; 4], u32, u8)> = Vec::new();
        for k in 0..cells.len() {
            let nodes = cells.nodes(k);
            for (f, face) in cells.kind[k].faces().iter().enumerate() {
                if face.iter().any(|&i| i >= nodes.len()) {
                    continue;
                }
                let mut key = [u32::MAX; 4];
                for (slot, &i) in face.iter().enumerate() {
                    key[slot] = nodes[i];
                }
                key.sort_unstable();
                faces.push((key, k as u32, f as u8));
            }
        }
        faces.sort_unstable_by(|a, b| a.0.cmp(&b.0));
        let mut outer: Vec<(u32, u8)> = Vec::new();
        let mut i = 0;
        while i < faces.len() {
            let mut j = i + 1;
            while j < faces.len() && faces[j].0 == faces[i].0 {
                j += 1;
            }
            if j == i + 1 {
                outer.push((faces[i].1, faces[i].2));
            }
            i = j;
        }
        outer.sort_unstable();

        let groups = self.groups.len().max(1);
        let mut surfaces: Vec<Surface> = (0..groups).map(|g| Surface { group: g as u32, ..Surface::default() }).collect();
        let add_face = |surface: &mut Surface, cell: u32, corners: &[u32], outward: Option<[f64; 3]>| {
            let mut corners = corners.to_vec();
            let at: Vec<[f64; 3]> = corners.iter().map(|&i| point(i)).collect();
            let mut n = newell(&at);
            if let Some(inside) = outward {
                if dot(n, sub(centroid(at.iter().copied()), inside)) < 0.0 {
                    corners.reverse();
                    n = [-n[0], -n[1], -n[2]];
                }
            }
            let len = dot(n, n).sqrt();
            let normal = if len > 0.0 { [(n[0] / len) as f32, (n[1] / len) as f32, (n[2] / len) as f32] } else { [0.0; 3] };
            let first = surface.positions.len() as u32;
            for &c in &corners {
                surface.positions.push(point(c));
                surface.normals.push(normal);
                surface.point.push(c);
                surface.cell.push(cell);
            }
            for k in 1..corners.len().saturating_sub(1) as u32 {
                surface.triangles.extend_from_slice(&[first, first + k, first + k + 1]);
            }
        };
        let mut outer = outer.into_iter().peekable();
        for k in 0..cells.len() {
            let nodes = cells.nodes(k);
            let group = (cells.group[k] as usize).min(groups - 1);
            let surface = &mut surfaces[group];
            let kind = cells.kind[k];
            match kind.dimension() {
                0 => {
                    for &n in nodes {
                        surface.dots.push(surface.positions.len() as u32);
                        surface.positions.push(point(n));
                        surface.normals.push([0.0; 3]);
                        surface.point.push(n);
                        surface.cell.push(k as u32);
                    }
                }
                1 => {
                    let first = surface.positions.len() as u32;
                    for &n in nodes {
                        surface.positions.push(point(n));
                        surface.normals.push([0.0; 3]);
                        surface.point.push(n);
                        surface.cell.push(k as u32);
                    }
                    for s in 1..nodes.len() as u32 {
                        surface.lines.extend_from_slice(&[first + s - 1, first + s]);
                    }
                }
                2 if nodes.len() >= 3 => add_face(surface, k as u32, nodes, None),
                3 => {
                    let inside = centroid(nodes.iter().map(|&i| point(i)));
                    while let Some(&(cell, f)) = outer.peek() {
                        if cell as usize != k {
                            break;
                        }
                        outer.next();
                        let face: Vec<u32> = kind.faces()[f as usize].iter().map(|&i| nodes[i]).collect();
                        add_face(surface, k as u32, &face, Some(inside));
                    }
                }
                _ => {}
            }
        }
        surfaces.retain(|s| !s.is_empty());
        surfaces
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two hexahedra side by side, a unit cube each.
    fn two_cubes() -> Mesh {
        let mut points = Vec::new();
        for z in 0..2 {
            for y in 0..2 {
                for x in 0..3 {
                    points.push([x as f64, y as f64, z as f64]);
                }
            }
        }
        let at = |x: u32, y: u32, z: u32| z * 6 + y * 3 + x;
        let mut cells = Cells::default();
        for x in 0..2 {
            let n = [at(x, 0, 0), at(x + 1, 0, 0), at(x + 1, 1, 0), at(x, 1, 0), at(x, 0, 1), at(x + 1, 0, 1), at(x + 1, 1, 1), at(x, 1, 1)];
            cells.push(CellKind::Hexa, &n, 0);
        }
        Mesh { name: "cubes".into(), points, cells, groups: vec!["all".into()], steps: Vec::new() }
    }

    #[test]
    fn the_face_two_volumes_share_is_not_drawn() {
        let surfaces = two_cubes().surfaces();
        assert_eq!(surfaces.len(), 1);
        let s = &surfaces[0];
        // 12 outer quads of the 2 × 1 × 1 block, two triangles each
        assert_eq!(s.triangles.len() / 3, 20);
        assert_eq!(s.positions.len(), 40);
        assert!(s.cell.iter().all(|c| *c < 2));
    }

    #[test]
    fn outer_faces_face_outward_however_the_file_turns_them() {
        let mut mesh = two_cubes();
        // the second cube with its bottom and top swapped: its faces turn the other way
        let n: Vec<u32> = mesh.cells.nodes(1).to_vec();
        let swapped = [n[4], n[5], n[6], n[7], n[0], n[1], n[2], n[3]];
        let start = mesh.cells.start[1] as usize;
        mesh.cells.nodes[start..start + 8].copy_from_slice(&swapped);
        let s = &mesh.surfaces()[0];
        let middle = [1.0, 0.5, 0.5];
        for t in s.triangles.chunks_exact(3) {
            let p = s.positions[t[0] as usize];
            let n = s.normals[t[0] as usize];
            let out = [p[0] - middle[0], p[1] - middle[1], p[2] - middle[2]];
            assert!(out[0] * n[0] as f64 + out[1] * n[1] as f64 + out[2] * n[2] as f64 > 0.0, "{p:?} {n:?}");
        }
    }

    #[test]
    fn lines_points_and_faces_are_drawn_per_group() {
        let mut cells = Cells::default();
        cells.push(CellKind::Line, &[0, 1, 2], 0);
        cells.push(CellKind::Quad, &[0, 1, 2, 3], 1);
        cells.push(CellKind::Vertex, &[3], 1);
        let mesh = Mesh { name: "m".into(), points: vec![[0.0; 3], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]], cells, groups: vec!["beams".into(), "plates".into()], steps: Vec::new() };
        let s = mesh.surfaces();
        assert_eq!(s.len(), 2);
        assert_eq!((s[0].lines.len(), s[0].triangles.len()), (4, 0));
        assert_eq!((s[1].triangles.len(), s[1].dots.len()), (6, 1));
        assert_eq!(s[1].normals[0], [0.0, 0.0, 1.0]);
    }

    #[test]
    fn a_vector_field_has_its_length_as_value() {
        let f = Field { name: "U".into(), at: FieldAt::Point, components: 3, values: vec![3.0, 4.0, 0.0, 1.0, 0.0, 0.0] };
        assert_eq!((f.count(), f.magnitude(0), f.magnitude(1), f.component(0, 1)), (2, 5.0, 1.0, 4.0));
    }
}
