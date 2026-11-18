//! VTK files, through the `vtkio` crate: the legacy `.vtk` file and the XML files `.vtu`
//! (unstructured grid), `.vtp` (polygon data), `.vts` (structured grid), `.vtr` (rectilinear
//! grid) and `.vti` (image data), their parallel files (`.pvtu`, `.pvtp`, `.pvts`, `.pvtr`,
//! `.pvti`) whose pieces are files beside them, the multiblock file `.vtm` and the time series
//! `.pvd`.
//!
//! - Cells keep their corner nodes; a pixel and a voxel become a quad and a hexahedron, a strip
//!   its triangles, a poly-vertex its points. Polyhedra and higher-order cells are left out.
//! - A grid (structured, rectilinear, image data) of three dimensions keeps only its outer
//!   faces, each with the values of the cell behind it, so a large volume stays small.
//! - The point and cell data arrays become the fields of one step.
//! - The pieces of a parallel file and the blocks of a multiblock file are each a group.
//! - A time series reads the files of its first time step as the mesh and of the other steps
//!   only the fields; a step whose mesh has another number of points or cells is left out.

use crate::{stem, Side};
use fem_model::*;
use std::collections::BTreeMap;
use vtkio::model::{Attribute, Attributes, DataSet, ElementType, Extent, IOBuffer, Piece, VertexNumbers};
use vtkio::Vtk;

/// Reads a VTK file `name` (its bytes `data`), with the files it refers to from `side`.
pub fn read(name: &str, data: &[u8], side: Side) -> Result<Document, String> {
    let mut reader = Reader { side, notes: Vec::new(), skipped: BTreeMap::new() };
    let mesh = match crate::extension(name).as_str() {
        "pvd" => reader.series(name, data)?,
        "vtm" => reader.multiblock(name, data, "")?,
        _ => reader.file(name, data, "")?,
    };
    if mesh.cells.is_empty() {
        return Err("no cells".into());
    }
    let mut notes = reader.notes;
    for (kind, n) in reader.skipped {
        notes.push(format!("{n} cells of VTK type {kind} left out: this kind of cell is not drawn"));
    }
    Ok(Document { mesh: Some(mesh), analysis: None, format: "VTK".into(), notes })
}

struct Reader<'a> {
    side: Side<'a>,
    notes: Vec<String>,
    /// Cells left out, by their VTK type number.
    skipped: BTreeMap<u8, usize>,
}

/// `rel` as seen from the folder `base` (both relative to the first file's folder), with `.`
/// and `..` worked out.
fn join(base: &str, rel: &str) -> String {
    let rel = rel.replace('\\', "/");
    if rel.starts_with('/') || rel.contains(':') {
        return rel;
    }
    let mut parts: Vec<&str> = base.split('/').filter(|p| !p.is_empty()).collect();
    for p in rel.split('/') {
        match p {
            "" | "." => {}
            ".." if parts.last().is_some_and(|l| *l != "..") => {
                parts.pop();
            }
            _ => parts.push(p),
        }
    }
    parts.join("/")
}

/// The folder of a path relative to the first file's folder.
fn folder(path: &str) -> &str {
    path.rfind('/').map_or("", |k| &path[..k])
}

/// A legacy or an XML VTK file.
fn parse(data: &[u8]) -> Result<Vtk, String> {
    let start = data.iter().position(|b| !b.is_ascii_whitespace()).unwrap_or(0);
    let head = &data[start..data.len().min(start + 512)];
    let legacy = head.starts_with(b"# vtk") || head.starts_with(b"# VTK");
    let converted = if legacy { None } else { crate::vtk_raw::to_base64(data) };
    let data = converted.as_deref().unwrap_or(data);
    let parsed = if legacy { Vtk::parse_legacy_be(data) } else { Vtk::parse_xml(data) };
    parsed.map_err(|e| format!("not a VTK file it can read: {e}"))
}

fn buffer(b: &IOBuffer) -> Option<Vec<f64>> {
    b.cast_into::<f64>()
}

/// The points of a buffer of x y z triples.
fn points(b: &IOBuffer) -> Result<Vec<[f64; 3]>, String> {
    let v = buffer(b).ok_or("points that are not numbers")?;
    Ok(v.chunks_exact(3).map(|p| [p[0], p[1], p[2]]).collect())
}

/// The data arrays of points and cells as fields; an array of another length than there are
/// points or cells is left out.
fn fields(attributes: &Attributes, points: usize, cells: usize) -> Vec<Field> {
    let mut out = Vec::new();
    for (at, list, count) in [(FieldAt::Point, &attributes.point, points), (FieldAt::Cell, &attributes.cell, cells)] {
        for attribute in list {
            let arrays: Vec<(String, usize, &IOBuffer)> = match attribute {
                Attribute::DataArray(d) if !matches!(d.elem, ElementType::LookupTable) => vec![(d.name.clone(), d.elem.num_comp().max(1) as usize, &d.data)],
                Attribute::Field { data_array, .. } => data_array.iter().map(|a| (a.name.clone(), a.elem.max(1) as usize, &a.data)).collect(),
                _ => Vec::new(),
            };
            for (name, components, data) in arrays {
                let Some(values) = data.cast_into::<f32>() else { continue };
                if values.len() == count * components && count > 0 {
                    out.push(Field { name, at, components, values });
                }
            }
        }
    }
    out
}

/// The values of a field at the points or cells `from` (indices into its own).
fn pick(field: &Field, from: &[u32]) -> Field {
    let c = field.components.max(1);
    let mut values = Vec::with_capacity(from.len() * c);
    for &k in from {
        let k = k as usize * c;
        values.extend_from_slice(field.values.get(k..k + c).unwrap_or(&vec![f32::NAN; c]));
    }
    Field { values, ..field.clone() }
}

/// The number of points of a grid along each axis.
fn dims(extent: &Extent) -> [usize; 3] {
    extent.clone().into_dims().map(|d| d.max(1) as usize)
}

impl Reader<'_> {
    /// One VTK data file (a serial or a parallel one); `base` is its folder.
    fn file(&mut self, name: &str, data: &[u8], base: &str) -> Result<Mesh, String> {
        let vtk = parse(data)?;
        let mut mesh = self.dataset(vtk.data, base)?;
        mesh.name = stem(name);
        if mesh.groups.len() == 1 {
            mesh.groups[0] = stem(name);
        }
        Ok(mesh)
    }

    /// A file a piece, block or step refers to.
    fn refer(&mut self, path: &str) -> Result<Mesh, String> {
        let data = (self.side)(path)?;
        match crate::extension(path).as_str() {
            "vtm" => self.multiblock(path, &data, folder(path)),
            _ => self.file(path, &data, folder(path)),
        }
    }

    fn dataset(&mut self, data: DataSet, base: &str) -> Result<Mesh, String> {
        match data {
            DataSet::UnstructuredGrid { pieces, .. } => self.pieces(pieces, base, |r, p| r.unstructured(p)),
            DataSet::PolyData { pieces, .. } => self.pieces(pieces, base, |r, p| r.poly(p)),
            DataSet::StructuredGrid { pieces, .. } => self.pieces(pieces, base, |_, p| {
                let [nx, ny, _] = dims(&p.extent);
                let pts = points(&p.points)?;
                Ok(grid(dims(&p.extent), |i, j, k| pts.get(i + nx * (j + ny * k)).copied().unwrap_or([f64::NAN; 3]), &p.data))
            }),
            DataSet::RectilinearGrid { pieces, .. } => self.pieces(pieces, base, |_, p| {
                let (x, y, z) = (buffer(&p.coords.x).unwrap_or_default(), buffer(&p.coords.y).unwrap_or_default(), buffer(&p.coords.z).unwrap_or_default());
                let at = |v: &[f64], i: usize| v.get(i).copied().unwrap_or(0.0);
                Ok(grid(dims(&p.extent), |i, j, k| [at(&x, i), at(&y, j), at(&z, k)], &p.data))
            }),
            DataSet::ImageData { origin, spacing, pieces, .. } => self.pieces(pieces, base, |_, p| {
                let low = match &p.extent {
                    Extent::Ranges(r) => [*r[0].start(), *r[1].start(), *r[2].start()],
                    Extent::Dims(_) => [0; 3],
                };
                let at = |a: usize, i: usize| origin[a] as f64 + spacing[a] as f64 * (low[a] as f64 + i as f64);
                Ok(grid(dims(&p.extent), |i, j, k| [at(0, i), at(1, j), at(2, k)], &p.data))
            }),
            DataSet::Field { .. } => Err("a file of data arrays only: no points or cells".into()),
        }
    }

    /// The pieces of a data set, each a group; a piece in a file of its own is read from it.
    fn pieces<P>(&mut self, pieces: Vec<Piece<P>>, base: &str, inline: impl Fn(&mut Self, P) -> Result<Mesh, String>) -> Result<Mesh, String> {
        let several = pieces.len() > 1;
        let mut mesh = Mesh::default();
        for (k, piece) in pieces.into_iter().enumerate() {
            let mut part = match piece {
                Piece::Inline(p) => inline(self, *p)?,
                Piece::Loaded(data) => self.dataset(*data, base)?,
                Piece::Source(path, _) => self.refer(&join(base, &path))?,
            };
            if several && part.groups.len() == 1 && part.groups[0].is_empty() {
                part.groups[0] = format!("piece {}", k + 1);
            }
            if part.groups.is_empty() && !part.cells.is_empty() {
                part.groups.push(format!("piece {}", k + 1));
            }
            mesh.append(part);
        }
        Ok(mesh)
    }

    /// Adds cell `k` of VTK type `kind` with nodes `n` to `cells`, noting which cell of the
    /// file each new cell is in `source`.
    fn cell(&mut self, cells: &mut Cells, source: &mut Vec<u32>, k: usize, kind: u8, n: &[u32]) {
        let mut add = |cells: &mut Cells, kind: CellKind, nodes: &[u32]| {
            cells.push(kind, nodes, 0);
            source.push(k as u32);
        };
        let first = |m: usize| n.get(..m).filter(|s| s.len() == m);
        match kind {
            1 | 2 => n.iter().for_each(|i| add(cells, CellKind::Vertex, &[*i])),
            3 | 21 | 35 => {
                if let Some(s) = first(2) {
                    add(cells, CellKind::Line, s)
                }
            }
            4 if n.len() >= 2 => add(cells, CellKind::Line, n),
            5 | 22 | 34 => {
                if let Some(s) = first(3) {
                    add(cells, CellKind::Triangle, s)
                }
            }
            6 => {
                for t in 0..n.len().saturating_sub(2) {
                    let tri = if t % 2 == 0 { [n[t], n[t + 1], n[t + 2]] } else { [n[t + 1], n[t], n[t + 2]] };
                    add(cells, CellKind::Triangle, &tri);
                }
            }
            7 if n.len() >= 3 => add(cells, if n.len() == 3 { CellKind::Triangle } else if n.len() == 4 { CellKind::Quad } else { CellKind::Polygon }, n),
            8 if n.len() >= 4 => add(cells, CellKind::Quad, &[n[0], n[1], n[3], n[2]]),
            9 | 23 | 28 | 30 => {
                if let Some(s) = first(4) {
                    add(cells, CellKind::Quad, s)
                }
            }
            10 | 24 => {
                if let Some(s) = first(4) {
                    add(cells, CellKind::Tetra, s)
                }
            }
            11 if n.len() >= 8 => add(cells, CellKind::Hexa, &[n[0], n[1], n[3], n[2], n[4], n[5], n[7], n[6]]),
            12 | 25 | 29 | 33 => {
                if let Some(s) = first(8) {
                    add(cells, CellKind::Hexa, s)
                }
            }
            13 | 26 | 31 | 32 => {
                if let Some(s) = first(6) {
                    add(cells, CellKind::Wedge, s)
                }
            }
            14 | 27 => {
                if let Some(s) = first(5) {
                    add(cells, CellKind::Pyramid, s)
                }
            }
            _ => *self.skipped.entry(kind).or_default() += 1,
        }
    }

    fn unstructured(&mut self, p: vtkio::model::UnstructuredGridPiece) -> Result<Mesh, String> {
        let pts = points(&p.points)?;
        let (connectivity, offsets) = p.cells.cell_verts.into_xml();
        let total = p.cells.types.len();
        let (mut cells, mut source) = (Cells::default(), Vec::new());
        let mut start = 0usize;
        for (k, kind) in p.cells.types.iter().enumerate() {
            let end = offsets.get(k).map_or(start, |&o| o as usize).min(connectivity.len());
            let nodes: Vec<u32> = connectivity[start.min(end)..end].iter().map(|&i| i as u32).collect();
            start = end;
            if nodes.iter().any(|&i| i as usize >= pts.len()) {
                continue;
            }
            self.cell(&mut cells, &mut source, k, *kind as u8, &nodes);
        }
        Ok(with_fields(pts, cells, &source, &p.data, total))
    }

    fn poly(&mut self, p: vtkio::model::PolyDataPiece) -> Result<Mesh, String> {
        let pts = points(&p.points)?;
        let (mut cells, mut source) = (Cells::default(), Vec::new());
        let mut k = 0usize;
        // the cell data run over the vertices, lines, polygons and strips, in that order
        for (numbers, kind) in [(p.verts, 2u8), (p.lines, 4), (p.polys, 7), (p.strips, 6)] {
            let Some(numbers) = numbers else { continue };
            let (connectivity, offsets) = match numbers {
                VertexNumbers::XML { .. } | VertexNumbers::Legacy { .. } => numbers.into_xml(),
            };
            let mut start = 0usize;
            for &end in &offsets {
                let end = (end as usize).min(connectivity.len());
                let nodes: Vec<u32> = connectivity[start.min(end)..end].iter().map(|&i| i as u32).collect();
                start = end;
                if !nodes.iter().any(|&i| i as usize >= pts.len()) {
                    self.cell(&mut cells, &mut source, k, kind, &nodes);
                }
                k += 1;
            }
        }
        Ok(with_fields(pts, cells, &source, &p.data, k))
    }

    /// A multiblock file: every data set it names, each as a group named after its block.
    fn multiblock(&mut self, name: &str, data: &[u8], base: &str) -> Result<Mesh, String> {
        let mut mesh = Mesh { name: stem(name), ..Mesh::default() };
        let sets = crate::xml::elements(data, b"DataSet")?;
        if sets.is_empty() {
            return Err("a multiblock file without data sets".into());
        }
        for set in sets {
            let Some(file) = set.get("file").filter(|f| !f.is_empty()) else { continue };
            let path = join(base, file);
            let mut part = match self.refer(&path) {
                Ok(part) => part,
                Err(e) => {
                    self.notes.push(format!("{path}: {e}"));
                    continue;
                }
            };
            let label = set.get("name").filter(|n| !n.is_empty()).cloned().unwrap_or_else(|| stem(file));
            match part.groups.len() {
                1 => part.groups[0] = label,
                _ => part.groups.iter_mut().for_each(|g| *g = format!("{label} · {g}")),
            }
            mesh.append(part);
        }
        Ok(mesh)
    }

    /// A time series: the mesh of its first step with the fields of every step.
    fn series(&mut self, name: &str, data: &[u8]) -> Result<Mesh, String> {
        let sets = crate::xml::elements(data, b"DataSet")?;
        // the files of each time, in the order of the times
        let mut times: Vec<(f64, Vec<String>)> = Vec::new();
        for set in &sets {
            let Some(file) = set.get("file").filter(|f| !f.is_empty()) else { continue };
            let time = set.get("timestep").and_then(|t| t.trim().parse::<f64>().ok()).unwrap_or(times.len() as f64);
            match times.iter_mut().find(|(t, _)| *t == time) {
                Some((_, files)) => files.push(file.clone()),
                None => times.push((time, vec![file.clone()])),
            }
        }
        times.sort_by(|a, b| a.0.total_cmp(&b.0));
        if times.is_empty() {
            return Err("a time series without data sets".into());
        }
        let mut mesh: Option<Mesh> = None;
        for (time, files) in times {
            let mut step = Mesh::default();
            for file in &files {
                step.append(self.refer(&join("", file))?);
            }
            let fields = step.steps.into_iter().next().map(|s| s.fields).unwrap_or_default();
            match &mut mesh {
                None => {
                    step.steps = vec![Step { time: Some(time), fields }];
                    step.name = stem(name);
                    mesh = Some(step);
                }
                Some(first) if first.points.len() == step.points.len() && first.cells.len() == step.cells.len() => first.steps.push(Step { time: Some(time), fields }),
                Some(_) => self.notes.push(format!("time {time}: another mesh than the first step, left out")),
            }
        }
        mesh.ok_or_else(|| "no time steps".into())
    }
}

/// A mesh of `points` and `cells`, with the fields of `data` (for `cells_in_file` cells of the
/// file, which `source` maps the new cells to).
fn with_fields(points: Vec<[f64; 3]>, cells: Cells, source: &[u32], data: &Attributes, cells_in_file: usize) -> Mesh {
    let fields: Vec<Field> = fields(data, points.len(), cells_in_file).into_iter().map(|f| if f.at == FieldAt::Cell { pick(&f, source) } else { f }).collect();
    let steps = if fields.is_empty() { Vec::new() } else { vec![Step { time: None, fields }] };
    let groups = if cells.is_empty() { Vec::new() } else { vec![String::new()] };
    Mesh { name: String::new(), points, cells, groups, steps }
}

/// A grid of `n` points along x, y and z, point (i, j, k) at `at(i, j, k)`. In three dimensions
/// only its outer faces, each with the cell behind it; otherwise all its cells.
fn grid(n: [usize; 3], at: impl Fn(usize, usize, usize) -> [f64; 3], data: &Attributes) -> Mesh {
    let [nx, ny, nz] = n;
    let cells_along = n.map(|d| d.saturating_sub(1).max(1));
    let point_of = |i: usize, j: usize, k: usize| (i + nx * (j + ny * k)) as u32;
    let cell_of = |i: usize, j: usize, k: usize| (i + cells_along[0] * (j + cells_along[1] * k)) as u32;
    let total_points = nx * ny * nz;
    let total_cells = cells_along.iter().product::<usize>();
    let fields = fields(data, total_points, total_cells);
    // the points used, by their index in the grid
    let mut used: Vec<u32> = Vec::new();
    let mut new_index = std::collections::HashMap::new();
    let mut cells = Cells::default();
    let mut source = Vec::new();
    let mut add = |cells: &mut Cells, kind: CellKind, corners: &[u32], cell: u32| {
        let nodes: Vec<u32> = corners
            .iter()
            .map(|&c| {
                *new_index.entry(c).or_insert_with(|| {
                    used.push(c);
                    used.len() as u32 - 1
                })
            })
            .collect();
        cells.push(kind, &nodes, 0);
        source.push(cell);
    };
    let flat = n.iter().filter(|d| **d > 1).count();
    if flat == 3 {
        // the six sides; each quad turned outward from the middle of the cell behind it
        let point = |c: u32| {
            let c = c as usize;
            at(c % nx, (c / nx) % ny, c / (nx * ny))
        };
        let mut side = |cells: &mut Cells, quad: [u32; 4], cell: (usize, usize, usize)| {
            let (i, j, k) = cell;
            let middle = [point_of(i, j, k), point_of(i + 1, j + 1, k + 1)].map(point);
            let middle = [0, 1, 2].map(|a| (middle[0][a] + middle[1][a]) / 2.0);
            let p = quad.map(point);
            let e1 = [0, 1, 2].map(|a| p[1][a] - p[0][a]);
            let e2 = [0, 1, 2].map(|a| p[3][a] - p[0][a]);
            let normal = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
            let out: f64 = (0..3).map(|a| normal[a] * (p[0][a] - middle[a])).sum();
            let quad = if out < 0.0 { [quad[0], quad[3], quad[2], quad[1]] } else { quad };
            add(cells, CellKind::Quad, &quad, cell_of(i, j, k));
        };
        for (i, ci) in [(0, 0), (nx - 1, nx - 2)] {
            for k in 0..nz - 1 {
                for j in 0..ny - 1 {
                    side(&mut cells, [point_of(i, j, k), point_of(i, j + 1, k), point_of(i, j + 1, k + 1), point_of(i, j, k + 1)], (ci, j, k));
                }
            }
        }
        for (j, cj) in [(0, 0), (ny - 1, ny - 2)] {
            for k in 0..nz - 1 {
                for i in 0..nx - 1 {
                    side(&mut cells, [point_of(i, j, k), point_of(i + 1, j, k), point_of(i + 1, j, k + 1), point_of(i, j, k + 1)], (i, cj, k));
                }
            }
        }
        for (k, ck) in [(0, 0), (nz - 1, nz - 2)] {
            for j in 0..ny - 1 {
                for i in 0..nx - 1 {
                    side(&mut cells, [point_of(i, j, k), point_of(i + 1, j, k), point_of(i + 1, j + 1, k), point_of(i, j + 1, k)], (i, j, ck));
                }
            }
        }
    } else {
        // a plane, a line or a point of cells: all of them
        let step = |d: usize| if n[d] > 1 { 1 } else { 0 };
        for k in 0..cells_along[2] {
            for j in 0..cells_along[1] {
                for i in 0..cells_along[0] {
                    let (di, dj, dk) = (step(0), step(1), step(2));
                    let corner = |a: usize, b: usize, c: usize| point_of(i + a * di, j + b * dj, k + c * dk);
                    let cell = cell_of(i, j, k);
                    match flat {
                        2 => {
                            // the two axes along which the grid runs
                            let quad = if dk == 0 {
                                [corner(0, 0, 0), corner(1, 0, 0), corner(1, 1, 0), corner(0, 1, 0)]
                            } else if dj == 0 {
                                [corner(0, 0, 0), corner(1, 0, 0), corner(1, 0, 1), corner(0, 0, 1)]
                            } else {
                                [corner(0, 0, 0), corner(0, 1, 0), corner(0, 1, 1), corner(0, 0, 1)]
                            };
                            add(&mut cells, CellKind::Quad, &quad, cell);
                        }
                        1 => add(&mut cells, CellKind::Line, &[corner(0, 0, 0), corner(1, 1, 1)], cell),
                        _ => add(&mut cells, CellKind::Vertex, &[corner(0, 0, 0)], cell),
                    }
                }
            }
        }
    }
    let points: Vec<[f64; 3]> = used
        .iter()
        .map(|&c| {
            let c = c as usize;
            at(c % nx, (c / nx) % ny, c / (nx * ny))
        })
        .collect();
    let fields: Vec<Field> = fields.into_iter().map(|f| if f.at == FieldAt::Point { pick(&f, &used) } else { pick(&f, &source) }).collect();
    let steps = if fields.is_empty() { Vec::new() } else { vec![Step { time: None, fields }] };
    Mesh { name: String::new(), points, cells, groups: vec![String::new()], steps }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_side(path: &str) -> Result<Vec<u8>, String> {
        Err(format!("{path}: no files beside it"))
    }

    const LEGACY: &str = "# vtk DataFile Version 3.0
two cells
ASCII
DATASET UNSTRUCTURED_GRID
POINTS 5 float
0 0 0  1 0 0  0 1 0  0 0 1  1 1 1
CELLS 2 9
4 0 1 2 3
3 1 2 4
CELL_TYPES 2
10
5
POINT_DATA 5
SCALARS temperature float 1
LOOKUP_TABLE default
10 20 30 40 50
VECTORS displacement float
1 0 0  0 2 0  0 0 3  0 0 0  1 1 1
CELL_DATA 2
SCALARS stress float
LOOKUP_TABLE default
100 200
";

    #[test]
    fn reads_a_legacy_unstructured_grid_with_its_fields() {
        let doc = read("block.vtk", LEGACY.as_bytes(), &no_side).expect("legacy file");
        let mesh = doc.mesh.unwrap();
        assert_eq!((mesh.points.len(), mesh.cells.len()), (5, 2));
        assert_eq!(mesh.cells.kind, [CellKind::Tetra, CellKind::Triangle]);
        let fields = &mesh.steps[0].fields;
        let names: Vec<&str> = fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["temperature", "displacement", "stress"]);
        assert_eq!((fields[1].components, fields[1].magnitude(2)), (3, 3.0));
        assert_eq!((fields[2].at, fields[2].values.clone()), (FieldAt::Cell, vec![100.0, 200.0]));
        let surfaces = mesh.surfaces();
        assert_eq!(surfaces[0].triangles.len() / 3, 5, "the four faces of the tetrahedron and the triangle");
    }

    fn vtu(points: &str, temperature: &str) -> String {
        format!(
            r#"<?xml version="1.0"?>
<VTKFile type="UnstructuredGrid" version="0.1" byte_order="LittleEndian">
  <UnstructuredGrid>
    <Piece NumberOfPoints="4" NumberOfCells="1">
      <PointData Scalars="T">
        <DataArray type="Float32" Name="T" format="ascii">{temperature}</DataArray>
      </PointData>
      <Points>
        <DataArray type="Float32" NumberOfComponents="3" format="ascii">{points}</DataArray>
      </Points>
      <Cells>
        <DataArray type="Int32" Name="connectivity" format="ascii">0 1 2 3</DataArray>
        <DataArray type="Int32" Name="offsets" format="ascii">4</DataArray>
        <DataArray type="UInt8" Name="types" format="ascii">9</DataArray>
      </Cells>
    </Piece>
  </UnstructuredGrid>
</VTKFile>"#
        )
    }

    #[test]
    fn reads_an_xml_unstructured_grid() {
        let doc = read("quad.vtu", vtu("0 0 0 1 0 0 1 1 0 0 1 0", "1 2 3 4").as_bytes(), &no_side).expect("vtu");
        let mesh = doc.mesh.unwrap();
        assert_eq!(mesh.cells.kind, [CellKind::Quad]);
        assert_eq!(mesh.groups, ["quad"]);
        assert_eq!(mesh.steps[0].fields[0].values, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn a_time_series_reads_the_fields_of_every_step() {
        let pvd = r#"<?xml version="1.0"?>
<VTKFile type="Collection" version="0.1">
  <Collection>
    <DataSet timestep="0.5" part="0" file="out/step_1.vtu"/>
    <DataSet timestep="0" part="0" file="out/step_0.vtu"/>
  </Collection>
</VTKFile>"#;
        let side = |path: &str| -> Result<Vec<u8>, String> {
            match path {
                "out/step_0.vtu" => Ok(vtu("0 0 0 1 0 0 1 1 0 0 1 0", "1 1 1 1").into_bytes()),
                "out/step_1.vtu" => Ok(vtu("0 0 0 1 0 0 1 1 0 0 1 0", "5 6 7 8").into_bytes()),
                _ => Err(format!("{path}: not there")),
            }
        };
        let mesh = read("run.pvd", pvd.as_bytes(), &side).expect("series").mesh.unwrap();
        assert_eq!(mesh.name, "run");
        assert_eq!(mesh.steps.len(), 2);
        assert_eq!(mesh.steps.iter().map(|s| s.time).collect::<Vec<_>>(), [Some(0.0), Some(0.5)]);
        assert_eq!(mesh.steps[1].fields[0].values, [5.0, 6.0, 7.0, 8.0]);
        // the files beside it are needed
        assert!(read("run.pvd", pvd.as_bytes(), &no_side).unwrap_err().contains("no files beside it"));
    }

    #[test]
    fn a_multiblock_file_makes_a_group_per_block() {
        let vtm = r#"<VTKFile type="vtkMultiBlockDataSet" version="1.0">
  <vtkMultiBlockDataSet>
    <Block index="0" name="walls">
      <DataSet index="0" name="left" file="parts/a.vtu"/>
    </Block>
    <DataSet index="1" name="right" file="parts/b.vtu"/>
  </vtkMultiBlockDataSet>
</VTKFile>"#;
        let side = |path: &str| -> Result<Vec<u8>, String> {
            match path {
                "parts/a.vtu" => Ok(vtu("0 0 0 1 0 0 1 1 0 0 1 0", "1 1 1 1").into_bytes()),
                "parts/b.vtu" => Ok(vtu("2 0 0 3 0 0 3 1 0 2 1 0", "2 2 2 2").into_bytes()),
                _ => Err(format!("{path}: not there")),
            }
        };
        let mesh = read("house.vtm", vtm.as_bytes(), &side).expect("multiblock").mesh.unwrap();
        assert_eq!(mesh.groups, ["left", "right"]);
        assert_eq!((mesh.points.len(), mesh.cells.len()), (8, 2));
        assert_eq!(mesh.cells.nodes(1), [4, 5, 6, 7]);
        assert_eq!(mesh.steps[0].fields[0].values, [1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0]);
    }

    #[test]
    fn an_image_volume_keeps_its_outer_faces_with_the_cell_values() {
        let vti = r#"<?xml version="1.0"?>
<VTKFile type="ImageData" version="0.1" byte_order="LittleEndian">
  <ImageData WholeExtent="0 2 0 2 0 2" Origin="0 0 0" Spacing="1 1 1">
    <Piece Extent="0 2 0 2 0 2">
      <CellData Scalars="density">
        <DataArray type="Float32" Name="density" format="ascii">1 2 3 4 5 6 7 8</DataArray>
      </CellData>
    </Piece>
  </ImageData>
</VTKFile>"#;
        let mesh = read("ct.vti", vti.as_bytes(), &no_side).expect("vti").mesh.unwrap();
        // 3 × 3 × 3 points, 2 × 2 × 2 cells: 24 outer quads on 26 outer points
        assert_eq!((mesh.cells.len(), mesh.points.len()), (24, 26));
        let density = &mesh.steps[0].fields[0];
        assert_eq!(density.count(), 24);
        assert!(density.values.contains(&8.0) && density.values.contains(&1.0));
        let s = &mesh.surfaces()[0];
        for t in s.triangles.chunks_exact(3) {
            let (p, n) = (s.positions[t[0] as usize], s.normals[t[0] as usize]);
            let out = [p[0] - 1.0, p[1] - 1.0, p[2] - 1.0];
            assert!(out[0] * n[0] as f64 + out[1] * n[1] as f64 + out[2] * n[2] as f64 > 0.0, "{p:?} {n:?}");
        }
    }

    #[test]
    fn a_polygon_file_draws_lines_and_faces() {
        let vtk = "# vtk DataFile Version 2.0
lines and a face
ASCII
DATASET POLYDATA
POINTS 4 float
0 0 0 1 0 0 1 1 0 0 1 0
LINES 1 4
3 0 1 2
POLYGONS 1 5
4 0 1 2 3
CELL_DATA 2
SCALARS id int 1
LOOKUP_TABLE default
7 8
";
        let mesh = read("p.vtk", vtk.as_bytes(), &no_side).expect("polydata").mesh.unwrap();
        assert_eq!(mesh.cells.kind, [CellKind::Line, CellKind::Quad]);
        assert_eq!(mesh.steps[0].fields[0].values, [7.0, 8.0]);
    }

    #[test]
    fn paths_are_joined_from_the_folder_of_the_file_that_names_them() {
        assert_eq!(join("", "a/b.vtu"), "a/b.vtu");
        assert_eq!(join("out", "../c.vtu"), "c.vtu");
        assert_eq!(join("out/x", "./d\\e.vtu"), "out/x/d/e.vtu");
    }
}
