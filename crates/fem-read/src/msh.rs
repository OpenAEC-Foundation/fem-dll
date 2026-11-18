//! The finite element mesh (`.msh`) in versions 2 and 4, ASCII and binary: sections between
//! `$Name` and `$EndName` lines.
//!
//! What is read:
//!
//! - `$MeshFormat`: the version, ASCII or binary, and the size of a size number.
//! - `$PhysicalNames`: the names of the physical groups, the groups of the mesh.
//! - `$Entities` (version 4): which physical group each point, curve, surface and volume is in.
//! - `$Nodes` and `$Elements`: points, lines, triangles, quadrangles, tetrahedra, hexahedra,
//!   prisms and pyramids, of the first order and higher (by their corners).
//! - `$NodeData` and `$ElementData` (ASCII): the fields, each time step a step.
//!
//! An element in no physical group is grouped by the entity it is in.

use fem_model::*;
use std::collections::{BTreeMap, HashMap};

/// The cell kind of an element type and its number of nodes.
fn element_type(t: i32) -> Option<(CellKind, usize)> {
    Some(match t {
        15 => (CellKind::Vertex, 1),
        1 => (CellKind::Line, 2),
        8 => (CellKind::Line, 3),
        26 => (CellKind::Line, 4),
        27 => (CellKind::Line, 5),
        28 => (CellKind::Line, 6),
        2 => (CellKind::Triangle, 3),
        9 => (CellKind::Triangle, 6),
        20 => (CellKind::Triangle, 9),
        21 => (CellKind::Triangle, 10),
        3 => (CellKind::Quad, 4),
        10 => (CellKind::Quad, 9),
        16 => (CellKind::Quad, 8),
        4 => (CellKind::Tetra, 4),
        11 => (CellKind::Tetra, 10),
        29 => (CellKind::Tetra, 20),
        5 => (CellKind::Hexa, 8),
        12 => (CellKind::Hexa, 27),
        17 => (CellKind::Hexa, 20),
        6 => (CellKind::Wedge, 6),
        13 => (CellKind::Wedge, 18),
        18 => (CellKind::Wedge, 15),
        7 => (CellKind::Pyramid, 5),
        14 => (CellKind::Pyramid, 14),
        19 => (CellKind::Pyramid, 13),
        _ => return None,
    })
}

/// Reads binary numbers of the file's byte order (little endian; the `1` the format writes
/// tells).
struct Bytes<'a> {
    data: &'a [u8],
    at: usize,
    size_t: usize,
    swap: bool,
}

impl Bytes<'_> {
    fn take(&mut self, n: usize) -> Result<&[u8], String> {
        let s = self.data.get(self.at..self.at + n).ok_or("the binary data ends early")?;
        self.at += n;
        Ok(s)
    }

    fn int(&mut self) -> Result<i32, String> {
        let b: [u8; 4] = self.take(4)?.try_into().unwrap();
        Ok(if self.swap { i32::from_be_bytes(b) } else { i32::from_le_bytes(b) })
    }

    fn size(&mut self) -> Result<u64, String> {
        match self.size_t {
            4 => self.int().map(|v| v as u64),
            _ => {
                let b: [u8; 8] = self.take(8)?.try_into().unwrap();
                Ok(if self.swap { u64::from_be_bytes(b) } else { u64::from_le_bytes(b) })
            }
        }
    }

    fn double(&mut self) -> Result<f64, String> {
        let b: [u8; 8] = self.take(8)?.try_into().unwrap();
        Ok(if self.swap { f64::from_be_bytes(b) } else { f64::from_le_bytes(b) })
    }
}

/// The sections of the file: their names and where their content starts and ends.
fn sections(data: &[u8]) -> Vec<(String, usize, usize)> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < data.len() {
        // a section title at the start of a line
        let Some(k) = data[at..].iter().position(|&b| b == b'$').map(|k| k + at) else { break };
        if k > 0 && data[k - 1] != b'\n' {
            at = k + 1;
            continue;
        }
        let end_of_line = data[k..].iter().position(|&b| b == b'\n').map_or(data.len(), |e| e + k);
        let name = String::from_utf8_lossy(&data[k + 1..end_of_line]).trim().to_string();
        if name.starts_with("End") {
            at = end_of_line;
            continue;
        }
        let close = format!("\n$End{name}");
        let content = (end_of_line + 1).min(data.len());
        let stop = data[content..].windows(close.len()).position(|w| w == close.as_bytes()).map_or(data.len(), |p| p + content);
        out.push((name, content, stop));
        at = stop + close.len();
    }
    out
}

fn words(s: &[u8]) -> impl Iterator<Item = &str> {
    std::str::from_utf8(s).unwrap_or("").split_ascii_whitespace()
}

pub fn read(name: &str, data: &[u8]) -> Result<Document, String> {
    let parts = sections(data);
    let section = |n: &str| parts.iter().find(|(s, _, _)| s == n).map(|&(_, a, b)| &data[a..b]);
    let format = section("MeshFormat").ok_or("no $MeshFormat: not a mesh file of this kind")?;
    let mut w = words(format.split(|&b| b == b'\n').next().unwrap_or(&[]));
    let version: f64 = w.next().and_then(|v| v.parse().ok()).ok_or("no version")?;
    let binary = w.next() == Some("1");
    let size_t: usize = w.next().and_then(|v| v.parse().ok()).unwrap_or(8);
    // the byte order: the 1 written after the format line
    let swap = binary && {
        let line_end = format.iter().position(|&b| b == b'\n').map_or(format.len(), |k| k + 1);
        format.get(line_end..line_end + 4).is_some_and(|b| b == [0, 0, 0, 1])
    };
    let v4 = version >= 4.0;
    if !(2.0..5.0).contains(&version) {
        return Err(format!("version {version} of the format is not read"));
    }
    let mut notes = Vec::new();

    // the names of the physical groups, by dimension and number
    let mut physical: HashMap<(i32, i32), String> = HashMap::new();
    if let Some(names) = section("PhysicalNames") {
        let text = String::from_utf8_lossy(names);
        for line in text.lines().skip(1) {
            let mut p = line.split_whitespace();
            if let (Some(dim), Some(tag)) = (p.next().and_then(|v| v.parse().ok()), p.next().and_then(|v| v.parse().ok())) {
                let name: String = p.collect::<Vec<_>>().join(" ").trim_matches('"').to_string();
                physical.insert((dim, tag), name);
            }
        }
    }

    // version 4: the physical group of each entity, by dimension and entity
    let mut entity_group: HashMap<(i32, i32), i32> = HashMap::new();
    if let (true, Some(entities)) = (v4, section("Entities")) {
        if binary {
            let mut b = Bytes { data: entities, at: 0, size_t, swap };
            let counts = [b.size()?, b.size()?, b.size()?, b.size()?];
            for (dim, &count) in counts.iter().enumerate() {
                for _ in 0..count {
                    let tag = b.int()?;
                    let coordinates = if dim == 0 && version >= 4.1 { 3 } else { 6 };
                    for _ in 0..coordinates {
                        b.double()?;
                    }
                    let n = b.size()?;
                    let tags: Vec<i32> = (0..n).map(|_| b.int()).collect::<Result<_, _>>()?;
                    if let Some(&first) = tags.first() {
                        entity_group.insert((dim as i32, tag), first);
                    }
                    if dim > 0 {
                        let bounding = b.size()?;
                        for _ in 0..bounding {
                            b.int()?;
                        }
                    }
                }
            }
        } else {
            let text = String::from_utf8_lossy(entities);
            let mut lines = text.lines();
            let counts: Vec<usize> = lines.next().unwrap_or("").split_whitespace().filter_map(|v| v.parse().ok()).collect();
            for (dim, &count) in counts.iter().enumerate().take(4) {
                for _ in 0..count {
                    let v: Vec<f64> = lines.next().unwrap_or("").split_whitespace().filter_map(|v| v.parse().ok()).collect();
                    let coordinates = if dim == 0 && version >= 4.1 { 3 } else { 6 };
                    let (Some(&tag), Some(&n)) = (v.first(), v.get(1 + coordinates)) else { continue };
                    if n >= 1.0 {
                        if let Some(&first) = v.get(2 + coordinates) {
                            entity_group.insert((dim as i32, tag as i32), first as i32);
                        }
                    }
                }
            }
        }
    }

    // the nodes
    let nodes = section("Nodes").ok_or("no $Nodes")?;
    let mut points: Vec<[f64; 3]> = Vec::new();
    let mut node_at: HashMap<u64, u32> = HashMap::new();
    let mut add = |tag: u64, p: [f64; 3]| {
        node_at.insert(tag, points.len() as u32);
        points.push(p);
    };
    if v4 {
        if binary {
            let mut b = Bytes { data: nodes, at: 0, size_t, swap };
            let (blocks, _, _, _) = (b.size()?, b.size()?, b.size()?, b.size()?);
            for _ in 0..blocks {
                let (dim, _entity, parametric, n) = (b.int()?, b.int()?, b.int()?, b.size()? as usize);
                let tags: Vec<u64> = (0..n).map(|_| b.size()).collect::<Result<_, _>>()?;
                let extra = if parametric != 0 { dim.max(0) as usize } else { 0 };
                for tag in tags {
                    let p = [b.double()?, b.double()?, b.double()?];
                    for _ in 0..extra {
                        b.double()?;
                    }
                    add(tag, p);
                }
            }
        } else {
            let text = String::from_utf8_lossy(nodes);
            let mut lines = text.lines();
            let blocks: usize = lines.next().unwrap_or("").split_whitespace().next().and_then(|v| v.parse().ok()).unwrap_or(0);
            for _ in 0..blocks {
                let head: Vec<usize> = lines.next().unwrap_or("").split_whitespace().filter_map(|v| v.parse().ok()).collect();
                let n = head.get(3).copied().unwrap_or(0);
                let tags: Vec<u64> = (0..n).filter_map(|_| lines.next()?.trim().parse().ok()).collect();
                for tag in tags {
                    let v: Vec<f64> = lines.next().unwrap_or("").split_whitespace().filter_map(|v| v.parse().ok()).collect();
                    add(tag, [v.first().copied().unwrap_or(0.0), v.get(1).copied().unwrap_or(0.0), v.get(2).copied().unwrap_or(0.0)]);
                }
            }
        }
    } else if binary {
        let mut b = Bytes { data: nodes, at: 0, size_t, swap };
        let header_end = nodes.iter().position(|&c| c == b'\n').ok_or("no node count")? + 1;
        let n: usize = words(&nodes[..header_end]).next().and_then(|v| v.parse().ok()).unwrap_or(0);
        b.at = header_end;
        for _ in 0..n {
            let tag = b.int()? as u64;
            add(tag, [b.double()?, b.double()?, b.double()?]);
        }
    } else {
        let text = String::from_utf8_lossy(nodes);
        for line in text.lines().skip(1) {
            let v: Vec<&str> = line.split_whitespace().collect();
            if let [tag, x, y, z, ..] = v.as_slice() {
                if let (Ok(tag), Ok(x), Ok(y), Ok(z)) = (tag.parse(), x.parse(), y.parse(), z.parse()) {
                    add(tag, [x, y, z]);
                }
            }
        }
    }

    // the elements, with their group
    let elements = section("Elements").ok_or("no $Elements")?;
    let mut mesh = Mesh { name: crate::stem(name), points: Vec::new(), ..Mesh::default() };
    let mut group_at: HashMap<String, u32> = HashMap::new();
    let mut element_at: HashMap<u64, u32> = HashMap::new();
    let mut skipped: BTreeMap<i32, usize> = BTreeMap::new();
    let dim_name = ["point", "curve", "surface", "volume"];
    let mut cell = |mesh: &mut Mesh, tag: u64, t: i32, group: String, nodes: &[u64]| {
        let Some((kind, _)) = element_type(t) else {
            *skipped.entry(t).or_default() += 1;
            return;
        };
        let Some(nodes) = nodes.iter().map(|n| node_at.get(n).copied()).collect::<Option<Vec<u32>>>() else { return };
        let nodes = match kind.corners() {
            Some(c) => nodes[..c.min(nodes.len())].to_vec(),
            // a line of higher order: its two ends come first
            None => nodes[..2.min(nodes.len())].to_vec(),
        };
        let next = group_at.len() as u32;
        let g = *group_at.entry(group.clone()).or_insert(next);
        if g as usize == mesh.groups.len() {
            mesh.groups.push(group);
        }
        element_at.insert(tag, mesh.cells.len() as u32);
        mesh.cells.push(kind, &nodes, g);
    };
    let group_of = |dim: i32, physical_tag: Option<i32>, entity: i32| -> String {
        match physical_tag.filter(|t| *t != 0) {
            Some(t) => physical.get(&(dim, t)).cloned().unwrap_or_else(|| format!("group {t}")),
            None => format!("{} {entity}", dim_name.get(dim as usize).unwrap_or(&"entity")),
        }
    };
    if v4 {
        if binary {
            let mut b = Bytes { data: elements, at: 0, size_t, swap };
            let (blocks, _, _, _) = (b.size()?, b.size()?, b.size()?, b.size()?);
            for _ in 0..blocks {
                let (dim, entity, t, n) = (b.int()?, b.int()?, b.int()?, b.size()? as usize);
                let count = element_type(t).map(|e| e.1).ok_or_else(|| format!("element type {t} is not read"))?;
                let group = group_of(dim, entity_group.get(&(dim, entity)).copied(), entity);
                for _ in 0..n {
                    let tag = b.size()?;
                    let nodes: Vec<u64> = (0..count).map(|_| b.size()).collect::<Result<_, _>>()?;
                    cell(&mut mesh, tag, t, group.clone(), &nodes);
                }
            }
        } else {
            let text = String::from_utf8_lossy(elements);
            let mut lines = text.lines();
            let blocks: usize = lines.next().unwrap_or("").split_whitespace().next().and_then(|v| v.parse().ok()).unwrap_or(0);
            for _ in 0..blocks {
                let head: Vec<i64> = lines.next().unwrap_or("").split_whitespace().filter_map(|v| v.parse().ok()).collect();
                let [dim, entity, t, n] = [0, 1, 2, 3].map(|k| head.get(k).copied().unwrap_or(0));
                let group = group_of(dim as i32, entity_group.get(&(dim as i32, entity as i32)).copied(), entity as i32);
                for _ in 0..n {
                    let v: Vec<u64> = lines.next().unwrap_or("").split_whitespace().filter_map(|v| v.parse().ok()).collect();
                    if let Some((tag, nodes)) = v.split_first() {
                        cell(&mut mesh, *tag, t as i32, group.clone(), nodes);
                    }
                }
            }
        }
    } else if binary {
        let header_end = elements.iter().position(|&c| c == b'\n').ok_or("no element count")? + 1;
        let total: usize = words(&elements[..header_end]).next().and_then(|v| v.parse().ok()).unwrap_or(0);
        let mut b = Bytes { data: elements, at: header_end, size_t: 4, swap };
        let mut read = 0;
        while read < total {
            let (t, n, tags) = (b.int()?, b.int()? as usize, b.int()? as usize);
            let count = element_type(t).map(|e| e.1).ok_or_else(|| format!("element type {t} is not read"))?;
            for _ in 0..n {
                let tag = b.int()? as u64;
                let given: Vec<i32> = (0..tags).map(|_| b.int()).collect::<Result<_, _>>()?;
                let nodes: Vec<u64> = (0..count).map(|_| b.int().map(|v| v as u64)).collect::<Result<_, _>>()?;
                let dim = element_type(t).map_or(0, |e| e.0.dimension() as i32);
                let group = group_of(dim, given.first().copied(), given.get(1).copied().unwrap_or(0));
                cell(&mut mesh, tag, t, group, &nodes);
            }
            read += n.max(1);
        }
    } else {
        let text = String::from_utf8_lossy(elements);
        for line in text.lines().skip(1) {
            let v: Vec<i64> = line.split_whitespace().filter_map(|v| v.parse().ok()).collect();
            let [tag, t, tags] = [0, 1, 2].map(|k| v.get(k).copied().unwrap_or(0));
            let Some(nodes) = v.get(3 + tags.max(0) as usize..) else { continue };
            let dim = element_type(t as i32).map_or(0, |e| e.0.dimension() as i32);
            let group = group_of(dim, v.get(3).map(|p| *p as i32).filter(|_| tags > 0), v.get(4).copied().unwrap_or(0) as i32);
            let nodes: Vec<u64> = nodes.iter().map(|n| *n as u64).collect();
            cell(&mut mesh, tag as u64, t as i32, group, &nodes);
        }
    }
    for (t, n) in skipped {
        notes.push(format!("{n} elements of type {t}: this type is not read"));
    }
    mesh.points = points;
    if mesh.cells.is_empty() {
        return Err("no elements".into());
    }

    // fields: per time step, the values at nodes or elements
    let mut steps: BTreeMap<i64, Step> = BTreeMap::new();
    for (title, a, z) in &parts {
        let at = match title.as_str() {
            "NodeData" => FieldAt::Point,
            "ElementData" => FieldAt::Cell,
            "ElementNodeData" => {
                notes.push("values per node of each element are not read".into());
                continue;
            }
            _ => continue,
        };
        if binary {
            notes.push(format!("the binary ${title} is not read"));
            continue;
        }
        let text = String::from_utf8_lossy(&data[*a..*z]);
        let mut lines = text.lines();
        let take = |lines: &mut std::str::Lines| -> Vec<String> {
            let n: usize = lines.next().and_then(|l| l.trim().parse().ok()).unwrap_or(0);
            (0..n).filter_map(|_| lines.next().map(|l| l.trim().trim_matches('"').to_string())).collect()
        };
        let strings = take(&mut lines);
        let reals = take(&mut lines);
        let ints = take(&mut lines);
        let field_name = strings.first().cloned().unwrap_or_else(|| title.clone());
        let time = reals.first().and_then(|t| t.parse::<f64>().ok());
        let step_number: i64 = ints.first().and_then(|v| v.parse().ok()).unwrap_or(0);
        let components: usize = ints.get(1).and_then(|v| v.parse().ok()).unwrap_or(1).max(1);
        let count = if at == FieldAt::Point { mesh.points.len() } else { mesh.cells.len() };
        let mut values = vec![f32::NAN; count * components];
        for line in lines {
            let mut v = line.split_whitespace();
            let Some(tag) = v.next().and_then(|t| t.parse::<u64>().ok()) else { continue };
            let slot = match at {
                FieldAt::Point => node_at.get(&tag).copied(),
                FieldAt::Cell => element_at.get(&tag).copied(),
            };
            let Some(slot) = slot else { continue };
            for (c, x) in v.take(components).enumerate() {
                values[slot as usize * components + c] = x.parse().unwrap_or(f32::NAN);
            }
        }
        let step = steps.entry(step_number).or_insert_with(|| Step { time, fields: Vec::new() });
        step.fields.push(Field { name: field_name, at, components, values });
    }
    mesh.steps = steps.into_values().collect();
    Ok(Document { mesh: Some(mesh), analysis: None, format: "Finite element mesh (.msh)".into(), notes })
}

#[cfg(test)]
mod tests {
    use super::*;

    const V2: &str = "$MeshFormat
2.2 0 8
$EndMeshFormat
$PhysicalNames
2
2 1 \"plate\"
3 2 \"block\"
$EndPhysicalNames
$Nodes
5
1 0 0 0
2 1 0 0
3 1 1 0
4 0 1 0
5 0 0 1
$EndNodes
$Elements
3
1 3 2 1 7 1 2 3 4
2 4 2 2 9 1 2 4 5
3 1 2 0 3 1 2
$EndElements
$NodeData
1
\"temperature\"
1
0.5
3
0
1
5
1 10
2 20
3 30
4 40
5 50
$EndNodeData
";

    #[test]
    fn reads_a_version_2_mesh_with_its_groups_and_a_field() {
        let doc = read("plate.msh", V2.as_bytes()).expect("mesh");
        let mesh = doc.mesh.unwrap();
        assert_eq!(mesh.cells.kind, [CellKind::Quad, CellKind::Tetra, CellKind::Line]);
        assert_eq!(mesh.groups, ["plate", "block", "curve 3"]);
        assert_eq!(mesh.steps.len(), 1);
        assert_eq!(mesh.steps[0].time, Some(0.5));
        assert_eq!(mesh.steps[0].fields[0].values, [10.0, 20.0, 30.0, 40.0, 50.0]);
    }

    const V4: &str = "$MeshFormat
4.1 0 8
$EndMeshFormat
$PhysicalNames
1
2 5 \"floor\"
$EndPhysicalNames
$Entities
0 0 1 0
1 0 0 0 1 1 0 1 5 0
$EndEntities
$Nodes
1 4 1 4
2 1 0 4
1
2
3
4
0 0 0
1 0 0
1 1 0
0 1 0
$EndNodes
$Elements
1 2 1 2
2 1 2 2
1 1 2 3
2 1 3 4
$EndElements
";

    #[test]
    fn reads_a_version_4_mesh_by_its_entities() {
        let mesh = read("floor.msh", V4.as_bytes()).expect("mesh").mesh.unwrap();
        assert_eq!(mesh.points.len(), 4);
        assert_eq!(mesh.cells.kind, [CellKind::Triangle, CellKind::Triangle]);
        assert_eq!(mesh.groups, ["floor"]);
    }

    /// The version 4 mesh above, written in binary.
    fn binary_v4() -> Vec<u8> {
        let mut d = b"$MeshFormat\n4.1 1 8\n".to_vec();
        d.extend_from_slice(&1i32.to_le_bytes());
        d.extend_from_slice(b"\n$EndMeshFormat\n$Entities\n");
        for c in [0u64, 0, 1, 0] {
            d.extend_from_slice(&c.to_le_bytes());
        }
        d.extend_from_slice(&1i32.to_le_bytes());
        for v in [0.0f64, 0.0, 0.0, 1.0, 1.0, 0.0] {
            d.extend_from_slice(&v.to_le_bytes());
        }
        d.extend_from_slice(&1u64.to_le_bytes());
        d.extend_from_slice(&5i32.to_le_bytes());
        d.extend_from_slice(&0u64.to_le_bytes());
        d.extend_from_slice(b"\n$EndEntities\n$Nodes\n");
        for v in [1u64, 4, 1, 4] {
            d.extend_from_slice(&v.to_le_bytes());
        }
        for v in [2i32, 1, 0] {
            d.extend_from_slice(&v.to_le_bytes());
        }
        d.extend_from_slice(&4u64.to_le_bytes());
        for t in 1u64..=4 {
            d.extend_from_slice(&t.to_le_bytes());
        }
        for p in [[0.0f64, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]] {
            for v in p {
                d.extend_from_slice(&v.to_le_bytes());
            }
        }
        d.extend_from_slice(b"\n$EndNodes\n$Elements\n");
        for v in [1u64, 1, 1, 1] {
            d.extend_from_slice(&v.to_le_bytes());
        }
        for v in [2i32, 1, 3] {
            d.extend_from_slice(&v.to_le_bytes());
        }
        d.extend_from_slice(&1u64.to_le_bytes());
        for v in [1u64, 1, 2, 3, 4] {
            d.extend_from_slice(&v.to_le_bytes());
        }
        d.extend_from_slice(b"\n$EndElements\n");
        d
    }

    #[test]
    fn reads_a_binary_version_4_mesh() {
        let mesh = read("floor.msh", &binary_v4()).expect("binary mesh").mesh.unwrap();
        assert_eq!(mesh.points[2], [1.0, 1.0, 0.0]);
        assert_eq!(mesh.cells.kind, [CellKind::Quad]);
        assert_eq!(mesh.groups, ["group 5"]);
    }
}
