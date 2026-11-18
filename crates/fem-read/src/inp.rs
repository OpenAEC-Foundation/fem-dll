//! The FEM input deck (`.inp`): lines of keywords that start with `*` (with parameters after
//! commas, `*ELEMENT, TYPE=S4, ELSET=FLOOR`) and the data lines under them; `**` starts a
//! comment.
//!
//! What is read:
//!
//! - `*NODE`, `*ELEMENT` (by its type: beams and trusses become members, shells and membranes
//!   plates, solids and plane elements cells of the mesh), `*NSET` and `*ELSET` (also with
//!   `GENERATE`), `*INCLUDE`.
//! - `*PART`, `*INSTANCE` (moved, not turned) and `*ASSEMBLY`: an instance's nodes and elements
//!   are named `instance.number`.
//! - `*SHELL SECTION` (thickness), `*BEAM SECTION` (rectangle, circle, pipe, box, I) and
//!   `*SOLID SECTION`, each with its material; `*MATERIAL` with `*ELASTIC` and `*DENSITY`.
//! - `*BOUNDARY` (degrees of freedom, and the named kinds `ENCASTRE`, `PINNED` and the symmetry
//!   ones), `*CLOAD` (forces and moments on nodes), `*DLOAD` with `P` on shells (a pressure) and
//!   `GRAV` (the weight); each `*STEP` is a load case.
//!
//! The deck has no units: they are taken as SI (metres, newtons).

use crate::build::{corners, Builder, Placed, Role};
use crate::Side;
use fem_model::*;
use std::collections::HashMap;

/// A keyword line: the keyword in capitals and its parameters (names in capitals).
struct Keyword {
    name: String,
    params: HashMap<String, String>,
}

impl Keyword {
    fn parse(line: &str) -> Keyword {
        let mut parts = line[1..].split(',');
        let name = parts.next().unwrap_or_default().trim().to_ascii_uppercase();
        let params = parts
            .filter_map(|p| {
                let (k, v) = p.split_once('=').map_or((p, ""), |(k, v)| (k, v));
                let k = k.trim().to_ascii_uppercase();
                (!k.is_empty()).then(|| (k, v.trim().trim_matches('"').to_string()))
            })
            .collect();
        Keyword { name, params }
    }

    fn get(&self, key: &str) -> Option<&str> {
        self.params.get(key).map(String::as_str)
    }

    fn has(&self, key: &str) -> bool {
        self.params.contains_key(key)
    }
}

/// The cell kind of an element type, its number of nodes, and what it is.
fn element_type(t: &str) -> Option<(CellKind, usize, Role)> {
    let t = t.to_ascii_uppercase();
    let digits = |s: &str| -> Option<usize> { s.chars().take_while(|c| c.is_ascii_digit()).collect::<String>().parse().ok() };
    let solid = |n: usize| -> Option<CellKind> {
        Some(match n {
            4 | 10 => CellKind::Tetra,
            5 | 13 => CellKind::Pyramid,
            6 | 15 => CellKind::Wedge,
            8 | 20 | 27 => CellKind::Hexa,
            _ => return None,
        })
    };
    let face = |n: usize| -> Option<CellKind> {
        Some(match n {
            3 | 6 => CellKind::Triangle,
            4 | 8 | 9 => CellKind::Quad,
            _ => return None,
        })
    };
    if let Some(rest) = t.strip_prefix("C3D") {
        let n = digits(rest)?;
        return Some((solid(n)?, n, Role::Cell));
    }
    if let Some(rest) = t.strip_prefix("SC") {
        let n = digits(rest)?;
        return Some((solid(n)?, n, Role::Cell));
    }
    for (prefix, role) in [("STRI", Role::Plate(PlateKind::Shell)), ("S", Role::Plate(PlateKind::Shell)), ("M3D", Role::Plate(PlateKind::Membrane)), ("R3D", Role::Cell), ("CPS", Role::Cell), ("CPE", Role::Cell), ("CAX", Role::Cell), ("DS", Role::Cell)] {
        if let Some(rest) = t.strip_prefix(prefix) {
            if let Some(n) = digits(rest) {
                let n = if prefix == "STRI" { 3 } else { n };
                if let Some(kind) = face(n) {
                    return Some((kind, n, role));
                }
            }
        }
    }
    if let Some(rest) = t.strip_prefix("B") {
        // B31, B32, B33 (3D), B21, B22, B23 (plane): the last digit is the order
        let n = match rest.as_bytes().get(1) {
            Some(b'2') => 3,
            Some(_) => 2,
            None => return None,
        };
        return Some((CellKind::Line, n, Role::Member(MemberKind::Beam)));
    }
    for prefix in ["T3D", "T2D"] {
        if let Some(rest) = t.strip_prefix(prefix) {
            return Some((CellKind::Line, digits(rest)?, Role::Member(MemberKind::Truss)));
        }
    }
    if t.starts_with("SPRING1") || t.starts_with("MASS") || t.starts_with("ROTARYI") {
        return Some((CellKind::Vertex, 1, Role::Cell));
    }
    if t.starts_with("SPRING") || t.starts_with("DASHPOT") || t.starts_with("CONN") {
        return Some((CellKind::Line, 2, Role::Cell));
    }
    None
}

/// The lines of the deck and of the files it includes, without comments.
fn lines(text: &str, side: Side, depth: usize, out: &mut Vec<String>, notes: &mut Vec<String>) {
    for line in text.lines() {
        let line = line.trim_end_matches('\r');
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("**") {
            continue;
        }
        if trimmed.starts_with('*') {
            let k = Keyword::parse(trimmed);
            if k.name == "INCLUDE" {
                match k.get("INPUT") {
                    Some(file) if depth < 8 => match side(file) {
                        Ok(data) => lines(&String::from_utf8_lossy(&data), side, depth + 1, out, notes),
                        Err(e) => notes.push(format!("included file {file}: {e}")),
                    },
                    _ => {}
                }
                continue;
            }
        }
        out.push(trimmed.to_string());
    }
}

fn number(s: &str) -> Option<f64> {
    s.trim().replace(['d', 'D'], "e").parse::<f64>().ok().filter(|v| v.is_finite())
}

/// The degrees of freedom a named boundary holds.
fn named_boundary(name: &str) -> Option<Vec<usize>> {
    Some(match name.to_ascii_uppercase().as_str() {
        "ENCASTRE" => vec![0, 1, 2, 3, 4, 5],
        "PINNED" => vec![0, 1, 2],
        "XSYMM" => vec![0, 4, 5],
        "YSYMM" => vec![1, 3, 5],
        "ZSYMM" => vec![2, 3, 4],
        "XASYMM" => vec![1, 2, 3],
        "YASYMM" => vec![0, 2, 4],
        "ZASYMM" => vec![0, 1, 5],
        _ => return None,
    })
}

/// A section of an element set.
enum SectionOf {
    Shell { thickness: Option<f64>, material: Option<usize> },
    Beam { section: usize },
    Solid { material: Option<usize> },
}

pub fn read(name: &str, data: &[u8], side: Side) -> Result<Document, String> {
    let text = String::from_utf8_lossy(data);
    let mut notes = Vec::new();
    let mut all = Vec::new();
    lines(&text, side, 0, &mut all, &mut notes);
    if !all.iter().any(|l| l.starts_with('*')) {
        return Err("no keywords: not an input deck".into());
    }

    // first pass: the parts, so instances can place them
    let mut b = Builder::default();
    let mut nsets: HashMap<String, Vec<String>> = HashMap::new();
    let mut elsets: HashMap<String, Vec<String>> = HashMap::new();
    // elements as read: name, type, node names, element set
    struct Raw {
        name: String,
        kind: (CellKind, usize, Role),
        nodes: Vec<String>,
        elset: String,
    }
    let mut raw: Vec<Raw> = Vec::new();
    let mut sections: Vec<(String, SectionOf)> = Vec::new();
    let mut materials: HashMap<String, usize> = HashMap::new();
    // the parts: their lines, to be read once per instance
    let mut parts: HashMap<String, Vec<String>> = HashMap::new();
    let mut skipped_types: HashMap<String, usize> = HashMap::new();
    // loads and boundaries, read once the nodes are known
    let mut later: Vec<(Option<String>, Keyword, Vec<Vec<String>>)> = Vec::new();

    let mut k = 0;
    // the lines outside parts, and those of each instance with its prefix and move
    let mut flat: Vec<(String, [f64; 3], String)> = Vec::new();
    {
        let mut in_part: Option<String> = None;
        let mut instance: Option<(String, String, Option<[f64; 3]>)> = None;
        while k < all.len() {
            let line = &all[k];
            k += 1;
            if line.starts_with('*') {
                let kw = Keyword::parse(line);
                match kw.name.as_str() {
                    "PART" => {
                        in_part = kw.get("NAME").map(str::to_string);
                        continue;
                    }
                    "END PART" => {
                        in_part = None;
                        continue;
                    }
                    "INSTANCE" => {
                        let name = kw.get("NAME").unwrap_or("instance").to_string();
                        let part = kw.get("PART").unwrap_or(&name).to_string();
                        // the move on the next data line, the turn (left out) on the one after
                        let mut shift = None;
                        if k < all.len() && !all[k].starts_with('*') {
                            let v: Vec<f64> = all[k].split(',').filter_map(number).collect();
                            if v.len() >= 3 {
                                shift = Some([v[0], v[1], v[2]]);
                            }
                            k += 1;
                            if k < all.len() && !all[k].starts_with('*') {
                                notes.push(format!("instance {name}: its turn is left out"));
                                k += 1;
                            }
                        }
                        instance = Some((name, part, shift));
                        continue;
                    }
                    "END INSTANCE" => {
                        if let Some((name, part, shift)) = instance.take() {
                            for l in parts.get(&part).cloned().unwrap_or_default() {
                                flat.push((format!("{name}."), shift.unwrap_or([0.0; 3]), l));
                            }
                        }
                        continue;
                    }
                    _ => {}
                }
            }
            match &in_part {
                Some(part) => parts.entry(part.clone()).or_default().push(line.clone()),
                None if instance.is_some() => {}
                None => flat.push((String::new(), [0.0; 3], line.clone())),
            }
        }
    }

    // second pass: the model, keyword by keyword
    let mut kw = Keyword { name: String::new(), params: HashMap::new() };
    let mut prefix = String::new();
    let mut shift = [0.0; 3];
    let mut step: Option<String> = None;
    let mut material: Option<usize> = None;
    let mut i = 0;
    while i < flat.len() {
        let (pre, sh, line) = &flat[i];
        i += 1;
        if line.starts_with('*') {
            kw = Keyword::parse(line);
            prefix = pre.clone();
            shift = *sh;
            match kw.name.as_str() {
                "STEP" => step = Some(kw.get("NAME").map(str::to_string).unwrap_or_else(|| format!("Step {}", b.analysis.cases.len() + 1))),
                "END STEP" => step = None,
                "MATERIAL" => {
                    let name = kw.get("NAME").unwrap_or("material").to_string();
                    b.analysis.materials.push(Material { name: name.clone(), ..Material::default() });
                    material = Some(b.analysis.materials.len() - 1);
                    materials.insert(name.to_ascii_uppercase(), b.analysis.materials.len() - 1);
                }
                "SHELL SECTION" | "BEAM SECTION" | "SOLID SECTION" | "MEMBRANE SECTION" => {
                    let elset = format!("{prefix}{}", kw.get("ELSET").unwrap_or_default()).to_ascii_uppercase();
                    let mat = kw.get("MATERIAL").and_then(|m| materials.get(&m.to_ascii_uppercase()).copied());
                    let data: Vec<f64> = flat.get(i).filter(|l| !l.2.starts_with('*')).map(|l| l.2.split(',').filter_map(number).collect()).unwrap_or_default();
                    let section = match kw.name.as_str() {
                        "SHELL SECTION" | "MEMBRANE SECTION" => SectionOf::Shell { thickness: data.first().copied(), material: mat },
                        "SOLID SECTION" => SectionOf::Solid { material: mat },
                        _ => {
                            let kind = kw.get("SECTION").unwrap_or_default().to_ascii_uppercase();
                            let d = |k: usize| data.get(k).copied().unwrap_or(0.0);
                            let shape = match kind.as_str() {
                                "RECT" => Shape::Rectangle { b: d(0), h: d(1) },
                                "CIRC" => Shape::Circle { d: 2.0 * d(0) },
                                "PIPE" => Shape::Tube { d: 2.0 * d(0), t: d(1) },
                                "BOX" => Shape::Box { b: d(0), h: d(1), t: d(2) },
                                // height, offset, bottom and top width, bottom and top flange, web
                                "I" => Shape::I { h: d(1), b: d(2).max(d(3)), tw: d(6), tf: d(4).max(d(5)) },
                                _ => Shape::Unknown,
                            };
                            b.analysis.sections.push(Section { name: format!("{kind} {}", kw.get("ELSET").unwrap_or_default()).trim().to_string(), shape, props: SectionProps::default(), material: mat });
                            SectionOf::Beam { section: b.analysis.sections.len() - 1 }
                        }
                    };
                    sections.push((elset, section));
                }
                "BOUNDARY" | "CLOAD" | "DLOAD" => later.push((step.clone(), Keyword { name: kw.name.clone(), params: kw.params.clone() }, Vec::new())),
                _ => {}
            }
            continue;
        }
        let fields: Vec<&str> = line.split(',').map(str::trim).collect();
        // an assembly names the nodes and elements of an instance with INSTANCE=
        let member = kw.get("INSTANCE").map_or_else(|| prefix.clone(), |inst| format!("{inst}."));
        match kw.name.as_str() {
            "NODE" => {
                let (Some(id), Some(x)) = (fields.first(), fields.get(1).and_then(|v| number(v))) else { continue };
                let y = fields.get(2).and_then(|v| number(v)).unwrap_or(0.0);
                let z = fields.get(3).and_then(|v| number(v)).unwrap_or(0.0);
                let node = format!("{prefix}{id}");
                b.node(&node, [x + shift[0], y + shift[1], z + shift[2]]);
                if let Some(set) = kw.get("NSET") {
                    nsets.entry(format!("{prefix}{set}").to_ascii_uppercase()).or_default().push(node);
                }
            }
            "ELEMENT" => {
                let t = kw.get("TYPE").unwrap_or_default();
                // an element over several lines: the line ends in a comma, or too few nodes
                let mut values: Vec<String> = fields.iter().filter(|f| !f.is_empty()).map(|s| s.to_string()).collect();
                let kind = element_type(t);
                let want = kind.map(|k| k.1 + 1);
                while i < flat.len() && !flat[i].2.starts_with('*') && (line_ends_open(&flat[i - 1].2) || want.is_some_and(|w| values.len() < w)) {
                    values.extend(flat[i].2.split(',').map(str::trim).filter(|f| !f.is_empty()).map(str::to_string));
                    i += 1;
                }
                let Some(kind) = kind else {
                    *skipped_types.entry(t.to_ascii_uppercase()).or_default() += 1;
                    continue;
                };
                let Some((id, nodes)) = values.split_first() else { continue };
                let elset = kw.get("ELSET").map(|s| format!("{prefix}{s}")).unwrap_or_else(|| t.to_ascii_uppercase());
                let name = format!("{prefix}{id}");
                elsets.entry(elset.to_ascii_uppercase()).or_default().push(name.clone());
                raw.push(Raw { name, kind, nodes: nodes.iter().map(|n| format!("{prefix}{n}")).collect(), elset });
            }
            "NSET" | "ELSET" => {
                let key = if kw.name == "NSET" { "NSET" } else { "ELSET" };
                let set = format!("{prefix}{}", kw.get(key).unwrap_or_default()).to_ascii_uppercase();
                let names: Vec<String> = if kw.has("GENERATE") {
                    let v: Vec<i64> = fields.iter().filter_map(|f| f.parse().ok()).collect();
                    match v.as_slice() {
                        [a, z, rest @ ..] => {
                            let step = rest.first().copied().unwrap_or(1).max(1);
                            (*a..=*z).step_by(step as usize).map(|n| format!("{member}{n}")).collect()
                        }
                        _ => Vec::new(),
                    }
                } else {
                    // a member is a number or the name of another set
                    let mut out = Vec::new();
                    for f in fields.iter().filter(|f| !f.is_empty()) {
                        let other = format!("{prefix}{f}").to_ascii_uppercase();
                        let sets = if key == "NSET" { &nsets } else { &elsets };
                        match (f.parse::<i64>().is_err(), sets.get(&other)) {
                            (true, Some(members)) => out.extend(members.iter().cloned()),
                            _ => out.push(format!("{member}{f}")),
                        }
                    }
                    out
                };
                let sets = if key == "NSET" { &mut nsets } else { &mut elsets };
                sets.entry(set).or_default().extend(names);
            }
            "ELASTIC" => {
                if let (Some(m), Some(e)) = (material, fields.first().and_then(|v| number(v))) {
                    b.analysis.materials[m].e = Some(e);
                    b.analysis.materials[m].nu = fields.get(1).and_then(|v| number(v));
                }
            }
            "DENSITY" => {
                if let Some(m) = material {
                    b.analysis.materials[m].density = fields.first().and_then(|v| number(v));
                }
            }
            "EXPANSION" => {
                if let Some(m) = material {
                    b.analysis.materials[m].alpha = fields.first().and_then(|v| number(v));
                }
            }
            "BOUNDARY" | "CLOAD" | "DLOAD" => {
                if let Some(last) = later.last_mut() {
                    last.2.push(fields.iter().map(|f| format!("{member}\u{1}{f}")).collect());
                }
            }
            _ => {}
        }
    }

    // the elements, each with the section of its set
    let mut section_of: HashMap<String, usize> = HashMap::new();
    for (k, (elset, _)) in sections.iter().enumerate() {
        for e in elsets.get(elset).into_iter().flatten() {
            section_of.insert(e.clone(), k);
        }
    }
    for e in raw {
        let (kind, _, role) = e.kind;
        let Some(nodes) = e.nodes.iter().map(|n| b.node_at.get(n).copied()).collect::<Option<Vec<usize>>>() else {
            notes.push(format!("element {}: a node that is not there", e.name));
            continue;
        };
        let Some(nodes) = corners(kind, &nodes) else { continue };
        let (mut section, mut material, mut thickness) = (None, None, None);
        let mut role = role;
        match section_of.get(&e.name).map(|&k| &sections[k].1) {
            Some(SectionOf::Shell { thickness: t, material: m }) => (thickness, material) = (*t, *m),
            Some(SectionOf::Beam { section: s }) => (section, material) = (Some(*s), b.analysis.sections[*s].material),
            Some(SectionOf::Solid { material: m }) => {
                material = *m;
                // a plane element with a solid section is a cell; a shell type stays a plate
            }
            None => {}
        }
        if let (Role::Plate(_), Some(SectionOf::Solid { .. })) = (role, section_of.get(&e.name).map(|&k| &sections[k].1)) {
            role = Role::Cell;
        }
        b.element(&e.name, kind, nodes, role, &e.elset, section, material, thickness);
    }

    // boundaries and loads: on a node, or on every node or element of a set
    let nodes_of = |b: &Builder, what: &str| -> Vec<usize> {
        let (prefix, name) = what.split_once('\u{1}').unwrap_or(("", what));
        let full = format!("{prefix}{name}");
        match b.node_at.get(&full) {
            Some(&n) => vec![n],
            None => nsets.get(&full.to_ascii_uppercase()).into_iter().flatten().filter_map(|n| b.node_at.get(n).copied()).collect(),
        }
    };
    let elements_of = |b: &Builder, what: &str| -> Vec<Placed> {
        let (prefix, name) = what.split_once('\u{1}').unwrap_or(("", what));
        let full = format!("{prefix}{name}");
        match b.element_at.get(&full) {
            Some(&p) => vec![p],
            None => elsets.get(&full.to_ascii_uppercase()).into_iter().flatten().filter_map(|e| b.element_at.get(e).copied()).collect(),
        }
    };
    let plain = |s: &String| s.split_once('\u{1}').map_or(s.clone(), |(_, v)| v.to_string());
    let mut skipped: HashMap<String, usize> = HashMap::new();
    for (step, kw, rows) in later {
        let case = |b: &mut Builder| b.case(step.as_deref().unwrap_or("Initial"));
        for row in rows {
            let Some(target) = row.first() else { continue };
            let values: Vec<String> = row.iter().skip(1).map(plain).collect();
            match kw.name.as_str() {
                "BOUNDARY" => {
                    let dofs: Vec<usize> = match values.first().map(String::as_str) {
                        Some(named) if named.parse::<f64>().is_err() => match named_boundary(named) {
                            Some(d) => d,
                            None => continue,
                        },
                        _ => {
                            let first = values.first().and_then(|v| v.parse::<usize>().ok()).unwrap_or(1);
                            let last = values.get(1).and_then(|v| v.parse::<usize>().ok()).unwrap_or(first);
                            (first.max(1)..=last.min(6)).map(|d| d - 1).collect()
                        }
                    };
                    if step.is_some() && values.get(2).and_then(|v| number(v)).is_some_and(|v| v != 0.0) {
                        *skipped.entry("prescribed displacement (held, its value left out)".into()).or_default() += 1;
                    }
                    for n in nodes_of(&b, target) {
                        b.hold(n, dofs.iter().copied());
                    }
                }
                "CLOAD" => {
                    let (Some(dof), Some(value)) = (values.first().and_then(|v| v.parse::<usize>().ok()), values.get(1).and_then(|v| number(v))) else { continue };
                    if !(1..=6).contains(&dof) {
                        continue;
                    }
                    let c = case(&mut b);
                    for n in nodes_of(&b, target) {
                        let (mut force, mut moment) = ([0.0; 3], [0.0; 3]);
                        if dof <= 3 {
                            force[dof - 1] = value;
                        } else {
                            moment[dof - 4] = value;
                        }
                        b.analysis.actions.push(Action { case: c, load: Load::Node { node: n, force, moment } });
                    }
                }
                "DLOAD" => {
                    let label = values.first().map(|v| v.to_ascii_uppercase()).unwrap_or_default();
                    let magnitude = values.get(1).and_then(|v| number(v)).unwrap_or(0.0);
                    match label.as_str() {
                        "GRAV" => {
                            let dir: Vec<f64> = values.iter().skip(2).filter_map(|v| number(v)).collect();
                            let c = case(&mut b);
                            if dir.len() == 3 && dir[2] < 0.0 {
                                b.analysis.cases[c].self_weight = Some(magnitude * -dir[2] / 9.806_65);
                            } else {
                                *skipped.entry("gravity not along −z".into()).or_default() += 1;
                            }
                        }
                        "P" | "P1" | "P2" => {
                            let c = case(&mut b);
                            for e in elements_of(&b, target) {
                                match e {
                                    // a positive pressure pushes against the plate's normal
                                    Placed::Plate(plate) => b.analysis.actions.push(Action { case: c, load: Load::Plate { plate, axes: Axes::Local, dir: [0.0, 0.0, -1.0], q: magnitude } }),
                                    _ => *skipped.entry("pressure on a solid face".into()).or_default() += 1,
                                }
                            }
                        }
                        other => *skipped.entry(format!("distributed load {other}")).or_default() += 1,
                    }
                }
                _ => {}
            }
        }
    }
    for (t, n) in skipped_types {
        notes.push(format!("{n} elements of type {t}: this type is not read"));
    }
    for (what, n) in skipped {
        notes.push(format!("{n} × {what}"));
    }
    notes.push("the deck has no units: read as SI (metres, newtons)".into());
    b.finish("FEM input deck (.inp)", &crate::stem(name), notes)
}

/// True when a data line goes on on the next line.
fn line_ends_open(line: &str) -> bool {
    line.trim_end().ends_with(',')
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_side(path: &str) -> Result<Vec<u8>, String> {
        Err(format!("{path}: not there"))
    }

    const FRAME: &str = "** a portal frame on a floor, with a block
*HEADING
portal
*NODE, NSET=ALL
1, 0., 0., 0.
2, 0., 0., 3.
3, 4., 0., 3.
4, 4., 0., 0.
5, 0., 2., 3.
6, 4., 2., 3.
*ELEMENT, TYPE=B31, ELSET=COLUMNS
1, 1, 2
3, 3, 4
*ELEMENT, TYPE=B31, ELSET=BEAM
2, 2, 3
*ELEMENT, TYPE=S4R, ELSET=FLOOR
10, 2, 3, 6, 5
*NSET, NSET=FEET
1, 4
*MATERIAL, NAME=STEEL
*ELASTIC
2.1e11, 0.3
*DENSITY
7850.
*BEAM SECTION, ELSET=COLUMNS, MATERIAL=STEEL, SECTION=RECT
0.2, 0.3
0., 1., 0.
*BEAM SECTION, ELSET=BEAM, MATERIAL=STEEL, SECTION=PIPE
0.1, 0.01
*SHELL SECTION, ELSET=FLOOR, MATERIAL=STEEL
0.2
*BOUNDARY
FEET, ENCASTRE
*STEP, NAME=LIVE
*STATIC
*CLOAD
3, 1, 1000.
*DLOAD
FLOOR, P, 2500.
*END STEP
";

    #[test]
    fn beams_and_shells_become_an_analysis_model() {
        let doc = read("portal.inp", FRAME.as_bytes(), &no_side).expect("deck");
        assert!(doc.mesh.is_none(), "no solids");
        let a = doc.analysis.expect("analysis");
        assert_eq!((a.nodes.len(), a.members.len(), a.plates.len()), (6, 3, 1));
        let column = &a.members[0];
        assert_eq!(a.sections[column.section.unwrap()].shape, Shape::Rectangle { b: 0.2, h: 0.3 });
        let beam = a.members.iter().find(|m| m.name == "2").unwrap();
        assert_eq!(a.sections[beam.section.unwrap()].shape, Shape::Tube { d: 0.2, t: 0.01 });
        assert_eq!(a.plates[0].thickness, Some(0.2));
        assert_eq!(a.materials[0].e, Some(2.1e11));
        let held: Vec<&str> = a.nodes.iter().filter(|n| n.support == Some([Dof::Fixed; 6])).map(|n| n.name.as_str()).collect();
        assert_eq!(held, ["1", "4"]);
        assert_eq!(a.cases.len(), 1);
        assert_eq!(a.cases[0].name, "LIVE");
        assert!(a.actions.iter().any(|x| matches!(x.load, Load::Node { force, .. } if force == [1000.0, 0.0, 0.0])));
        assert!(a.actions.iter().any(|x| matches!(x.load, Load::Plate { q, .. } if q == 2500.0)));
    }

    #[test]
    fn solids_become_a_mesh_and_instances_move_their_part() {
        let deck = "*PART, NAME=BLOCK
*NODE
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 0, 0, 1
6, 1, 0, 1
7, 1, 1, 1
8, 0, 1, 1
*ELEMENT, TYPE=C3D8R, ELSET=CUBE
1, 1, 2, 3, 4, 5, 6, 7,
8
*END PART
*ASSEMBLY, NAME=A
*INSTANCE, NAME=LEFT, PART=BLOCK
*END INSTANCE
*INSTANCE, NAME=RIGHT, PART=BLOCK
5., 0., 0.
*END INSTANCE
*NSET, NSET=BASE, INSTANCE=LEFT
1, 2, 3, 4
*END ASSEMBLY
*BOUNDARY
LEFT.1, 1, 3
";
        let doc = read("blocks.inp", deck.as_bytes(), &no_side).expect("deck");
        let mesh = doc.mesh.expect("mesh");
        assert_eq!(mesh.cells.len(), 2);
        assert_eq!(mesh.cells.kind[0], CellKind::Hexa);
        assert_eq!(mesh.groups, ["LEFT.CUBE", "RIGHT.CUBE"]);
        let right = mesh.cells.nodes(1)[0] as usize;
        assert_eq!(mesh.points[right], [5.0, 0.0, 0.0]);
        let a = doc.analysis.expect("the support");
        assert_eq!(a.nodes[0].support.unwrap()[..3], [Dof::Fixed; 3]);
    }

    #[test]
    fn included_files_are_read_and_sets_are_generated() {
        let deck = "*INCLUDE, INPUT=nodes.inp
*ELEMENT, TYPE=T3D2, ELSET=BARS
1, 1, 2
2, 2, 3
*ELSET, ELSET=ALL, GENERATE
1, 2, 1
*NSET, NSET=ENDS
1, 3
*BOUNDARY
ENDS, PINNED
";
        let side = |path: &str| -> Result<Vec<u8>, String> {
            match path {
                "nodes.inp" => Ok(b"*NODE\n1, 0, 0, 0\n2, 1, 0, 0\n3, 2, 0, 0\n".to_vec()),
                _ => Err("not there".into()),
            }
        };
        let a = read("truss.inp", deck.as_bytes(), &side).expect("deck").analysis.unwrap();
        assert_eq!(a.members.len(), 2);
        assert_eq!(a.members[0].kind, MemberKind::Truss);
        assert_eq!(a.nodes.iter().filter(|n| n.support.is_some()).count(), 2);
    }
}
