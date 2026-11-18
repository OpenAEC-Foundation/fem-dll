//! FEM bulk data (`.bdf`, `.nas`, `.dat`, `.blk`): cards of fields, written in free field
//! (commas), small field (8 characters) or large field (16 characters, the card's name ending
//! in `*`), with continuation lines; `$` starts a comment. The cards before `BEGIN BULK` (the
//! executive and case control) are skipped.
//!
//! What is read:
//!
//! - `GRID` (in the basic coordinate system; a grid in another system is noted), `INCLUDE`.
//! - Members: `CBAR`, `CBEAM` (with their orientation vector), `CROD`, `CONROD`, `CTUBE`.
//! - Plates: `CQUAD4`, `CQUAD8`, `CQUADR`, `CTRIA3`, `CTRIA6`, `CTRIAR`, `CSHEAR`, with the
//!   thickness of their `PSHELL` (or the plies of their `PCOMP`).
//! - Cells of the mesh: `CHEXA`, `CTETRA`, `CPENTA`, `CPYRAM`, grouped by their property.
//! - Sections: `PBAR`, `PBEAM`, `PROD`, `PTUBE` (area and moments) and `PBARL`, `PBEAML` (their
//!   shapes ROD, TUBE, BAR, BOX, I, CHAN, T, L); materials `MAT1`.
//! - Supports `SPC`, `SPC1`; loads `FORCE`, `MOMENT`, `PLOAD4` and `PLOAD2` on plates, `GRAV`,
//!   each set of a load set number a load case; `LOAD` combinations.
//!
//! The file has no units: they are taken as SI (metres, newtons).

use crate::build::{corners, Builder, Placed, Role};
use crate::Side;
use fem_model::*;
use std::collections::HashMap;

/// A number as the format writes it: `1.5`, `1.5E-3`, `1.5-3`, `.5+2`, `1.D3`.
fn real(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    if let Ok(v) = s.parse::<f64>() {
        return Some(v);
    }
    let s = s.replace(['d', 'D'], "E");
    if let Ok(v) = s.parse::<f64>() {
        return Some(v);
    }
    // an exponent without its E
    let bytes = s.as_bytes();
    let at = (1..bytes.len()).rev().find(|&k| (bytes[k] == b'+' || bytes[k] == b'-') && bytes[k - 1] != b'E' && bytes[k - 1] != b'e')?;
    format!("{}E{}", &s[..at], &s[at..]).parse().ok()
}

fn int(s: &str) -> Option<i64> {
    s.trim().parse().ok()
}

/// The cards of the bulk data: each its name in capitals and its data fields (continuations
/// joined).
fn cards(text: &str, side: Side, depth: usize, out: &mut Vec<(String, Vec<String>)>, notes: &mut Vec<String>, bulk: &mut bool) {
    let mut lines = text.lines().peekable();
    while let Some(line) = lines.next() {
        let line = line.split('$').next().unwrap_or_default().trim_end_matches('\r');
        if line.trim().is_empty() {
            continue;
        }
        let upper = line.trim_start().to_ascii_uppercase();
        if !*bulk {
            if upper.starts_with("BEGIN") && upper.contains("BULK") {
                *bulk = true;
            }
            continue;
        }
        if upper.starts_with("ENDDATA") {
            break;
        }
        if upper.starts_with("INCLUDE") {
            let file = line.trim_start()[7..].trim().trim_matches(['\'', '"']).to_string();
            match side(&file) {
                Ok(data) if depth < 8 => {
                    let mut inner = true;
                    cards(&String::from_utf8_lossy(&data), side, depth + 1, out, notes, &mut inner);
                }
                Ok(_) => {}
                Err(e) => notes.push(format!("included file {file}: {e}")),
            }
            continue;
        }
        let (name, mut fields, large) = split(line);
        // the continuation lines that follow
        while let Some(next) = lines.peek() {
            let next = next.split('$').next().unwrap_or_default().trim_end_matches('\r');
            let first = next.chars().next();
            let continues = matches!(first, Some('+') | Some('*')) || (first == Some(' ') && !next.trim().is_empty() && !large_free(next)) || (first == Some(',') && next.contains(','));
            if !continues {
                break;
            }
            lines.next();
            let (_, more, _) = split(next);
            fields.extend(more);
        }
        let _ = large;
        out.push((name, fields));
    }
}

/// A line that starts with blanks and then a card name: no continuation.
fn large_free(line: &str) -> bool {
    let t = line.trim_start();
    t.chars().next().is_some_and(|c| c.is_ascii_alphabetic())
}

/// The name and the data fields of one line, and whether it is in large field.
fn split(line: &str) -> (String, Vec<String>, bool) {
    if line.contains(',') {
        let mut parts = line.split(',').map(|s| s.trim().to_string());
        let name = parts.next().unwrap_or_default();
        let large = name.ends_with('*');
        let mut fields: Vec<String> = parts.collect();
        // a free-field continuation marker at the end of a line of nine fields
        if fields.len() > 8 {
            fields.truncate(8);
        }
        return (name.trim_end_matches('*').trim_start_matches('+').to_ascii_uppercase(), fields, large);
    }
    let chars: Vec<char> = line.chars().collect();
    let field = |from: usize, width: usize| -> String { chars.iter().skip(from).take(width).collect::<String>().trim().to_string() };
    let name = field(0, 8);
    let large = name.ends_with('*');
    let fields = if large { (0..4).map(|k| field(8 + 16 * k, 16)).collect() } else { (0..8).map(|k| field(8 + 8 * k, 8)).collect() };
    (name.trim_end_matches('*').trim_start_matches(['+', '*']).to_ascii_uppercase(), fields, large)
}

/// The components a field of digits names (`123456`), as degrees of freedom 0 to 5.
fn components(s: &str) -> Vec<usize> {
    s.chars().filter_map(|c| c.to_digit(10)).filter(|d| (1..=6).contains(d)).map(|d| d as usize - 1).collect()
}

pub fn read(name: &str, data: &[u8], side: Side) -> Result<Document, String> {
    let text = String::from_utf8_lossy(data);
    let mut notes = Vec::new();
    let mut all = Vec::new();
    // a file of bulk data only has no BEGIN BULK
    let mut bulk = !text.lines().any(|l| {
        let u = l.trim_start().to_ascii_uppercase();
        u.starts_with("BEGIN") && u.contains("BULK")
    });
    cards(&text, side, 0, &mut all, &mut notes, &mut bulk);
    if !all.iter().any(|(n, _)| n == "GRID") {
        return Err("no GRID cards: not bulk data".into());
    }
    let mut b = Builder::default();
    fn f(fields: &[String], k: usize) -> &str {
        fields.get(k).map(String::as_str).unwrap_or("")
    }

    // properties and materials first: the elements refer to them
    let mut materials: HashMap<i64, usize> = HashMap::new();
    for (card, fields) in &all {
        if card == "MAT1" {
            let Some(id) = int(f(fields, 0)) else { continue };
            b.analysis.materials.push(Material { name: format!("MAT1 {id}"), e: real(f(fields, 1)), g: real(f(fields, 2)), nu: real(f(fields, 3)), density: real(f(fields, 4)), alpha: real(f(fields, 5)) });
            materials.insert(id, b.analysis.materials.len() - 1);
        }
    }
    // a property: its section (members), thickness (plates) and material
    #[derive(Default, Clone, Copy)]
    struct Property {
        section: Option<usize>,
        thickness: Option<f64>,
        material: Option<usize>,
    }
    let mut properties: HashMap<i64, Property> = HashMap::new();
    for (card, fields) in &all {
        let Some(pid) = int(f(fields, 0)) else { continue };
        let material = int(f(fields, 1)).and_then(|m| materials.get(&m).copied());
        let section = |b: &mut Builder, shape: Shape, props: SectionProps| {
            b.analysis.sections.push(Section { name: format!("{card} {pid}"), shape, props, material });
            Some(b.analysis.sections.len() - 1)
        };
        let property = match card.as_str() {
            "PSHELL" => Property { thickness: real(f(fields, 2)), material, ..Property::default() },
            "PCOMP" => {
                // the plies: material, thickness, angle, output, from the ninth field on
                let t: f64 = fields.iter().skip(8).step_by(4).skip(0).zip(fields.iter().skip(9).step_by(4)).filter_map(|(_, t)| real(t)).sum();
                Property { thickness: (t > 0.0).then_some(t), ..Property::default() }
            }
            "PBAR" | "PBEAM" => {
                let props = SectionProps { area: real(f(fields, 2)), iy: real(f(fields, 3)), iz: real(f(fields, 4)), it: real(f(fields, if card == "PBAR" { 5 } else { 6 })) };
                Property { section: section(&mut b, Shape::Unknown, props), material, ..Property::default() }
            }
            "PROD" => Property { section: section(&mut b, Shape::Unknown, SectionProps { area: real(f(fields, 2)), it: real(f(fields, 3)), ..SectionProps::default() }), material, ..Property::default() },
            "PTUBE" => {
                let (d, t) = (real(f(fields, 2)).unwrap_or(0.0), real(f(fields, 3)).unwrap_or(0.0));
                let shape = if t > 0.0 && t < d / 2.0 { Shape::Tube { d, t } } else { Shape::Circle { d } };
                Property { section: section(&mut b, shape, SectionProps::default()), material, ..Property::default() }
            }
            "PBARL" | "PBEAML" => {
                let kind = f(fields, 3).to_ascii_uppercase();
                let d: Vec<f64> = fields.iter().skip(8).filter_map(|v| real(v)).collect();
                let v = |k: usize| d.get(k).copied().unwrap_or(0.0);
                let shape = match kind.as_str() {
                    "ROD" => Shape::Circle { d: 2.0 * v(0) },
                    "TUBE" => Shape::Tube { d: 2.0 * v(0), t: v(0) - v(1) },
                    "BAR" => Shape::Rectangle { b: v(0), h: v(1) },
                    "BOX" => Shape::Box { b: v(0), h: v(1), t: v(2).max(v(3)) },
                    "I" => Shape::I { h: v(0), b: v(1).max(v(2)), tw: v(3), tf: v(4).max(v(5)) },
                    "CHAN" | "CHAN1" | "CHAN2" => Shape::Channel { h: v(1), b: v(0), tw: v(2), tf: v(3) },
                    "T" | "T1" | "T2" => Shape::Tee { h: v(1), b: v(0), tw: v(3), tf: v(2) },
                    "L" => Shape::Angle { h: v(1), b: v(0), t: v(2).max(v(3)) },
                    _ => Shape::Unknown,
                };
                Property { section: section(&mut b, shape, SectionProps::default()), material, ..Property::default() }
            }
            "PSOLID" => Property { material, ..Property::default() },
            _ => continue,
        };
        properties.insert(pid, property);
    }

    // the grids
    let mut other_systems = 0;
    for (card, fields) in &all {
        if card != "GRID" {
            continue;
        }
        let Some(id) = int(f(fields, 0)) else { continue };
        if int(f(fields, 1)).is_some_and(|cp| cp != 0) {
            other_systems += 1;
        }
        let p = [real(f(fields, 2)).unwrap_or(0.0), real(f(fields, 3)).unwrap_or(0.0), real(f(fields, 4)).unwrap_or(0.0)];
        let node = b.node(&id.to_string(), p);
        // components held at the grid itself
        let held = components(f(fields, 6));
        if !held.is_empty() {
            b.hold(node, held);
        }
    }
    if other_systems > 0 {
        notes.push(format!("{other_systems} grids in a coordinate system of their own: placed as if in the basic one"));
    }

    // the elements
    let mut skipped: HashMap<String, usize> = HashMap::new();
    for (card, fields) in &all {
        let (kind, role, nodes_from, count) = match card.as_str() {
            "CBAR" => (CellKind::Line, Role::Member(MemberKind::Beam), 2, 2),
            "CBEAM" => (CellKind::Line, Role::Member(MemberKind::Beam), 2, 2),
            "CROD" | "CTUBE" => (CellKind::Line, Role::Member(MemberKind::Truss), 2, 2),
            "CONROD" => (CellKind::Line, Role::Member(MemberKind::Truss), 1, 2),
            "CQUAD4" | "CQUADR" | "CSHEAR" | "CQUAD8" => (CellKind::Quad, Role::Plate(PlateKind::Shell), 2, 4),
            "CTRIA3" | "CTRIAR" | "CTRIA6" => (CellKind::Triangle, Role::Plate(PlateKind::Shell), 2, 3),
            "CHEXA" => (CellKind::Hexa, Role::Cell, 2, 8),
            "CTETRA" => (CellKind::Tetra, Role::Cell, 2, 4),
            "CPENTA" => (CellKind::Wedge, Role::Cell, 2, 6),
            "CPYRAM" => (CellKind::Pyramid, Role::Cell, 2, 5),
            c if c.starts_with('C') && c.len() > 2 && !c.starts_with("CORD") => {
                *skipped.entry(c.to_string()).or_default() += 1;
                continue;
            }
            _ => continue,
        };
        let Some(eid) = int(f(fields, 0)) else { continue };
        let pid = int(f(fields, 1));
        let nodes: Option<Vec<usize>> = (0..count).map(|k| int(f(fields, nodes_from + k)).and_then(|g| b.node_at.get(&g.to_string()).copied())).collect();
        let Some(nodes) = nodes.and_then(|n| corners(kind, &n)) else {
            notes.push(format!("{card} {eid}: a grid that is not there"));
            continue;
        };
        let property = if card == "CONROD" { None } else { pid.and_then(|p| properties.get(&p).copied()) };
        let (section, thickness, mut material) = property.map_or((None, None, None), |p| (p.section, p.thickness, p.material));
        if card == "CONROD" {
            material = pid.and_then(|m| materials.get(&m).copied());
        }
        let group = match role {
            Role::Cell => format!("{} {}", if card == "CHEXA" || card == "CTETRA" || card == "CPENTA" || card == "CPYRAM" { "PSOLID" } else { card.as_str() }, pid.unwrap_or(0)),
            _ => String::new(),
        };
        let placed = b.element(&eid.to_string(), kind, nodes, role, &group, section, material, thickness);
        // a bar's or beam's orientation: a vector, or a grid it points to
        if let (Placed::Member(m), "CBAR" | "CBEAM") = (placed, card.as_str()) {
            let x: Vec<Option<f64>> = (4..7).map(|k| real(f(fields, k))).collect();
            let orientation = match (int(f(fields, 4)), x[1], x[2]) {
                (Some(g0), None, None) => b.node_at.get(&g0.to_string()).map(|&n| {
                    let (from, to) = (b.analysis.nodes[b.analysis.members[m].nodes[0]].p, b.analysis.nodes[n].p);
                    [to[0] - from[0], to[1] - from[1], to[2] - from[2]]
                }),
                (_, Some(_), Some(_)) => Some([x[0].unwrap_or(0.0), x[1].unwrap_or(0.0), x[2].unwrap_or(0.0)]),
                _ => None,
            };
            b.analysis.members[m].orientation = orientation;
        }
    }
    for (c, n) in skipped {
        notes.push(format!("{n} × {c}: this element is not read"));
    }

    // supports
    for (card, fields) in &all {
        match card.as_str() {
            "SPC" => {
                // set, then grid and components, twice
                for pair in [(1, 2), (4, 5)] {
                    if let Some(&n) = int(f(fields, pair.0)).and_then(|g| b.node_at.get(&g.to_string())) {
                        let held = components(f(fields, pair.1));
                        b.hold(n, held);
                    }
                }
            }
            "SPC1" => {
                let held = components(f(fields, 1));
                let rest: Vec<&str> = fields.iter().skip(2).map(String::as_str).filter(|s| !s.is_empty()).collect();
                let grids: Vec<i64> = match rest.as_slice() {
                    [a, thru, z, ..] if thru.eq_ignore_ascii_case("THRU") => match (int(a), int(z)) {
                        (Some(a), Some(z)) => (a..=z).collect(),
                        _ => Vec::new(),
                    },
                    _ => rest.iter().filter_map(|s| int(s)).collect(),
                };
                for g in grids {
                    if let Some(&n) = b.node_at.get(&g.to_string()) {
                        b.hold(n, held.iter().copied());
                    }
                }
            }
            _ => {}
        }
    }

    // loads: each load set a case
    for (card, fields) in &all {
        let Some(sid) = int(f(fields, 0)) else { continue };
        let case_name = format!("Load set {sid}");
        match card.as_str() {
            "FORCE" | "MOMENT" => {
                let Some(&n) = int(f(fields, 1)).and_then(|g| b.node_at.get(&g.to_string())) else { continue };
                if int(f(fields, 2)).is_some_and(|c| c != 0) {
                    let note = "loads in a coordinate system of their own: taken as in the basic one".to_string();
                    if !notes.contains(&note) {
                        notes.push(note);
                    }
                }
                let scale = real(f(fields, 3)).unwrap_or(0.0);
                let v = [4, 5, 6].map(|k| real(f(fields, k)).unwrap_or(0.0) * scale);
                let c = b.case(&case_name);
                let (force, moment) = if card == "FORCE" { (v, [0.0; 3]) } else { ([0.0; 3], v) };
                b.analysis.actions.push(Action { case: c, load: Load::Node { node: n, force, moment } });
            }
            "PLOAD4" | "PLOAD2" => {
                let Some(p) = real(f(fields, if card == "PLOAD2" { 1 } else { 2 })) else { continue };
                let elements: Vec<i64> = if card == "PLOAD2" { fields.iter().skip(2).filter_map(|s| int(s)).collect() } else { int(f(fields, 1)).into_iter().collect() };
                let c = b.case(&case_name);
                for e in elements {
                    if let Some(Placed::Plate(plate)) = b.element_at.get(&e.to_string()).copied() {
                        // a positive pressure acts along the plate's normal
                        b.analysis.actions.push(Action { case: c, load: Load::Plate { plate, axes: Axes::Local, dir: [0.0, 0.0, 1.0], q: p } });
                    }
                }
            }
            "GRAV" => {
                let a = real(f(fields, 2)).unwrap_or(0.0);
                let n = [3, 4, 5].map(|k| real(f(fields, k)).unwrap_or(0.0));
                let c = b.case(&case_name);
                if n[2] < 0.0 && n[0] == 0.0 && n[1] == 0.0 {
                    b.analysis.cases[c].self_weight = Some(a * -n[2] / 9.806_65);
                } else {
                    notes.push(format!("GRAV {sid}: not along −z, left out"));
                }
            }
            _ => {}
        }
    }
    // combinations of load sets
    for (card, fields) in &all {
        if card != "LOAD" {
            continue;
        }
        let Some(sid) = int(f(fields, 0)) else { continue };
        let overall = real(f(fields, 1)).unwrap_or(1.0);
        let mut terms = Vec::new();
        let rest: Vec<&String> = fields.iter().skip(2).collect();
        for pair in rest.chunks(2) {
            let (Some(s), Some(l)) = (pair.first().and_then(|s| real(s)), pair.get(1).and_then(|l| int(l))) else { continue };
            let c = b.case(&format!("Load set {l}"));
            terms.push((Term::Case(c), overall * s));
        }
        b.analysis.combinations.push(Combination { name: format!("LOAD {sid}"), kind: String::new(), terms });
    }
    notes.push("the file has no units: read as SI (metres, newtons)".into());
    b.finish("FEM bulk data (.bdf)", &crate::stem(name), notes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_side(path: &str) -> Result<Vec<u8>, String> {
        Err(format!("{path}: not there"))
    }

    #[test]
    fn numbers_without_their_e_are_read() {
        assert_eq!((real("1.5-3"), real(".5+2"), real("2.1E11"), real("7.85D3"), real("-4."), real("")), (Some(1.5e-3), Some(50.0), Some(2.1e11), Some(7850.0), Some(-4.0), None));
    }

    const DECK: &str = "SOL 101
CEND
SUBCASE 1
  LOAD = 1
BEGIN BULK
$ a frame of a bar, a beam and a plate, in three field formats
GRID           1       0      0.      0.      0.
GRID           2       0      0.      0.      3.
GRID,3,0,4.,0.,3.
GRID*                  4               0              4.              0.
*                     0.
GRID           5       0      0.      2.      3.
GRID           6       0      4.      2.      3.
CBAR         101       1       1       2      1.      0.      0.
CBEAM        102       2       2       3       5
CROD         103       3       3       4
CQUAD4       201      11       2       3       6       5
PBARL          1      10            BAR
             0.2     0.3
PBEAM          2      10   0.012   1.e-4   2.e-5      0.   3.e-6
PROD           3      10    1.-3
PSHELL        11      10    0.25      10
MAT1,10,2.1+11,,0.3,7850.
SPC1           1  123456       1       4
FORCE          1       3       0   1000.      1.      0.      0.
PLOAD4         1     201   2500.
GRAV           2       0    9.81      0.      0.     -1.
LOAD          10      1.     1.5       1     1.2       2
ENDDATA
";

    #[test]
    fn bars_beams_rods_and_plates_become_an_analysis_model() {
        let doc = read("frame.bdf", DECK.as_bytes(), &no_side).expect("bulk data");
        let a = doc.analysis.expect("analysis");
        assert_eq!((a.nodes.len(), a.members.len(), a.plates.len()), (6, 3, 1));
        assert_eq!(a.nodes.iter().find(|n| n.name == "4").unwrap().p, [4.0, 0.0, 0.0]);
        let bar = &a.members[0];
        assert_eq!(a.sections[bar.section.unwrap()].shape, Shape::Rectangle { b: 0.2, h: 0.3 });
        assert_eq!(bar.orientation, Some([1.0, 0.0, 0.0]));
        // the beam points its y axis to grid 5
        assert_eq!(a.members[1].orientation, Some([0.0, 2.0, 0.0]));
        assert_eq!(a.sections[a.members[1].section.unwrap()].props.area, Some(0.012));
        assert_eq!(a.members[2].kind, MemberKind::Truss);
        assert_eq!(a.sections[a.members[2].section.unwrap()].props.area, Some(1e-3));
        assert_eq!(a.plates[0].thickness, Some(0.25));
        assert_eq!(a.materials[0].e, Some(2.1e11));
        assert_eq!(a.nodes.iter().filter(|n| n.support == Some([Dof::Fixed; 6])).count(), 2);
        let names: Vec<&str> = a.cases.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["Load set 1", "Load set 2"]);
        assert!((a.cases[1].self_weight.unwrap() - 1.0).abs() < 0.01);
        assert_eq!(a.actions.len(), 2);
        assert_eq!(a.combinations[0].terms, [(Term::Case(0), 1.5), (Term::Case(1), 1.2)]);
    }

    #[test]
    fn solids_become_a_mesh_by_their_property() {
        let deck = "GRID,1,,0.,0.,0.\nGRID,2,,1.,0.,0.\nGRID,3,,0.,1.,0.\nGRID,4,,0.,0.,1.\nCTETRA,1,7,1,2,3,4\nPSOLID,7,1\n";
        let doc = read("tet.nas", deck.as_bytes(), &no_side).expect("bulk data");
        let mesh = doc.mesh.expect("mesh");
        assert_eq!(mesh.groups, ["PSOLID 7"]);
        assert_eq!(mesh.cells.kind, [CellKind::Tetra]);
        assert!(doc.analysis.is_none());
    }
}
