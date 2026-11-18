//! The structural analysis text model (`.s2k`, `.$2k`): tables, each a line
//! `TABLE:  "JOINT COORDINATES"` and records of `Name=value` pairs under it (a value with spaces
//! in quotes, a record going on on the next line after ` _`), up to `END TABLE DATA`.
//!
//! What is read:
//!
//! - `PROGRAM CONTROL`: the program and its units (`CurrUnits="KN, m, C"`).
//! - `JOINT COORDINATES`; `CONNECTIVITY - FRAME`, `- AREA` and `- SOLID`.
//! - `FRAME SECTION ASSIGNMENTS`, `FRAME SECTION PROPERTIES 01 - GENERAL` (rectangle, circle,
//!   pipe, box, I, channel, tee, angle; area and moments), `FRAME LOCAL AXES ASSIGNMENTS`,
//!   `FRAME RELEASE ASSIGNMENTS`.
//! - `AREA SECTION ASSIGNMENTS`, `AREA SECTION PROPERTIES` (thickness, shell or membrane).
//! - `MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES` (E, G, ν, weight, α).
//! - `JOINT RESTRAINT ASSIGNMENTS`, `JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED`.
//! - `LOAD PATTERN DEFINITIONS`, `JOINT LOADS - FORCE`, `FRAME LOADS - DISTRIBUTED`,
//!   `FRAME LOADS - POINT`, `AREA LOADS - UNIFORM`, `COMBINATION DEFINITIONS` (whose load cases
//!   name their patterns through `CASE - STATIC 1 - LOAD ASSIGNMENTS`).
//!
//! Everything is turned into SI units.

use crate::build::{Builder, Placed, Role};
use fem_model::*;
use std::collections::HashMap;

/// A record: its names (in capitals) and values.
type Record = HashMap<String, String>;

/// The records of each table, by the table's name in capitals.
fn tables(text: &str) -> HashMap<String, Vec<Record>> {
    let mut out: HashMap<String, Vec<Record>> = HashMap::new();
    let mut table: Option<String> = None;
    let mut record = String::new();
    let flush = |table: &Option<String>, record: &mut String, out: &mut HashMap<String, Vec<Record>>| {
        if let (Some(t), false) = (table, record.trim().is_empty()) {
            out.entry(t.clone()).or_default().push(pairs(record));
        }
        record.clear();
    };
    for line in text.lines() {
        let line = line.trim_end_matches('\r');
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('$') || trimmed.starts_with(';') {
            continue;
        }
        let upper = trimmed.to_ascii_uppercase();
        if let Some(rest) = upper.strip_prefix("TABLE:") {
            flush(&table, &mut record, &mut out);
            table = Some(rest.trim().trim_matches('"').trim().to_string());
            continue;
        }
        if upper.starts_with("END TABLE DATA") {
            flush(&table, &mut record, &mut out);
            break;
        }
        match trimmed.strip_suffix(" _").or_else(|| trimmed.strip_suffix("_").filter(|r| r.ends_with(' '))) {
            Some(open) => {
                record.push_str(open);
                record.push(' ');
            }
            None => {
                record.push_str(trimmed);
                flush(&table, &mut record, &mut out);
            }
        }
    }
    flush(&table, &mut record, &mut out);
    out
}

/// The `Name=value` pairs of a record.
fn pairs(record: &str) -> Record {
    let mut out = Record::new();
    let mut chars = record.chars().peekable();
    loop {
        while chars.peek().is_some_and(|c| c.is_whitespace()) {
            chars.next();
        }
        let name: String = chars.by_ref().take_while(|&c| c != '=').collect();
        if name.trim().is_empty() {
            break;
        }
        let value = if chars.peek() == Some(&'"') {
            chars.next();
            chars.by_ref().take_while(|&c| c != '"').collect()
        } else {
            let mut v = String::new();
            while let Some(&c) = chars.peek().filter(|c| !c.is_whitespace()) {
                v.push(c);
                chars.next();
            }
            v
        };
        out.insert(name.trim().to_ascii_uppercase(), value);
    }
    out
}

fn force_unit(u: &str) -> Option<f64> {
    Some(match u.trim().to_ascii_uppercase().as_str() {
        "N" => 1.0,
        "KN" => 1e3,
        "MN" => 1e6,
        "KGF" => 9.806_65,
        "TONF" => 9_806.65,
        "LB" => 4.448_221_615_260_5,
        "KIP" => 4_448.221_615_260_5,
        _ => return None,
    })
}

fn length_unit(u: &str) -> Option<f64> {
    Some(match u.trim().to_ascii_uppercase().as_str() {
        "MM" => 1e-3,
        "CM" => 1e-2,
        "M" => 1.0,
        "IN" => 0.0254,
        "FT" => 0.3048,
        _ => return None,
    })
}

pub fn read(name: &str, data: &[u8]) -> Result<Document, String> {
    let text = String::from_utf8_lossy(data);
    let t = tables(&text);
    let rows = |name: &str| t.get(name).map(Vec::as_slice).unwrap_or(&[]);
    let joints = rows("JOINT COORDINATES");
    if joints.is_empty() {
        return Err("no table of joint coordinates: not a model of this kind".into());
    }
    let mut notes = Vec::new();
    let control = rows("PROGRAM CONTROL").first();
    let units = control.and_then(|c| c.get("CURRUNITS")).cloned().unwrap_or_else(|| "KN, m, C".into());
    let mut u = units.split(',');
    let (force, length) = match (u.next().and_then(force_unit), u.next().and_then(length_unit)) {
        (Some(f), Some(l)) => (f, l),
        _ => {
            notes.push(format!("units {units} not known: read as kN and m"));
            (1e3, 1.0)
        }
    };
    let value = |r: &Record, k: &str| r.get(k).and_then(|v| v.trim().parse::<f64>().ok()).filter(|v| v.is_finite());
    let text_of = |r: &Record, k: &str| r.get(k).cloned().unwrap_or_default();
    let yes = |r: &Record, k: &str| r.get(k).is_some_and(|v| v.eq_ignore_ascii_case("yes"));

    let mut b = Builder::default();
    b.analysis.application = control.and_then(|c| c.get("PROGRAMNAME")).map(|p| format!("{p} {}", control.and_then(|c| c.get("VERSION")).cloned().unwrap_or_default()).trim().to_string());
    for j in joints {
        let id = text_of(j, "JOINT");
        // the global coordinates where the file gives them, else the given ones (cartesian)
        let p = match (value(j, "GLOBALX"), value(j, "GLOBALY"), value(j, "GLOBALZ")) {
            (Some(x), Some(y), Some(z)) => [x, y, z],
            _ => {
                if j.get("COORDTYPE").is_some_and(|c| !c.eq_ignore_ascii_case("cartesian")) {
                    notes.push(format!("joint {id}: not cartesian, read as if it were"));
                }
                [value(j, "XORR").unwrap_or(0.0), value(j, "Y").unwrap_or(0.0), value(j, "Z").unwrap_or(0.0)]
            }
        };
        b.node(&id, p.map(|v| v * length));
    }

    // materials and sections
    let mut materials: HashMap<String, usize> = HashMap::new();
    let pressure = force / (length * length);
    for m in rows("MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES") {
        let name = text_of(m, "MATERIAL");
        let density = value(m, "UNITMASS").map(|v| v * force / length.powi(4)).filter(|v| *v > 0.0).or_else(|| value(m, "UNITWEIGHT").map(|w| w * force / length.powi(3) / 9.806_65));
        b.analysis.materials.push(Material { name: name.clone(), e: value(m, "E1").map(|e| e * pressure), g: value(m, "G12").map(|g| g * pressure), nu: value(m, "U12"), density, alpha: value(m, "A1") });
        materials.insert(name, b.analysis.materials.len() - 1);
    }
    let mut sections: HashMap<String, usize> = HashMap::new();
    for s in rows("FRAME SECTION PROPERTIES 01 - GENERAL") {
        let name = text_of(s, "SECTIONNAME");
        let d = |k: &str| value(s, k).map(|v| v * length).unwrap_or(0.0);
        let (h, w, tf, tw) = (d("T3"), d("T2"), d("TF"), d("TW"));
        let shape = match text_of(s, "SHAPE").to_ascii_lowercase().as_str() {
            k if k.contains("rectangular") => Shape::Rectangle { b: w, h },
            k if k.contains("circle") => Shape::Circle { d: h },
            k if k.contains("pipe") => Shape::Tube { d: h, t: tw },
            k if k.contains("box") || k.contains("tube") => Shape::Box { b: w, h, t: tf.max(tw) },
            k if k.contains("i/wide") || k.contains("wide flange") => Shape::I { h, b: w.max(d("T2B")), tw, tf: tf.max(d("TFB")) },
            k if k.contains("channel") => Shape::Channel { h, b: w, tw, tf },
            k if k.contains("tee") => Shape::Tee { h, b: w, tw, tf },
            k if k.contains("angle") => Shape::Angle { h, b: w, t: tf.max(tw) },
            _ => Shape::Unknown,
        };
        let props = SectionProps { area: value(s, "AREA").map(|v| v * length.powi(2)), iy: value(s, "I33").map(|v| v * length.powi(4)), iz: value(s, "I22").map(|v| v * length.powi(4)), it: value(s, "TORSCONST").map(|v| v * length.powi(4)) };
        let material = s.get("MATERIAL").and_then(|m| materials.get(m).copied());
        b.analysis.sections.push(Section { name: name.clone(), shape, props, material });
        sections.insert(name, b.analysis.sections.len() - 1);
    }
    let section_of: HashMap<String, usize> = rows("FRAME SECTION ASSIGNMENTS").iter().filter_map(|r| Some((text_of(r, "FRAME"), *sections.get(r.get("ANALSECT").or_else(|| r.get("SECTIONNAME"))?)?))).collect();
    let mut plate_props: HashMap<String, (Option<f64>, Option<usize>, PlateKind)> = HashMap::new();
    for s in rows("AREA SECTION PROPERTIES") {
        let thickness = value(s, "THICKNESS").or_else(|| value(s, "THICK")).or_else(|| value(s, "MEMTHICK")).map(|t| t * length);
        let kind = match text_of(s, "TYPE").to_ascii_lowercase() {
            k if k.contains("membrane") => PlateKind::Membrane,
            _ => PlateKind::Shell,
        };
        plate_props.insert(text_of(s, "SECTION"), (thickness, s.get("MATERIAL").and_then(|m| materials.get(m).copied()), kind));
    }
    let area_section: HashMap<String, String> = rows("AREA SECTION ASSIGNMENTS").iter().map(|r| (text_of(r, "AREA"), text_of(r, "SECTION"))).collect();

    // frames, areas, solids
    let node = |b: &Builder, r: &Record, k: &str| r.get(k).and_then(|j| b.node_at.get(j).copied());
    for f in rows("CONNECTIVITY - FRAME") {
        let name = text_of(f, "FRAME");
        let (Some(i), Some(j)) = (node(&b, f, "JOINTI"), node(&b, f, "JOINTJ")) else { continue };
        let section = section_of.get(&name).copied();
        let material = section.and_then(|s| b.analysis.sections[s].material);
        let (pi, pj) = (b.analysis.nodes[i].p, b.analysis.nodes[j].p);
        let vertical = (pi[0] - pj[0]).abs() < 1e-9 && (pi[1] - pj[1]).abs() < 1e-9;
        b.element(&format!("frame {name}"), CellKind::Line, vec![i, j], Role::Member(if vertical { MemberKind::Column } else { MemberKind::Beam }), "", section, material, None);
    }
    for a in rows("CONNECTIVITY - AREA") {
        let name = text_of(a, "AREA");
        let n = value(a, "NUMJOINTS").unwrap_or(0.0) as usize;
        let Some(nodes) = (1..=n.max(3)).map(|k| node(&b, a, &format!("JOINT{k}"))).take(n.max(3)).collect::<Option<Vec<usize>>>() else { continue };
        let (thickness, material, kind) = area_section.get(&name).and_then(|s| plate_props.get(s)).copied().unwrap_or((None, None, PlateKind::Shell));
        // a plate in a horizontal plane is a floor, one in a vertical plane a wall
        let zs: Vec<f64> = nodes.iter().map(|&n| b.analysis.nodes[n].p[2]).collect();
        let flat = zs.iter().all(|z| (z - zs[0]).abs() < 1e-9);
        let kind = match kind {
            PlateKind::Shell if flat => PlateKind::Floor,
            other => other,
        };
        b.element(&format!("area {name}"), if n == 3 { CellKind::Triangle } else if n == 4 { CellKind::Quad } else { CellKind::Polygon }, nodes, Role::Plate(kind), "", None, material, thickness);
    }
    for s in rows("CONNECTIVITY - SOLID") {
        let name = text_of(s, "SOLID");
        let Some(nodes) = (1..=8).map(|k| node(&b, s, &format!("JOINT{k}"))).collect::<Option<Vec<usize>>>() else { continue };
        // the joints go round the bottom and then the top in pairs: 1 2 3 4 are (x, y) corners
        let hexa = vec![nodes[0], nodes[1], nodes[3], nodes[2], nodes[4], nodes[5], nodes[7], nodes[6]];
        b.element(&format!("solid {name}"), CellKind::Hexa, hexa, Role::Cell, "solids", None, None, None);
    }
    // local axes and releases of the frames
    let member_of = |b: &Builder, r: &Record| match b.element_at.get(&format!("frame {}", text_of(r, "FRAME"))) {
        Some(Placed::Member(m)) => Some(*m),
        _ => None,
    };
    for r in rows("FRAME LOCAL AXES ASSIGNMENTS 1 - TYPICAL") {
        if let (Some(m), Some(angle)) = (member_of(&b, r), value(r, "ANGLE")) {
            b.analysis.members[m].angle = angle.to_radians();
        }
    }
    for r in rows("FRAME RELEASE ASSIGNMENTS 1 - GENERAL") {
        let Some(m) = member_of(&b, r) else { continue };
        for (end, suffix) in ["I", "J"].iter().enumerate() {
            for (dof, key) in ["P", "V2", "V3", "T", "M2", "M3"].iter().enumerate() {
                b.analysis.members[m].releases[end][dof] = yes(r, &format!("{key}{suffix}"));
            }
        }
    }

    // supports
    for r in rows("JOINT RESTRAINT ASSIGNMENTS") {
        let Some(n) = node(&b, r, "JOINT") else { continue };
        let held: Vec<usize> = ["U1", "U2", "U3", "R1", "R2", "R3"].iter().enumerate().filter(|(_, k)| yes(r, k)).map(|(d, _)| d).collect();
        if !held.is_empty() {
            b.hold(n, held);
        }
    }
    for r in rows("JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED") {
        let Some(n) = node(&b, r, "JOINT") else { continue };
        let support = b.analysis.nodes[n].support.get_or_insert([Dof::Free; 6]);
        for (d, k) in ["U1", "U2", "U3", "R1", "R2", "R3"].iter().enumerate() {
            if let Some(s) = value(r, k).filter(|s| *s > 0.0) {
                support[d] = Dof::Spring(s * if d < 3 { force / length } else { force * length });
            }
        }
    }

    // load patterns are the load cases
    for p in rows("LOAD PATTERN DEFINITIONS") {
        let name = text_of(p, "LOADPAT");
        let c = b.case(&name);
        let kind = text_of(p, "DESIGNTYPE");
        b.analysis.cases[c].kind = match kind.to_ascii_lowercase().as_str() {
            "dead" | "super dead" | "superdead" => CaseKind::Permanent,
            "live" | "roof live" | "reducible live" | "snow" | "wind" | "temperature" => CaseKind::Variable,
            "quake" | "seismic" => CaseKind::Accidental,
            "" => CaseKind::Unknown,
            _ => CaseKind::Other(kind.clone()),
        };
        b.analysis.cases[c].self_weight = value(p, "SELFWTMULT").filter(|f| *f != 0.0);
    }
    let pattern = |b: &mut Builder, r: &Record| b.case(&text_of(r, "LOADPAT"));
    let direction = |r: &Record| -> ([f64; 3], Axes) {
        match text_of(r, "DIR").to_ascii_uppercase().as_str() {
            "X" => ([1.0, 0.0, 0.0], Axes::Global),
            "Y" => ([0.0, 1.0, 0.0], Axes::Global),
            "Z" => ([0.0, 0.0, 1.0], Axes::Global),
            "1" => ([1.0, 0.0, 0.0], Axes::Local),
            "2" => ([0.0, 1.0, 0.0], Axes::Local),
            "3" => ([0.0, 0.0, 1.0], Axes::Local),
            "X PROJECTED" | "Y PROJECTED" | "Z PROJECTED" | "GRAVITY PROJECTED" => ([0.0, 0.0, -1.0], Axes::Global),
            _ => ([0.0, 0.0, -1.0], Axes::Global),
        }
    };
    for r in rows("JOINT LOADS - FORCE") {
        let Some(n) = node(&b, r, "JOINT") else { continue };
        let c = pattern(&mut b, r);
        let v = |a: &str, z: &str, y: &str, unit: f64| [value(r, a).unwrap_or(0.0) * unit, value(r, z).unwrap_or(0.0) * unit, value(r, y).unwrap_or(0.0) * unit];
        b.analysis.actions.push(Action { case: c, load: Load::Node { node: n, force: v("F1", "F2", "F3", force), moment: v("M1", "M2", "M3", force * length) } });
    }
    let mut skipped: HashMap<String, usize> = HashMap::new();
    for r in rows("FRAME LOADS - DISTRIBUTED") {
        let Some(m) = member_of(&b, r) else { continue };
        if text_of(r, "TYPE").eq_ignore_ascii_case("moment") {
            *skipped.entry("distributed moment on a frame".into()).or_default() += 1;
            continue;
        }
        let c = pattern(&mut b, r);
        let (dir, axes) = direction(r);
        let (from, to) = (value(r, "RELDISTA").unwrap_or(0.0), value(r, "RELDISTB").unwrap_or(1.0));
        let q = [value(r, "FOVERLA").unwrap_or(0.0) * force / length, value(r, "FOVERLB").unwrap_or(0.0) * force / length];
        b.analysis.actions.push(Action { case: c, load: Load::MemberLine { member: m, axes, dir, q, from, to } });
    }
    for r in rows("FRAME LOADS - POINT") {
        let Some(m) = member_of(&b, r) else { continue };
        if text_of(r, "TYPE").eq_ignore_ascii_case("moment") {
            *skipped.entry("point moment on a frame".into()).or_default() += 1;
            continue;
        }
        let c = pattern(&mut b, r);
        let (dir, axes) = direction(r);
        let f = value(r, "FORCE").unwrap_or(0.0) * force;
        b.analysis.actions.push(Action { case: c, load: Load::MemberPoint { member: m, axes, at: value(r, "RELDIST").unwrap_or(0.5), force: dir.map(|d| d * f) } });
    }
    for r in rows("AREA LOADS - UNIFORM") {
        let Some(Placed::Plate(plate)) = b.element_at.get(&format!("area {}", text_of(r, "AREA"))).copied() else { continue };
        let c = pattern(&mut b, r);
        let (dir, axes) = direction(r);
        b.analysis.actions.push(Action { case: c, load: Load::Plate { plate, axes, dir, q: value(r, "UNIFLOAD").unwrap_or(0.0) * pressure } });
    }
    for (what, n) in skipped {
        notes.push(format!("{n} × {what}: not read"));
    }

    // combinations of load cases; a load case of one pattern goes by that pattern
    let mut patterns_of: HashMap<String, Vec<String>> = HashMap::new();
    for r in rows("CASE - STATIC 1 - LOAD ASSIGNMENTS") {
        if text_of(r, "LOADTYPE").eq_ignore_ascii_case("load pattern") {
            patterns_of.entry(text_of(r, "CASE")).or_default().push(text_of(r, "LOADNAME"));
        }
    }
    let mut combination_at: HashMap<String, usize> = HashMap::new();
    for r in rows("COMBINATION DEFINITIONS") {
        let name = text_of(r, "COMBONAME");
        let k = *combination_at.entry(name.clone()).or_insert_with(|| {
            b.analysis.combinations.push(Combination { name: name.clone(), kind: String::new(), terms: Vec::new() });
            b.analysis.combinations.len() - 1
        });
        if let Some(kind) = r.get("COMBOTYPE") {
            b.analysis.combinations[k].kind = kind.clone();
        }
        let case = text_of(r, "CASENAME");
        let factor = value(r, "SCALEFACTOR").unwrap_or(1.0);
        let term = match (combination_at.get(&case), patterns_of.get(&case).map(Vec::as_slice)) {
            (Some(&other), _) if other != k => Term::Combination(other),
            (_, Some([one])) => Term::Case(b.case(one)),
            _ => Term::Case(b.case(&case)),
        };
        b.analysis.combinations[k].terms.push((term, factor));
    }
    b.finish("Structural analysis text model (.s2k)", &crate::stem(name), notes)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODEL: &str = r#"File portal.s2k was saved on 1/1/26

TABLE:  "PROGRAM CONTROL"
   ProgramName="Analysis"   Version=1.0   CurrUnits="KN, m, C"

TABLE:  "JOINT COORDINATES"
   Joint=1   CoordSys=GLOBAL   CoordType=Cartesian   XorR=0   Y=0   Z=0
   Joint=2   CoordSys=GLOBAL   CoordType=Cartesian   XorR=0   Y=0   Z=3
   Joint=3   CoordSys=GLOBAL   CoordType=Cartesian   XorR=4   Y=0   Z=3
   Joint=4   CoordSys=GLOBAL   CoordType=Cartesian   XorR=4   Y=0   Z=0   _
      SpecialJt=No
   Joint=5   CoordSys=GLOBAL   CoordType=Cartesian   XorR=0   Y=2   Z=3
   Joint=6   CoordSys=GLOBAL   CoordType=Cartesian   XorR=4   Y=2   Z=3

TABLE:  "CONNECTIVITY - FRAME"
   Frame=1   JointI=1   JointJ=2
   Frame=2   JointI=2   JointJ=3
   Frame=3   JointI=4   JointJ=3

TABLE:  "CONNECTIVITY - AREA"
   Area=1   NumJoints=4   Joint1=2   Joint2=3   Joint3=6   Joint4=5

TABLE:  "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES"
   Material=C30   UnitWeight=25   UnitMass=2.5   E1=33000000   G12=13750000   U12=0.2   A1=0.00001

TABLE:  "FRAME SECTION PROPERTIES 01 - GENERAL"
   SectionName=COL   Material=C30   Shape=Rectangular   t3=0.4   t2=0.3   Area=0.12   I33=0.0016   I22=0.0009
   SectionName="HE 200"   Material=C30   Shape="I/Wide Flange"   t3=0.19   t2=0.2   tf=0.01   tw=0.0065

TABLE:  "FRAME SECTION ASSIGNMENTS"
   Frame=1   SectionType=Rectangular   AnalSect=COL
   Frame=2   SectionType="I/Wide Flange"   AnalSect="HE 200"
   Frame=3   SectionType=Rectangular   AnalSect=COL

TABLE:  "FRAME RELEASE ASSIGNMENTS 1 - GENERAL"
   Frame=2   PI=No   V2I=No   V3I=No   TI=No   M2I=Yes   M3I=Yes   PJ=No   V2J=No   V3J=No   TJ=No   M2J=No   M3J=No

TABLE:  "AREA SECTION PROPERTIES"
   Section=SLAB   Material=C30   AreaType=Shell   Type=Shell-Thin   Thickness=0.2

TABLE:  "AREA SECTION ASSIGNMENTS"
   Area=1   Section=SLAB

TABLE:  "JOINT RESTRAINT ASSIGNMENTS"
   Joint=1   U1=Yes   U2=Yes   U3=Yes   R1=Yes   R2=Yes   R3=Yes
   Joint=4   U1=Yes   U2=Yes   U3=Yes   R1=No   R2=No   R3=No

TABLE:  "LOAD PATTERN DEFINITIONS"
   LoadPat=DEAD   DesignType=Dead   SelfWtMult=1
   LoadPat=LIVE   DesignType=Live   SelfWtMult=0

TABLE:  "JOINT LOADS - FORCE"
   Joint=3   LoadPat=LIVE   CoordSys=GLOBAL   F1=10   F2=0   F3=0   M1=0   M2=0   M3=0

TABLE:  "FRAME LOADS - DISTRIBUTED"
   Frame=2   LoadPat=DEAD   CoordSys=GLOBAL   Type=Force   Dir=Gravity   DistType=RelDist   RelDistA=0   RelDistB=1   FOverLA=5   FOverLB=5

TABLE:  "AREA LOADS - UNIFORM"
   Area=1   LoadPat=LIVE   CoordSys=GLOBAL   Dir=Gravity   UnifLoad=2.5

TABLE:  "COMBINATION DEFINITIONS"
   ComboName=ULS   ComboType="Linear Add"   CaseName=DEAD   ScaleFactor=1.35
   ComboName=ULS   CaseName=LIVE   ScaleFactor=1.5

END TABLE DATA
"#;

    #[test]
    fn reads_the_tables_into_an_analysis_model_in_si_units() {
        let doc = read("portal.s2k", MODEL.as_bytes()).expect("model");
        let a = doc.analysis.expect("analysis");
        assert_eq!((a.nodes.len(), a.members.len(), a.plates.len()), (6, 3, 1));
        assert_eq!(a.members[0].kind, MemberKind::Column);
        assert_eq!(a.members[1].kind, MemberKind::Beam);
        assert_eq!(a.sections[a.members[0].section.unwrap()].shape, Shape::Rectangle { b: 0.3, h: 0.4 });
        assert_eq!(a.sections[a.members[1].section.unwrap()].shape, Shape::I { h: 0.19, b: 0.2, tw: 0.0065, tf: 0.01 });
        assert_eq!(a.members[1].releases[0], [false, false, false, false, true, true]);
        assert_eq!((a.plates[0].thickness, a.plates[0].kind), (Some(0.2), PlateKind::Floor));
        assert_eq!(a.materials[0].e, Some(33e9));
        assert_eq!(a.nodes.iter().filter(|n| n.support.is_some()).count(), 2);
        assert_eq!(a.cases.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["DEAD", "LIVE"]);
        assert_eq!(a.cases[0].self_weight, Some(1.0));
        assert!(a.actions.iter().any(|x| matches!(x.load, Load::Node { force, .. } if force == [10e3, 0.0, 0.0])));
        assert!(a.actions.iter().any(|x| matches!(x.load, Load::MemberLine { q, dir, .. } if q == [5e3, 5e3] && dir == [0.0, 0.0, -1.0])));
        assert!(a.actions.iter().any(|x| matches!(x.load, Load::Plate { q, .. } if q == 2.5e3)));
        assert_eq!(a.combinations[0].kind, "Linear Add");
        assert_eq!(a.combinations[0].terms, [(Term::Case(0), 1.35), (Term::Case(1), 1.5)]);
        assert_eq!(a.application.as_deref(), Some("Analysis 1.0"));
    }

    #[test]
    fn a_record_goes_on_after_an_underscore_and_quoted_values_keep_their_spaces() {
        let t = tables("TABLE:  \"A\"\n   Name=\"x y\"   B=1 _\n   C=2\nEND TABLE DATA\n");
        let r = &t["A"][0];
        assert_eq!((r["NAME"].as_str(), r["B"].as_str(), r["C"].as_str()), ("x y", "1", "2"));
    }
}
