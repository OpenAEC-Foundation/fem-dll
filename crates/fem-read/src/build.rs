//! What the readers of element files share: every element becomes a member, a plate or a cell
//! of the mesh, and at the end the nodes of the analysis model are the ones its members, plates,
//! supports and loads use.

use fem_model::*;
use std::collections::HashMap;

/// What an element of a file is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Role {
    Member(MemberKind),
    Plate(PlateKind),
    /// A cell of the mesh: a solid, a plane element, a point.
    Cell,
}

/// Where an element went.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Placed {
    Member(usize),
    Plate(usize),
    Cell(usize),
    None,
}

#[derive(Default)]
pub(crate) struct Builder {
    pub analysis: Analysis,
    pub mesh: Mesh,
    /// Node by the file's number or name.
    pub node_at: HashMap<String, usize>,
    /// Element by the file's number or name.
    pub element_at: HashMap<String, Placed>,
    group_at: HashMap<String, u32>,
    case_at: HashMap<String, usize>,
}

impl Builder {
    pub fn node(&mut self, name: &str, p: [f64; 3]) -> usize {
        match self.node_at.get(name) {
            Some(&k) => {
                self.analysis.nodes[k].p = p;
                k
            }
            None => {
                self.analysis.nodes.push(Node { name: name.to_string(), p, support: None });
                self.node_at.insert(name.to_string(), self.analysis.nodes.len() - 1);
                self.analysis.nodes.len() - 1
            }
        }
    }

    /// An element of `kind` with `nodes` (indices) as `role`, in the mesh group `group`.
    #[allow(clippy::too_many_arguments)]
    pub fn element(&mut self, name: &str, kind: CellKind, nodes: Vec<usize>, role: Role, group: &str, section: Option<usize>, material: Option<usize>, thickness: Option<f64>) -> Placed {
        let placed = match role {
            Role::Member(member) if nodes.len() >= 2 => {
                self.analysis.members.push(Member {
                    name: name.to_string(),
                    kind: member,
                    nodes: [nodes[0], nodes[nodes.len() - 1]],
                    section,
                    material,
                    angle: 0.0,
                    orientation: None,
                    releases: [[false; 6]; 2],
                });
                Placed::Member(self.analysis.members.len() - 1)
            }
            Role::Plate(plate) if nodes.len() >= 3 => {
                self.analysis.plates.push(Plate { name: name.to_string(), kind: plate, nodes, thickness, material });
                Placed::Plate(self.analysis.plates.len() - 1)
            }
            Role::Cell => {
                let next = self.group_at.len() as u32;
                let g = *self.group_at.entry(group.to_string()).or_insert(next);
                if g as usize == self.mesh.groups.len() {
                    self.mesh.groups.push(group.to_string());
                }
                let nodes: Vec<u32> = nodes.iter().map(|&n| n as u32).collect();
                self.mesh.cells.push(kind, &nodes, g);
                Placed::Cell(self.mesh.cells.len() - 1)
            }
            _ => Placed::None,
        };
        self.element_at.insert(name.to_string(), placed);
        placed
    }

    /// The load case named `name`, made when first named.
    pub fn case(&mut self, name: &str) -> usize {
        if let Some(&k) = self.case_at.get(name) {
            return k;
        }
        self.analysis.cases.push(LoadCase { name: name.to_string(), ..LoadCase::default() });
        self.case_at.insert(name.to_string(), self.analysis.cases.len() - 1);
        self.analysis.cases.len() - 1
    }

    /// Holds degrees of freedom `dofs` (0 to 5) of node `node`.
    pub fn hold(&mut self, node: usize, dofs: impl IntoIterator<Item = usize>) {
        let support = self.analysis.nodes[node].support.get_or_insert([Dof::Free; 6]);
        for d in dofs.into_iter().filter(|d| *d < 6) {
            support[d] = Dof::Fixed;
        }
    }

    /// The document: the analysis model with only the nodes it uses, the mesh of the cells.
    pub fn finish(mut self, format: &str, name: &str, mut notes: Vec<String>) -> Result<Document, String> {
        let mut analysis = std::mem::take(&mut self.analysis);
        let mut mesh = std::mem::take(&mut self.mesh);
        mesh.name = name.to_string();
        mesh.points = analysis.nodes.iter().map(|n| n.p).collect();
        // the nodes the analysis model uses
        let mut used = vec![false; analysis.nodes.len()];
        for m in &analysis.members {
            m.nodes.iter().for_each(|&n| used[n] = true);
        }
        for p in &analysis.plates {
            p.nodes.iter().for_each(|&n| used[n] = true);
        }
        for (k, n) in analysis.nodes.iter().enumerate() {
            used[k] |= n.support.is_some();
        }
        for a in &analysis.actions {
            if let Load::Node { node, .. } = a.load {
                used[node] = true;
            }
        }
        let mut new = vec![usize::MAX; analysis.nodes.len()];
        let mut nodes = Vec::new();
        for (k, n) in analysis.nodes.iter().enumerate() {
            if used[k] {
                new[k] = nodes.len();
                nodes.push(n.clone());
            }
        }
        analysis.nodes = nodes;
        for m in &mut analysis.members {
            m.nodes = m.nodes.map(|n| new[n]);
        }
        for p in &mut analysis.plates {
            p.nodes.iter_mut().for_each(|n| *n = new[*n]);
        }
        for a in &mut analysis.actions {
            if let Load::Node { node, .. } = &mut a.load {
                *node = new[*node];
            }
        }
        if analysis.name.is_empty() {
            analysis.name = name.to_string();
        }
        let has_mesh = !mesh.cells.is_empty();
        let has_analysis = !analysis.is_empty();
        if !has_mesh && !has_analysis {
            return Err("no elements".into());
        }
        notes.append(&mut analysis.notes);
        analysis.notes = notes.clone();
        Ok(Document { mesh: has_mesh.then_some(mesh), analysis: has_analysis.then_some(analysis), format: format.to_string(), notes })
    }
}

/// The corner nodes of an element of `kind` from the nodes the file gives (a quadratic element
/// has its corners first).
pub(crate) fn corners(kind: CellKind, nodes: &[usize]) -> Option<Vec<usize>> {
    match kind.corners() {
        Some(n) => nodes.get(..n).map(<[usize]>::to_vec),
        None => (nodes.len() >= 2).then(|| nodes.to_vec()),
    }
}
