//! A structural analysis model, in SI units (metres, newtons, pascals, kilograms) and the axes
//! of the file (most analysis files are Z up).
//!
//! Items refer to each other by their place in the lists of [`Analysis`]; their `name` is how
//! the file numbers or names them.

/// One degree of freedom of a support.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Dof {
    Free,
    Fixed,
    /// A spring, in N/m for a translation and N·m/rad for a rotation.
    Spring(f64),
}

/// A node.
#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    pub name: String,
    pub p: [f64; 3],
    /// Held in ux, uy, uz, rx, ry, rz of the global axes; `None` when not supported.
    pub support: Option<[Dof; 6]>,
}

/// What a member is for, where the file says.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MemberKind {
    #[default]
    Beam,
    Column,
    Brace,
    /// Carries axial force only (a rod, a truss bar).
    Truss,
    Cable,
}

/// A member: a bar between two nodes.
#[derive(Clone, Debug, PartialEq)]
pub struct Member {
    pub name: String,
    pub kind: MemberKind,
    /// Indices into [`Analysis::nodes`], start and end.
    pub nodes: [usize; 2],
    /// Index into [`Analysis::sections`].
    pub section: Option<usize>,
    /// Index into [`Analysis::materials`], when the member names one besides its section's.
    pub material: Option<usize>,
    /// The turn of the section about the member's axis, radians.
    pub angle: f64,
    /// A point the member's local y axis points to, where the file gives one (global axes).
    pub orientation: Option<[f64; 3]>,
    /// Released (true) in ux uy uz rx ry rz of the member's own axes, at the start and the end.
    pub releases: [[bool; 6]; 2],
}

/// What a plate is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PlateKind {
    /// Bending and membrane action (a shell).
    #[default]
    Shell,
    /// A floor or another horizontal plate.
    Floor,
    Wall,
    /// Membrane action only.
    Membrane,
}

/// A plate: a face of three or more nodes.
#[derive(Clone, Debug, PartialEq)]
pub struct Plate {
    pub name: String,
    pub kind: PlateKind,
    /// Indices into [`Analysis::nodes`], in order around the plate.
    pub nodes: Vec<usize>,
    /// Metres.
    pub thickness: Option<f64>,
    pub material: Option<usize>,
}

/// The shape of a cross-section, in metres. `h` is the depth along the section's local z,
/// `b` the width along its local y.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Rectangle { b: f64, h: f64 },
    Circle { d: f64 },
    /// A circular hollow section.
    Tube { d: f64, t: f64 },
    /// A rectangular hollow section.
    Box { b: f64, h: f64, t: f64 },
    I { h: f64, b: f64, tw: f64, tf: f64 },
    Channel { h: f64, b: f64, tw: f64, tf: f64 },
    Tee { h: f64, b: f64, tw: f64, tf: f64 },
    Angle { h: f64, b: f64, t: f64 },
    /// No shape the reader knows: only its name and properties.
    Unknown,
}

/// Properties a file gives of a section; m², m⁴.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct SectionProps {
    pub area: Option<f64>,
    /// About the local y axis (bending in the local z direction).
    pub iy: Option<f64>,
    pub iz: Option<f64>,
    pub it: Option<f64>,
}

/// A cross-section of members.
#[derive(Clone, Debug, PartialEq)]
pub struct Section {
    pub name: String,
    pub shape: Shape,
    pub props: SectionProps,
    pub material: Option<usize>,
}

/// A material.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Material {
    pub name: String,
    /// Pa.
    pub e: Option<f64>,
    pub g: Option<f64>,
    pub nu: Option<f64>,
    /// kg/m³.
    pub density: Option<f64>,
    /// 1/K.
    pub alpha: Option<f64>,
}

/// What a load case is, where the file says.
#[derive(Clone, Debug, PartialEq, Default)]
pub enum CaseKind {
    Permanent,
    Variable,
    Accidental,
    #[default]
    Unknown,
    /// As the file names it.
    Other(String),
}

/// A load case.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct LoadCase {
    pub name: String,
    pub kind: CaseKind,
    /// The factor on the weight of the structure itself, acting downward (−z).
    pub self_weight: Option<f64>,
}

/// A term of a combination.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Term {
    /// Index into [`Analysis::cases`].
    Case(usize),
    /// Index into [`Analysis::combinations`].
    Combination(usize),
}

/// A load combination.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Combination {
    pub name: String,
    /// As the file names its kind ("linear add", "envelope", ...), empty when it does not.
    pub kind: String,
    pub terms: Vec<(Term, f64)>,
}

/// The axes a load is given in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Axes {
    #[default]
    Global,
    /// The member's or plate's own axes.
    Local,
}

/// A load.
#[derive(Clone, Debug, PartialEq)]
pub enum Load {
    /// A force (N) and moment (N·m) on a node, global axes.
    Node { node: usize, force: [f64; 3], moment: [f64; 3] },
    /// A force on a member at `at` (0 at the start, 1 at the end), N.
    MemberPoint { member: usize, axes: Axes, at: f64, force: [f64; 3] },
    /// A force per length along a member, N/m: `dir` a unit vector, `q` the intensity at `from`
    /// and at `to` (parts of the length, 0 to 1).
    MemberLine { member: usize, axes: Axes, dir: [f64; 3], q: [f64; 2], from: f64, to: f64 },
    /// A force per area on a plate, N/m², `dir` a unit vector.
    Plate { plate: usize, axes: Axes, dir: [f64; 3], q: f64 },
}

/// A load of a load case.
#[derive(Clone, Debug, PartialEq)]
pub struct Action {
    /// Index into [`Analysis::cases`].
    pub case: usize,
    pub load: Load,
}

/// A structural analysis model.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Analysis {
    pub name: String,
    /// The program that wrote the file, where it says.
    pub application: Option<String>,
    pub nodes: Vec<Node>,
    pub members: Vec<Member>,
    pub plates: Vec<Plate>,
    pub sections: Vec<Section>,
    pub materials: Vec<Material>,
    pub cases: Vec<LoadCase>,
    pub combinations: Vec<Combination>,
    pub actions: Vec<Action>,
    /// What the reader left out or assumed.
    pub notes: Vec<String>,
}

impl Analysis {
    pub fn is_empty(&self) -> bool {
        self.members.is_empty() && self.plates.is_empty() && !self.nodes.iter().any(|n| n.support.is_some())
    }

    /// The support of a node from which of ux uy uz rx ry rz are held.
    pub fn held(held: [bool; 6]) -> Option<[Dof; 6]> {
        held.iter().any(|h| *h).then(|| held.map(|h| if h { Dof::Fixed } else { Dof::Free }))
    }
}
