use jolt_sys as sys;

/// The collision layer a body belongs to. Which layers collide is configured with [`LayerConfig`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ObjectLayer(pub u16);

/// A broad phase tree. Object layers are grouped into broad phase layers; a common split is
/// one layer for static geometry and one for everything that moves.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BroadPhaseLayer(pub u8);

/// Describes the object layers, which broad phase layer each one lives in, and which pairs collide.
///
/// ```
/// use jolt::{BroadPhaseLayer, LayerConfig};
///
/// let mut layers = LayerConfig::new();
/// let static_geometry = layers.add_object_layer(BroadPhaseLayer(0));
/// let moving = layers.add_object_layer(BroadPhaseLayer(1));
/// layers.enable_collision(static_geometry, moving);
/// layers.enable_collision(moving, moving);
/// ```
#[derive(Clone, Debug, Default)]
pub struct LayerConfig {
    pub(crate) object_to_broad_phase: Vec<u8>,
    pub(crate) colliding_pairs: Vec<sys::JPC_ObjectLayerPair>,
}

impl LayerConfig {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds an object layer living in `broad_phase_layer` and returns it. Layers are numbered from 0.
    pub fn add_object_layer(&mut self, broad_phase_layer: BroadPhaseLayer) -> ObjectLayer {
        let layer = u16::try_from(self.object_to_broad_phase.len()).expect("too many object layers");
        self.object_to_broad_phase.push(broad_phase_layer.0);
        ObjectLayer(layer)
    }

    /// Makes bodies in `a` and `b` collide with each other. Order does not matter.
    pub fn enable_collision(&mut self, a: ObjectLayer, b: ObjectLayer) -> &mut Self {
        self.colliding_pairs.push(sys::JPC_ObjectLayerPair { layer1: a.0, layer2: b.0 });
        self
    }

    pub fn num_object_layers(&self) -> u32 {
        self.object_to_broad_phase.len() as u32
    }

    pub fn num_broad_phase_layers(&self) -> u32 {
        self.object_to_broad_phase.iter().map(|&l| l as u32 + 1).max().unwrap_or(0)
    }
}
