use crate::atoms;
use crate::pos::VoxelPos;
use modular_bitfield::prelude::*;

#[derive(BitfieldSpecifier, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum ShoeOrientation {
    Normal,
    Rotated,
}

impl ShoeOrientation {
    pub fn get_other(self) -> Self {
        match self {
            ShoeOrientation::Normal => ShoeOrientation::Rotated,
            ShoeOrientation::Rotated => ShoeOrientation::Normal,
        }
    }
}

#[derive(
    BitfieldSpecifier, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, amplify::Display,
)]
#[display(Debug)]
#[repr(u8)]
#[bits = 2]
pub enum JointPosition {
    Zero,
    Plus90,
    Minus90,
}
impl std::fmt::Debug for JointPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            JointPosition::Zero => write!(f, "0"),
            JointPosition::Plus90 => write!(f, "+90"),
            JointPosition::Minus90 => write!(f, "-90"),
        }
    }
}

impl JointPosition {
    pub fn opposite(self) -> Self {
        match self {
            JointPosition::Zero => JointPosition::Zero,
            JointPosition::Plus90 => JointPosition::Minus90,
            JointPosition::Minus90 => JointPosition::Plus90,
        }
    }

    pub fn rotated(self, angle: atoms::RotationAngle) -> Option<JointPosition> {
        match (self, angle) {
            (JointPosition::Zero, atoms::RotationAngle::Plus90) => Some(JointPosition::Plus90),
            (JointPosition::Zero, atoms::RotationAngle::Minus90) => Some(JointPosition::Minus90),
            (JointPosition::Plus90, atoms::RotationAngle::Minus90)
            | (JointPosition::Minus90, atoms::RotationAngle::Plus90) => Some(JointPosition::Zero),
            (JointPosition::Plus90, atoms::RotationAngle::Plus90)
            | (JointPosition::Minus90, atoms::RotationAngle::Minus90) => None,
        }
    }
}

/// Canonized representation:
/// - when `other_body_dir.is_dir_to_plus` is true, the VoxelBody is assumed a BodyA
///     otherwise it's assumed BodyB
/// - when `is_shoe_rotated` is `false`, X-connectors are in the next axis
///         - `other_body_dir.axis` is `Axis::X` => X-connectors are in Y axis
///         - `other_body_dir.axis` is `Axis::Y` => X-connectors are in Z axis
///         - `other_body_dir.axis` is `Axis::Z` => X-connectors are in X axis
///     otherwise X-connectors are in the previous axis
///         - `other_body_dir.axis` is `Axis::X` => X-connectors are in Z axis
///         - `other_body_dir.axis` is `Axis::Y` => X-connectors are in X axis
///         - `other_body_dir.axis` is `Axis::Z` => X-connectors are in Y axis
/// - when `joint_pos` is `Zero`, the shoe joint is in the zero position
///     when `joint_pos` is `Plus90` the Z-connector is towards the plus axis
///     otherwise the Z-connector is towards the minus axis
#[bitfield(bits = 6)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, BitfieldSpecifier)]
pub struct VoxelBody {
    pub other_body_dir: atoms::Direction,
    pub is_shoe_rotated: bool,
    pub joint_pos: JointPosition,
}
static_assertions::assert_eq_size!(VoxelBody, u8);

pub type VoxelBodyWithPos = (VoxelBody, VoxelPos);

impl VoxelBody {
    pub fn new_with(
        other_body_dir: atoms::Direction,
        is_shoe_rotated: bool,
        joint_pos: JointPosition,
    ) -> Self {
        Self::new()
            .with_other_body_dir(other_body_dir)
            .with_is_shoe_rotated(is_shoe_rotated)
            .with_joint_pos(joint_pos)
    }

    pub fn x_conns_axis(self) -> atoms::Axis {
        let other_body_axis = self.other_body_dir().axis();
        if self.is_shoe_rotated() {
            other_body_axis.prev_axis()
        } else {
            other_body_axis.next_axis()
        }
    }

    pub fn z_conn_dir(self) -> atoms::Direction {
        let other_body_axis = self.other_body_dir().axis();
        let z_conn_axis = if self.is_shoe_rotated() {
            other_body_axis.next_axis()
        } else {
            other_body_axis.prev_axis()
        };
        match self.joint_pos() {
            JointPosition::Zero => self.other_body_dir().opposite(),
            JointPosition::Plus90 => atoms::Direction::new_with(z_conn_axis, true),
            JointPosition::Minus90 => atoms::Direction::new_with(z_conn_axis, false),
        }
    }

    pub fn get_connectors_dirs(self) -> [atoms::Direction; 3] {
        let x_conns_axis = self.x_conns_axis();
        [
            atoms::Direction::new_with(x_conns_axis, true),
            atoms::Direction::new_with(x_conns_axis, false),
            self.z_conn_dir(),
        ]
    }
}

pub fn get_neighbour_pos(body_with_pos: VoxelBodyWithPos) -> Result<VoxelPos, String> {
    body_with_pos
        .0
        .other_body_dir()
        .update_position(body_with_pos.1 .0)
        .map_err(|err_str| format!("Other body direction error ({err_str})"))
        .map(VoxelPos)
}
