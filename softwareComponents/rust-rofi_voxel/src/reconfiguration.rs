use crate::connectivity::ConnectivityGraph;
use crate::module_move::Move;
use crate::module_repr::{get_all_module_reprs, get_bodies};
use crate::voxel_world::VoxelWorld;
use std::assert_matches::{assert_matches, debug_assert_matches};
use std::collections::{HashMap, VecDeque};
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};

pub static TRUE_CUTS: AtomicU64 = AtomicU64::new(0);
pub static FALSE_CUTS: AtomicU64 = AtomicU64::new(0);

pub static ALL_WORLDS: AtomicU64 = AtomicU64::new(0);
pub static ADDED_WORLDS: AtomicU64 = AtomicU64::new(0);
pub static NEXT_WORLDS_BEFORE_ADD: AtomicU64 = AtomicU64::new(0);
pub static COLLIDED_WORLDS: AtomicU64 = AtomicU64::new(0);
pub static STEPS_COMPUTED: AtomicU64 = AtomicU64::new(0);

#[derive(Debug, Clone, Copy, amplify::Display, amplify::Error)]
#[display(doc_comments)]
pub enum Error {
    /// Bodies count doesn't match
    BodiesCountDoesNotMatch,
    /// Reconfiguration path not found
    PathNotFound,
}

fn get_path_to(
    goal: &VoxelWorld,
    parent_worlds: &HashMap<Rc<VoxelWorld>, Option<Rc<VoxelWorld>>>,
) -> Vec<Rc<VoxelWorld>> {
    assert!(parent_worlds.contains_key(goal), "Missing goal parent");

    let mut parent = Some(parent_worlds.get_key_value(goal).unwrap().0.clone());
    let mut path = std::iter::from_fn(|| {
        let child = parent.as_ref()?;
        assert!(parent_worlds.contains_key(child), "Missing prev node");
        let new_parent = parent_worlds[child].clone();
        std::mem::replace(&mut parent, new_parent)
    })
    .collect::<Vec<_>>();

    path.reverse();
    path
}

// Can return non-valid worlds
fn all_next_worlds<'a>(
    world: &'a VoxelWorld,
    connectivity_graph: &'a ConnectivityGraph<'a>,
) -> impl Iterator<Item = VoxelWorld> + 'a {
    get_all_module_reprs(world).flat_map(move |module| {
        let [body_a, body_b] = get_bodies(module, world);
        connectivity_graph
            .all_cuts_by_module(module)
            .flat_map(move |split| {
                Move::all_possible_moves(body_a.0, body_b.0)
                    .map(move |module_move| module_move.apply(module, split.clone()))
            })
    })
}

/// Returns all possible next worlds not normalized
pub fn all_possible_next_worlds_not_norm<'a>(
    world: &'a VoxelWorld,
    connectivity_graph: &'a ConnectivityGraph<'a>,
    goal_bodies_count: usize,
) -> impl Iterator<Item = VoxelWorld> + 'a {
    all_next_worlds(world, connectivity_graph).filter_map(move |new_world| {
        ALL_WORLDS.fetch_add(1, Ordering::Relaxed);
        if new_world.all_bodies().count() != goal_bodies_count {
            // Collision occured
            COLLIDED_WORLDS.fetch_add(1, Ordering::Relaxed);
            return None;
        }

        debug_assert_matches!(
            new_world.check_voxel_world(),
            Ok(()),
            "Next world has correct body count, but is invalid ({:?})",
            new_world
        );
        Some(new_world)
    })
}

/// Returns all possible next worlds
///
/// The returned world is normalized (one of possible eq norm worlds)
pub fn all_possible_next_worlds<'a>(
    world: &'a VoxelWorld,
    connectivity_graph: &'a ConnectivityGraph<'a>,
    goal_bodies_count: usize,
) -> impl Iterator<Item = VoxelWorld> + 'a {
    all_possible_next_worlds_not_norm(world, connectivity_graph, goal_bodies_count)
        .map(VoxelWorld::as_one_of_norm_eq_world)
}

fn find_parents_from_to(
    init: &VoxelWorld,
    goal: &VoxelWorld,
) -> Result<HashMap<Rc<VoxelWorld>, Option<Rc<VoxelWorld>>>, Error> {
    assert!(goal.is_normalized());

    let goal_bodies_count = goal.all_bodies().count();
    if init.all_bodies().count() != goal_bodies_count {
        return Err(Error::BodiesCountDoesNotMatch);
    }

    let mut parent_worlds = init
        .normalized_eq_worlds()
        .map(|init| (Rc::new(init), None))
        .collect::<HashMap<_, _>>();

    if parent_worlds.contains_key(goal) {
        return Ok(parent_worlds);
    }

    let init = parent_worlds.keys().next().unwrap().clone();
    let mut worlds_to_visit = VecDeque::from([init]);

    while let Some(current) = worlds_to_visit.pop_front() {
        debug_assert_matches!(current.check_voxel_world(), Ok(()));
        debug_assert!(current.is_normalized());
        STEPS_COMPUTED.fetch_add(1, Ordering::Relaxed);

        let connectivity_graph = ConnectivityGraph::compute_from(&current);
        for new_world in all_possible_next_worlds(&current, &connectivity_graph, goal_bodies_count)
        {
            NEXT_WORLDS_BEFORE_ADD.fetch_add(1, Ordering::Relaxed);
            debug_assert_matches!(new_world.check_voxel_world(), Ok(()));
            debug_assert!(new_world.is_normalized());
            if parent_worlds.contains_key(&new_world) {
                debug_assert!(
                    new_world
                        .normalized_eq_worlds()
                        .all(|norm_world| parent_worlds.contains_key(&norm_world)),
                    "parent_worlds contains a normalized variant, but not itself {new_world:?}"
                );
                continue;
            }

            debug_assert!(
                new_world
                    .normalized_eq_worlds()
                    .all(|norm_world| !parent_worlds.contains_key(&norm_world)),
                "parent_worlds contains itself but not some normalized variant {new_world:?}"
            );

            parent_worlds.extend(
                new_world
                    .normalized_eq_worlds()
                    .map(|norm_world| (Rc::new(norm_world), Some(current.clone()))),
            );

            if parent_worlds.contains_key(goal) {
                return Ok(parent_worlds);
            }

            debug_assert!(new_world.is_normalized());
            // Get the world as Rc from the parent_worlds
            let new_world = parent_worlds
                .get_key_value(&new_world)
                .expect("Has to contain new_world")
                .0
                .clone();

            worlds_to_visit.push_back(new_world);
            ADDED_WORLDS.fetch_add(1, Ordering::Relaxed);
        }
    }

    Err(Error::PathNotFound)
}

pub fn compute_reconfiguration_moves(
    init: &VoxelWorld,
    goal: VoxelWorld,
) -> Result<Vec<Rc<VoxelWorld>>, Error> {
    assert_matches!(init.check_voxel_world(), Ok(()));
    assert_matches!(goal.check_voxel_world(), Ok(()));

    let goal = goal.as_one_of_norm_eq_world();

    let parent_worlds = find_parents_from_to(init, &goal)?;

    log_counters();

    Ok(get_path_to(&goal, &parent_worlds))
}

pub fn log_counters() {
    let true_cuts = TRUE_CUTS.load(Ordering::Relaxed);
    let false_cuts = FALSE_CUTS.load(Ordering::Relaxed);
    log::info!("ALL_CUTS: {}", true_cuts + false_cuts);
    log::info!("TRUE_CUTS: {}", true_cuts);
    log::info!("FALSE_CUTS: {}", false_cuts);

    log::info!("ALL_WORLDS: {}", ALL_WORLDS.load(Ordering::Relaxed));
    log::info!(
        "COLLIDED_WORLDS: {}",
        COLLIDED_WORLDS.load(Ordering::Relaxed)
    );
    log::info!("STEPS_COMPUTED: {}", STEPS_COMPUTED.load(Ordering::Relaxed));
    log::info!(
        "NEXT_WORLDS_BEFORE_ADD: {}",
        NEXT_WORLDS_BEFORE_ADD.load(Ordering::Relaxed)
    );
    log::info!("ADDED_WORLDS: {}", ADDED_WORLDS.load(Ordering::Relaxed));
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::atoms::{Axis, Direction};
    use crate::pos::VoxelPos;
    use crate::voxel::{body::JointPosition, VoxelBody};

    #[test]
    pub fn test_reconfig_no_steps() {
        let (world, _) = VoxelWorld::from_bodies([
            (
                VoxelBody::new_with(
                    Direction::new_with(Axis::X, true),
                    true,
                    JointPosition::Plus90,
                ),
                VoxelPos([0; 3]),
            ),
            (
                VoxelBody::new_with(
                    Direction::new_with(Axis::X, false),
                    false,
                    JointPosition::Minus90,
                ),
                VoxelPos([1, 0, 0]),
            ),
        ])
        .unwrap();

        let result = compute_reconfiguration_moves(&world, world.clone()).unwrap();
        assert_eq!(result.len(), 1);
    }

    #[test]
    pub fn test_reconfig_one_step() {
        let (init_world, _) = VoxelWorld::from_bodies([
            (
                VoxelBody::new_with(
                    Direction::new_with(Axis::X, true),
                    true,
                    JointPosition::Plus90,
                ),
                VoxelPos([0; 3]),
            ),
            (
                VoxelBody::new_with(
                    Direction::new_with(Axis::X, false),
                    false,
                    JointPosition::Minus90,
                ),
                VoxelPos([1, 0, 0]),
            ),
        ])
        .unwrap();
        let (goal_world, _) = VoxelWorld::from_bodies([
            (
                VoxelBody::new_with(
                    Direction::new_with(Axis::X, true),
                    true,
                    JointPosition::Plus90,
                ),
                VoxelPos([0; 3]),
            ),
            (
                VoxelBody::new_with(
                    Direction::new_with(Axis::X, false),
                    true,
                    JointPosition::Minus90,
                ),
                VoxelPos([1, 0, 0]),
            ),
        ])
        .unwrap();

        let result = compute_reconfiguration_moves(&init_world, goal_world).unwrap();
        assert_eq!(result.len(), 2);
    }
}
