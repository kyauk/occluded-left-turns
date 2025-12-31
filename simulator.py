
"""
Simulator to step forward in the world wrt to time, given an ego's action. 
Want to detect collisions between the actors
Want to provide ground truth outcomes for risk calculations
"""

from __future__ import annotations
from world_state import *
from world_geometry import *
import math


# --------------------------------
# COLLISION DETECTION
# --------------------------------
def has_collide(a: Actor, b: Actor) -> bool:
    """
    Checks if two actors' bounding boxes will overlap
    Simple axis-aligned rectangle intersection, where the 
    """
    # create bounding boxes for the actors
    a_box = Rect(
        x_min=a.position.x - a.dims[0]/2,
        x_max=a.position.x + a.dims[0]/2,
        y_min=a.position.y - a.dims[1]/2,
        y_max=a.position.y + a.dims[1]/2
    )
    b_box = Rect(
        x_min=b.position.x - b.dims[0]/2,
        x_max=b.position.x + b.dims[0]/2,
        y_min=b.position.y - b.dims[1]/2,
        y_max=b.position.y + b.dims[1]/2
    )

    return a_box.overlappingRectangles(b_box)

def check_collisons(state: WorldState) -> list[tuple[Actor, Actor]]:
    """ O(n^2) checking all pairs"""
    collisions = []
    for i, actor_a in enumerate(state.actors):
        for actor_b in state.actors[i+1:]:
            if has_collide(actor_a, actor_b):
                collisions.append((actor_a,actor_b))
    return collisions

    

# --------------------------------
# STEPPING THROUGH THE WORLD
# --------------------------------

def step_world(state: WorldState, ego_action: Action, dt: float) -> WorldState:
    """Evolve world state by dt seconds given ego's action.

    Returns a NEW WorldState (functional update pattern).

    Args:
        state: Current world state
        ego_action: Action chosen by ego vehicle
        dt: Time step in seconds (must be positive)

    Returns:
        New WorldState after dt seconds

    Raises:
        ValueError: If dt <= 0
    """
    if dt <= 0:
        raise ValueError(f"Time step dt must be positive, got {dt}")

    ego = state.ego
    new_ego_turn_start_time = state.ego_turn_start_time

    # Update ego based on action
    if ego_action == Action.TURN:
        # If this is the first TURN step, record when it started
        if state.ego_turn_start_time is None:
            new_ego_turn_start_time = state.time
            relative_t = 0.0
        else:
            relative_t = state.time - state.ego_turn_start_time

        # Get position and velocity from turn path
        new_position = state.geometry.get_turn_position_at_time(relative_t)
        new_velocity = state.geometry.get_turn_velocity_at_time(relative_t, dt)

        updated_ego = Actor(
            position=new_position,
            velocity=new_velocity,
            dims=ego.dims,
            actor_type=ego.actor_type
        )

    elif ego_action == Action.WAIT:
        # Ego remains stationary (velocity = 0)
        updated_ego = Actor(
            position=ego.position,
            velocity=Vector(0, 0),
            dims=ego.dims,
            actor_type=ego.actor_type
        )
        # Clear turn start time if we were turning before
        new_ego_turn_start_time = None

        # Ignoring Abstain for Now as it is not required for phase 1
        """
        elif ego_action == Action.ABSTAIN:
        # Abstain means "refuse to act" - ego stays frozen
        updated_ego = Actor(
            position=ego.position,
            velocity=Vector(0, 0),
            dims=ego.dims,
            actor_type=ego.actor_type
        )
        new_ego_turn_start_time = None
        """

    else:
        raise ValueError(f"Unknown action: {ego_action}")

    # Step all other actors forward using constant velocity kinematics
    updated_others = [actor.step(dt) for actor in state.actors if actor != ego]

    # Build new actor tuple (functional update)
    new_actors = tuple([updated_ego] + updated_others)

    # Return a new WorldState
    return WorldState(
        geometry=state.geometry,
        time=state.time + dt,
        ego_turn_start_time=new_ego_turn_start_time,
        actors=new_actors
    )

# ---------------------
# TRAJECTORY SIMULATION
# ---------------------
def simulate_trajectory(initial_state: WorldState, ego_action: Action, duration: float, dt: float=0.1) -> list[WorldState]:
    """Simulate a full trajectory for a given ego action.

    Repeatedly calls step_world() to generate a sequence of states.

    Args:
        initial_state: Starting world state
        ego_action: Action for ego to take (held constant throughout)
        duration: Total simulation time in seconds (must be positive)
        dt: Time step for each iteration (must be positive, default 0.1s)

    Returns:
        List of WorldState snapshots at each timestep, including initial_state

    Raises:
        ValueError: If duration <= 0 or dt <= 0
    """
    if duration <= 0:
        raise ValueError(f"Duration must be positive, got {duration}")
    if dt <= 0:
        raise ValueError(f"Time step dt must be positive, got {dt}")

    states = [initial_state]
    current_state = initial_state
    elapsed = 0.0

    # Simulate until we've covered the full duration
    while elapsed < duration:
        # Don't overshoot the duration
        time_step = min(dt, duration - elapsed)
        current_state = step_world(current_state, ego_action, time_step)
        states.append(current_state)
        elapsed += time_step

    return states

# ---------------------
# OUTCOME EVALUATION
# ---------------------