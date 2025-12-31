# this is to allow referring functions to return the class type within the class
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from world_geometry import Vector, SceneGeometry


class Action(Enum):
    """Ego vehicle decision options."""
    ABSTAIN = "abstain"
    TURN = "turn"
    WAIT = "wait"

class ActorType(Enum):
    EGO = "ego"
    VEHICLE = "vehicle"
    PEDESTRIAN = "pedestrian"

@dataclass(frozen=True)
class Actor:
    """Moving object in the scene: vehicle, pedestrian, etc."""
    position: Vector
    velocity: Vector
    # box dimension
    dims: tuple[float, float]
    actor_type: ActorType  

    def step(self, dt: float) -> Actor:
        """Return new the new Actor after deterministic physics step."""
        return Actor(
            position=Vector(
                x=self.position.x + self.velocity.x * dt,
                y=self.position.y + self.velocity.y * dt
            ),
            velocity=self.velocity,
            dims=self.dims,
            actor_type=self.actor_type
        )


@dataclass(frozen=True)
class WorldState:
    """Complete state of the world at a given time. Immutable
    """
    geometry: SceneGeometry
    # global world time
    time: float
    # none if not turning, set to state.time when TURN action first chosen
    ego_turn_start_time: float | None
    actors: tuple[Actor, ...]  

    def __post_init__(self):
        """Validate invariants."""
        # only one ego can exist
        ego_count = sum(1 for a in self.actors if a.actor_type == ActorType.EGO)
        if ego_count == 0:
            raise ValueError("WorldState must contain exactly one ego vehicle (found 0)")
        if ego_count > 1:
            raise ValueError(f"WorldState must contain exactly one ego vehicle (found {ego_count})")

        # make sure actors and time all have non-negative dimensions
        for actor in self.actors:
            if actor.dims[0] <= 0 or actor.dims[1] <= 0:
                raise ValueError(f"Actor dimensions must be positive, got {actor.dims}")
        if self.time < 0:
            raise ValueError(f"Time must be non-negative, got {self.time}")

    @property
    def vehicles(self) -> list[Actor]:
        """Filter actors to vehicles only."""
        return [a for a in self.actors if a.actor_type == ActorType.VEHICLE]

    @property
    def pedestrians(self) -> list[Actor]:
        """Filter actors to pedestrians only."""
        return [a for a in self.actors if a.actor_type == ActorType.PEDESTRIAN]

    @property
    def ego(self) -> Actor:
        """Get the ego vehicle, which is guaranteed to exist"""
        ego_actors = [a for a in self.actors if a.actor_type == ActorType.EGO]
        return ego_actors[0]

__all__ = [
    "Action",
    "ActorType",
    "Actor",
    "WorldState"
]