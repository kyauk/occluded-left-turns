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


@dataclass
class WorldState:
    """Complete state of the world at a given time."""
    geometry: SceneGeometry
    time: float
    actors: list[Actor]

    @property
    def vehicles(self) -> list[Actor]:
        """Filter actors to vehicles only."""
        return [a for a in self.actors if a.actor_type == ActorType.VEHICLE]

    @property
    def pedestrians(self) -> list[Actor]:
        """Filter actors to pedestrians only."""
        return [a for a in self.actors if a.actor_type == ActorType.PEDESTRIAN]

    @property
    def ego(self) -> Actor | None:
        """Get the ego vehicle (if present)."""
        ego_actors = [a for a in self.actors if a.actor_type == ActorType.EGO]
        return ego_actors[0] if ego_actors else None
