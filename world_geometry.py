"""
Docstring for world_geometry: Defining shapes required for the world, as well as a basic hard-coded scene using shapes.
"""
from __future__ import annotations
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Vector:
    x: float
    y: float

@dataclass(frozen=True)
class Rect:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    
    def withinRect(self, p: Vector) -> bool:
        """ Returns if a point is whtin a Rectangle"""
        return (
            self.x_min <= p.x <= self.x_max and
            self.y_min <= p.y <= self.y_max
        )
    def overlappingRectangles(self, other: Rect) -> bool:
        """ Returns if two subjecting rectangles are overlapping"""
        return not (
            # self is to the left
            self.x_max < other.x_min or
            # self is to the right
            self.x_min > other.x_max or
            # self is below
            self.y_max < other.y_min or
            # self is above
            self.y_min > other.y_max
        )


@dataclass(frozen=True)
class Line:
    # ax + by + c = 0
    a: float
    b: float
    c: float
    def signed_value(self, p: Vector) -> float:
        return self.a * p.x + self.b * p.y + self.c
    
@dataclass(frozen=True)
class SceneGeometry():
    """
    Concrete geometry for a specific occluded left-turn scenario.
    Hardcoded coordinates define this exact intersection layout.
    """
    # ---------
    # LINES
    # ---------
    top_boundary = Line(a=0,b=1,c= -6)
    far_center = Line(a=0,b=1,c= -5)
    middle_road = Line(a=0,b=1,c= -4)
    near_center = Line(a=0,b=1,c= -3)
    stop_line = Line(a=0,b=1,c= -1)

    # ---------
    # RECTANGLES
    # ---------
    far_road = Rect(x_min=-math.inf, x_max=math.inf, y_min=4,y_max=6)
    near_road = Rect(x_min=-math.inf, x_max=math.inf, y_min=2,y_max=4)
    near_cross = Rect(x_min=-3, x_max=0, y_min=-3,y_max=2)  # Longer (extends to y=-3) and wider (3 units)
    far_cross_near = Rect(x_min=-4, x_max=-3, y_min=2,y_max=3)
    far_cross_far = Rect(x_min=-4, x_max=-3, y_min=3,y_max=6)
    
    def turn_path(self) -> list[tuple[float, Vector]]:
        """
        Hard-coded trajectory for left turn from the stop line.
        Returns list of (time, position) tuples defining the ego path.

        **SHOULD ONLY BE INITIATED IF CAR IS @ STOP LINE, AND HAS MADE APPROPRIATE STOP**

        Mix of straight and parabolic arc from stop line through intersection to far road.
        Go straight (Points 1-3), make parabolic turn after passing center line (Points 4-6).
        """
        return [
            # needs to start moving at stop line
            (0.0, Vector(0, 1)),      
            (0.3, Vector(0, 2)),   
            (0.5, Vector(0, 3)),      
            (1.5, Vector(-0.5, 4.5)), 
            (2.0, Vector(-1, 5)),     
            (2.5, Vector(-2, 5))      
        ]

    def get_turn_position_at_time(self, t: float) -> Vector:
        """
        Get ego position at time t along the turn path.
        Uses linear interpolation between waypoints.

        Args:
            t: Time elapsed since TURN decision was made (t=0 is decision time)

        Returns:
            Position along turn trajectory at time t

        Raises:
            ValueError: If t < 0 (invalid - can't query position before decision)
        """
        path = self.turn_path()

        # time can not be negative
        if t < 0:
            raise ValueError(f"Time t={t} cannot be negative. Turn path starts at t=0.")

        # At or before first waypoint
        if t <= path[0][0]:
            return path[0][1]

        # after turn path ends - continue straight at final segment velocity
        if t >= path[-1][0]:
            # Compute velocity from last segment
            t_last = path[-1][0]
            t_prev = path[-2][0]
            pos_last = path[-1][1]
            pos_prev = path[-2][1]

            velocity = Vector(
                x=(pos_last.x - pos_prev.x) / (t_last - t_prev),
                y=(pos_last.y - pos_prev.y) / (t_last - t_prev)
            )

            # extrapolate the position beyond final waypoint
            dt_extra = t - t_last
            return Vector(
                x=pos_last.x + velocity.x * dt_extra,
                y=pos_last.y + velocity.y * dt_extra
            )

        # in between waypoints
        for i in range(len(path) - 1):
            t0, pos0 = path[i]
            t1, pos1 = path[i + 1]

            if t0 <= t <= t1:
                # Linear interpolation
                alpha = (t - t0) / (t1 - t0)
                return Vector(
                    x=pos0.x + alpha * (pos1.x - pos0.x),
                    y=pos0.y + alpha * (pos1.y - pos0.y)
                )

        # Shouldn't reach here
        return path[-1][1]
    
    def get_turn_velocity_at_time(self, t: float, dt: float = 0.01) -> Vector:
        """
        Compute ego velocity at time t along turn path.
        Uses finite difference approximation: v(t) ≈ (pos(t+dt) - pos(t)) / dt

        Args:
            t: Time elapsed since TURN decision (relative time)
            dt: Small time step for numerical differentiation

        Returns:
            Velocity vector at time t
        """
        pos_now = self.get_turn_position_at_time(t)
        pos_future = self.get_turn_position_at_time(t + dt)
        return Vector(
            x=(pos_future.x - pos_now.x) / dt,
            y=(pos_future.y - pos_now.y) / dt 
        )


__all__ = [
    "Vector",
    "Rect",
    "Line",
    "SceneGeometry"
]