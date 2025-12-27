"""
Docstring for world_geometry: Defining shapes required for the world, as well as a basic hard-coded scene using shapes.
"""

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
        return (
            self.x_min <= p.x <= self.x_max and
            self.y_min <= p.y <= self.y_max
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
    # make bounds and lines first
    top_boundary = Line(a=0,b=1,c= -6)
    far_center = Line(a=0,b=1,c= -5)
    middle_road = Line(a=0,b=1,c= -4)
    near_center = Line(a=0,b=1,c= -3)
    stop_line = Line(a=0,b=1,c= -1)

    # make rectangular properties
    far_road = Rect(x_min=-math.inf, x_max=math.inf, y_min=4,y_max=6)
    near_road = Rect(x_min=-math.inf, x_max=math.inf, y_min=2,y_max=4)
    near_cross = Rect(x_min=-2, x_max=0, y_min=1,y_max=2)
    far_cross_near = Rect(x_min=-3, x_max=-2, y_min=2,y_max=4)
    far_cross_far = Rect(x_min=-3, x_max=-2, y_min=4,y_max=6)
    
    def turn_path(self) -> list:
        """ Hard-coded trajectory for left turn from the stop line. 
        Mix of straight and Parabolic arc from stop line thoruhg intersection to far road

        **SHOULD ONLY BE INITIATED IF CAR IS @ STOP LINE, AND HAS MADE APPROPIATE STOP**

        Go straight (Points 1-3), make parabolic turn after passing center line Points (4-6)
        
        """
        turn_path = [
            Vector(0,2),
            Vector(0,3),
            Vector(0,4),
            Vector(-1.5,4.5),
            Vector(-1,5),
            Vector(-2,5)
        ]
        return turn_path
