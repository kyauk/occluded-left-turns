"""
Pygame-based visualization for the left-turn simulation by Claude Code.

Displays:
- Intersection geometry (roads, lanes, stop lines)
- Ego vehicle and turn path
- Other vehicles
- Real-time simulation playback
"""

import pygame
import sys
from typing import Optional
from world_state import WorldState, Actor, ActorType, Action
from world_geometry import SceneGeometry, Vector
from simulator import simulate_trajectory, check_collisons


# Colors (matching your screenshot's aesthetic)
GRASS_GREEN = (139, 195, 74)
ROAD_GRAY = (96, 96, 96)
WHITE = (255, 255, 255)
RED = (244, 67, 54)
ORANGE = (255, 152, 0)
BLACK = (0, 0, 0)
YELLOW = (255, 235, 59)


class Visualizer:
    """Pygame-based visualizer for intersection simulation."""

    def __init__(self, width: int = 1200, height: int = 800, scale: float = 60.0):
        """Initialize pygame and create window.

        Args:
            width: Window width in pixels
            height: Window height in pixels
            scale: Pixels per meter (controls zoom level)
        """
        pygame.init()
        self.width = width
        self.height = height
        self.scale = scale
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Occluded Left Turn Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

    def world_to_screen(self, pos: Vector) -> tuple[int, int]:
        """Convert world coordinates (meters) to screen coordinates (pixels).

        World origin (0, 0) maps to center of screen.
        Y-axis is flipped (world +Y is up, screen +Y is down).
        """
        screen_x = int(self.width / 2 + pos.x * self.scale)
        screen_y = int(self.height / 2 - pos.y * self.scale)
        return (screen_x, screen_y)

    def draw_background(self):
        """Fill screen with grass color."""
        self.screen.fill(GRASS_GREEN)

    def draw_geometry(self, geometry: SceneGeometry):
        """Draw intersection roads and markings."""
        # Draw roads (rectangles)
        self._draw_road(geometry.far_road, ROAD_GRAY)
        self._draw_road(geometry.near_road, ROAD_GRAY)
        self._draw_road(geometry.near_cross, ROAD_GRAY)
        self._draw_road(geometry.far_cross_near, ROAD_GRAY)
        self._draw_road(geometry.far_cross_far, ROAD_GRAY)

        # Draw center lines (yellow dashed)
        self._draw_dashed_line(geometry.middle_road, YELLOW, dash_length=0.3)

        # Draw stop line (white solid)
        self._draw_solid_line(geometry.stop_line, WHITE, thickness=3)

    def _draw_road(self, rect, color):
        """Draw a rectangular road segment."""
        # Convert world rect to screen rect
        top_left = self.world_to_screen(Vector(rect.x_min if rect.x_min != float('-inf') else -10,
                                                rect.y_max))
        width = int((rect.x_max if rect.x_max != float('inf') else 10) -
                   (rect.x_min if rect.x_min != float('-inf') else -10)) * self.scale
        height = int((rect.y_max - rect.y_min) * self.scale)

        # Handle infinite bounds by clipping to screen
        if rect.x_min == float('-inf'):
            top_left = (0, top_left[1])
            width = self.width

        pygame.draw.rect(self.screen, color, (*top_left, width, height))

    def _draw_solid_line(self, line, color, thickness=2):
        """Draw a horizontal line across the screen."""
        # For horizontal line (b=1, c=-y_value), y = -c/b
        if line.b != 0:
            y_world = -line.c / line.b
            y_screen = self.world_to_screen(Vector(0, y_world))[1]
            pygame.draw.line(self.screen, color, (0, y_screen), (self.width, y_screen), thickness)

    def _draw_dashed_line(self, line, color, dash_length=0.5, gap_length=0.3):
        """Draw a dashed horizontal line."""
        if line.b != 0:
            y_world = -line.c / line.b
            y_screen = self.world_to_screen(Vector(0, y_world))[1]

            # Draw dashes across screen width
            x_world = -10
            while x_world < 10:
                start_screen = self.world_to_screen(Vector(x_world, y_world))
                end_screen = self.world_to_screen(Vector(x_world + dash_length, y_world))
                pygame.draw.line(self.screen, color, start_screen, end_screen, 2)
                x_world += dash_length + gap_length

    def draw_turn_path(self, geometry: SceneGeometry, color=WHITE, show_waypoints=True):
        """Draw the planned turn path as a dashed arc."""
        path = geometry.turn_path()

        # Draw line segments between waypoints
        for i in range(len(path) - 1):
            start_pos = path[i][1]
            end_pos = path[i + 1][1]
            start_screen = self.world_to_screen(start_pos)
            end_screen = self.world_to_screen(end_pos)

            # Dashed line
            self._draw_dashed_segment(start_screen, end_screen, color, RED if i % 2 else WHITE)

        # Draw waypoint markers
        if show_waypoints:
            for _, pos in path:
                screen_pos = self.world_to_screen(pos)
                pygame.draw.circle(self.screen, RED, screen_pos, 4)

    def _draw_dashed_segment(self, start, end, color1, color2, segments=8):
        """Draw a dashed line segment alternating colors."""
        dx = (end[0] - start[0]) / segments
        dy = (end[1] - start[1]) / segments

        for i in range(segments):
            if i % 2 == 0:
                seg_start = (start[0] + i * dx, start[1] + i * dy)
                seg_end = (start[0] + (i + 1) * dx, start[1] + (i + 1) * dy)
                pygame.draw.line(self.screen, color1, seg_start, seg_end, 2)

    def draw_actor(self, actor: Actor):
        """Draw a vehicle as a rotated rectangle with wheels."""
        # Determine color based on actor type
        if actor.actor_type == ActorType.EGO:
            body_color = RED
            wheel_color = BLACK
        elif actor.actor_type == ActorType.VEHICLE:
            body_color = ORANGE
            wheel_color = BLACK
        else:  # PEDESTRIAN
            body_color = (33, 150, 243)  # Blue
            wheel_color = BLACK

        # Get screen position
        center = self.world_to_screen(actor.position)

        # Calculate rotation angle from velocity
        if actor.velocity.x != 0 or actor.velocity.y != 0:
            import math
            angle = math.atan2(actor.velocity.y, actor.velocity.x)
            angle_deg = math.degrees(angle) - 90  # -90 because 0° should point up
        else:
            angle_deg = 0

        # Create vehicle surface (width x length in pixels)
        width_px = int(actor.dims[0] * self.scale)
        length_px = int(actor.dims[1] * self.scale)

        # Draw vehicle body
        vehicle_surface = pygame.Surface((width_px, length_px), pygame.SRCALPHA)
        pygame.draw.rect(vehicle_surface, body_color, (0, 0, width_px, length_px), border_radius=4)

        # Draw wheels (4 small black circles)
        wheel_radius = max(2, int(width_px * 0.15))
        wheel_inset_x = int(width_px * 0.15)
        wheel_inset_y = int(length_px * 0.2)

        # Front wheels
        pygame.draw.circle(vehicle_surface, wheel_color, (wheel_inset_x, wheel_inset_y), wheel_radius)
        pygame.draw.circle(vehicle_surface, wheel_color, (width_px - wheel_inset_x, wheel_inset_y), wheel_radius)

        # Rear wheels
        pygame.draw.circle(vehicle_surface, wheel_color, (wheel_inset_x, length_px - wheel_inset_y), wheel_radius)
        pygame.draw.circle(vehicle_surface, wheel_color, (width_px - wheel_inset_x, length_px - wheel_inset_y), wheel_radius)

        # Rotate and blit
        rotated = pygame.transform.rotate(vehicle_surface, angle_deg)
        rotated_rect = rotated.get_rect(center=center)
        self.screen.blit(rotated, rotated_rect.topleft)

    def draw_state(self, state: WorldState, show_turn_path: bool = True):
        """Draw complete world state."""
        self.draw_background()
        self.draw_geometry(state.geometry)

        if show_turn_path:
            self.draw_turn_path(state.geometry)

        # Draw all actors
        for actor in state.actors:
            self.draw_actor(actor)

        # Draw HUD
        self._draw_hud(state)

    def _draw_hud(self, state: WorldState):
        """Draw heads-up display with state info."""
        # Time
        time_text = self.font.render(f"Time: {state.time:.1f}s", True, BLACK)
        self.screen.blit(time_text, (10, 10))

        # Ego velocity
        ego_speed = (state.ego.velocity.x**2 + state.ego.velocity.y**2)**0.5
        speed_text = self.font.render(f"Ego Speed: {ego_speed:.1f} m/s", True, BLACK)
        self.screen.blit(speed_text, (10, 40))

        # Number of vehicles
        num_vehicles = sum(1 for a in state.actors if a.actor_type == ActorType.VEHICLE)
        vehicles_text = self.font.render(f"Vehicles: {num_vehicles}", True, BLACK)
        self.screen.blit(vehicles_text, (10, 70))

        # Check collisions
        collisions = check_collisons(state)
        if collisions:
            collision_text = self.font.render("COLLISION!", True, RED)
            self.screen.blit(collision_text, (self.width // 2 - 50, 10))

    def animate_trajectory(self, states: list[WorldState], fps: int = 30,
                          realtime_speed: float = 1.0):
        """Animate a pre-computed trajectory.

        Args:
            states: List of WorldState snapshots
            fps: Frame rate (frames per second)
            realtime_speed: Playback speed multiplier (1.0 = realtime, 2.0 = double speed)
        """
        if not states:
            return

        # Calculate time per frame
        if len(states) > 1:
            dt = states[1].time - states[0].time
        else:
            dt = 0.1

        frames_per_state = max(1, int(fps * dt / realtime_speed))

        state_idx = 0
        frame_count = 0
        running = True
        paused = False

        while running and state_idx < len(states):
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        paused = not paused
                    elif event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_RIGHT and paused:
                        # Step forward one frame when paused
                        state_idx = min(state_idx + 1, len(states) - 1)
                        frame_count = 0
                    elif event.key == pygame.K_LEFT and paused:
                        # Step backward one frame when paused
                        state_idx = max(state_idx - 1, 0)
                        frame_count = 0

            if not paused:
                # Draw current state
                self.draw_state(states[state_idx])

                # Draw pause indicator
                pause_text = self.font.render("PAUSED (SPACE to resume, arrows to step)", True, BLACK)
            else:
                self.draw_state(states[state_idx])
                pause_text = self.font.render("PAUSED (SPACE to resume, arrows to step)", True, RED)
                self.screen.blit(pause_text, (self.width // 2 - 200, self.height - 30))

            pygame.display.flip()

            if not paused:
                # Advance to next state
                frame_count += 1
                if frame_count >= frames_per_state:
                    state_idx += 1
                    frame_count = 0

            self.clock.tick(fps)

        # Keep window open at end
        if running:
            self._wait_for_close()

    def _wait_for_close(self):
        """Wait for user to close window."""
        waiting = True
        while waiting:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    waiting = False
            self.clock.tick(30)

    def close(self):
        """Clean up pygame resources."""
        pygame.quit()


def demo_visualization():
    """Demo: visualize a simple turn scenario."""
    from world_state import WorldState, Actor, ActorType, Action

    # Create scene
    geometry = SceneGeometry()

    # Ego at stop line (approaching from south, wanting to turn left/west)
    ego = Actor(
        position=Vector(0, 1),
        velocity=Vector(0, 0),
        dims=(1.7, 3.4),  # 0.85x smaller (was 2.0, 4.0)
        actor_type=ActorType.EGO
    )

    # Oncoming vehicle from the RIGHT (east), traveling west
    # This is the traffic ego must yield to when making unprotected left turn
    oncoming = Actor(
        position=Vector(8, 5),      # Start on the right (east) side
        velocity=Vector(-6, 0),     # Moving left (west) at 6 m/s
        dims=(1.7, 3.4),  # 0.85x smaller (was 2.0, 4.0)
        actor_type=ActorType.VEHICLE
    )

    initial_state = WorldState(
        geometry=geometry,
        time=0.0,
        ego_turn_start_time=None,
        actors=(ego, oncoming)
    )

    # Simulate ego turning
    states = simulate_trajectory(initial_state, Action.TURN, duration=4.0, dt=0.1)

    # Visualize
    viz = Visualizer(scale=50)
    viz.animate_trajectory(states, fps=30, realtime_speed=0.5)  # Half speed for clarity
    viz.close()


if __name__ == "__main__":
    demo_visualization()
