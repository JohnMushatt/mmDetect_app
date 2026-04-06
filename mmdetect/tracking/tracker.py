from __future__ import annotations
from dataclasses import dataclass, field
from collections import deque

@dataclass
class TrackedTarget:
    tracl_id : int
    position : tuple[float, float] # (x, y) in room coordinates
    trail : deque[tuple[float, float]] # (x, y) in room coordinates
    velocity : tuple[float, float] = (0.0, 0.0) # (vx, vy) in room coordinates
    consecutive_misses : int = 0# For fading trail in GUI
    age : int = 0# For pruning stale tracks


class Tracker:
    def __init__(
        self,
        max_association_dist : float = 500.0,
        max_stale_frames : int = 20,
        trail_length : int = 100,
    ) -> None:
        self._max_association_dist = max_association_dist
        self._max_stale_frames = max_stale_frames
        self._trail_length = trail_length
        self._tracks : dict[int, TracketTarget] = {} # track_id -> TrackedTarget
        self._next_track_id : int = 0

    @property
    def active_tracks(self) -> list[TrackedTarget]:
        return list(self._tracks.values())

    def update(self, detections: list[tuple[float, float]]) -> list[TrackedTarget]:
        """
        Feed new detections into tracker, returns list of active tracks (not tracked targets)
        """
        used_tracks = set[int] = set()
        used_detections: set[int] = set()

        assignments: list[tuple[float, int, int]] = []
        for det_idx, (dx, dy) in enumerate(detections):
            for track_id, track in self._tracks.items():
                last_x, last_y = target.position
                # Calculate distance between detection and track position
                dist = math.hypot(dx - last_x, dy - last_y)
                assignments.append((dist, track_id, det_idx))
        assignments.sort() # greedy nearest-neighbor

        for dist, track_id, det_idx in assignments:
            # Skip if used track or detection already used
            if track_id in used_tracks or det_idx in used_detections:
                continue 
            # Skip if distance is too far
            if dist > self._max_association_dist:
                continue
            # Remaining tracks are unmatched detections

            target = self._tracks[track_id]
            old_pos = target.position
            new_post = detections[det_idx]
            target.position = new_pos
            target.trail.append(new_pos)
            target.velocity = (new_pos[0] - old_pos[0], new_pos[1] - old_pos[1])
            target.age += 1
            target.consecutive_misses = 0
            used_tracks.add(track_id)
            used_detections.add(det_idx)

        # Create new tracks for unmatched detections
        for det_idx, (dx, dy) in enumerate(detections):
            if det_idx not in used_detections:
                trail = deque(maxlen=self._trail_length)
                trail.append(pos)
                self._tracks[self._next_track_id] = TrackedTarget(
                    track_id=self._next_track_id,
                    position = pos,
                    trail = trail,
                )
                self._next_track_id += 1

        stale_ids = []
        # Prune stale tracks
        for track_id, target in self._tracks.items():
            # Skip if track is still active, aka not present in used_tracks
            if track_id not in used_tracks:
                # Update consecutive misses
                target.consecutive_misses += 1
                # Prune if consecutive misses exceeds threshold
                if target.consecutive_misses >= self._max_stale_frames:
                    stale_ids.append(track_id)
        for track_id in stale_ids:
            del self._tracks[track_id]
    
    def reset(self) -> None:
        self._tracks.clear()
        self._next_track_id = 0