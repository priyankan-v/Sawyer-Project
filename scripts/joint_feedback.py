"""Merge mixed JointState messages without losing unrelated joint feedback."""

import math


class JointFeedback:
    """Cache position and velocity independently, with per-joint receipt times.

    A JointState message may contain any subset of joints, and either numeric
    array may be empty. Unrelated joints never refresh or invalidate required
    joints. Missing/invalid fields never refresh their last valid receipt time.
    """

    def __init__(self, names):
        self.names = tuple(names)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("Required joint names must be nonempty and unique")
        self.position = {}
        self.velocity = {}
        self.position_at = {}
        self.velocity_at = {}

    def update(self, names, positions, velocities, now):
        relevant = set(self.names).intersection(names)
        if not relevant:
            return False
        if len(set(names)) != len(names):
            raise ValueError("Duplicate names in arm JointState message")
        if not math.isfinite(now):
            raise ValueError("Joint feedback receipt time must be finite")
        updates = []
        for label, values, cache, times in (
            ("position", positions, self.position, self.position_at),
            ("velocity", velocities, self.velocity, self.velocity_at),
        ):
            if len(values) == 0:
                continue
            if len(values) != len(names):
                raise ValueError(f"Arm JointState {label} length {len(values)} != names length {len(names)}")
            selected = {name: float(value) for name, value in zip(names, values) if name in relevant}
            if not all(math.isfinite(value) for value in selected.values()):
                raise ValueError(f"Non-finite arm JointState {label}")
            updates.append((selected, cache, times))
        # Validate the whole relevant message before committing either field.
        for selected, cache, times in updates:
            cache.update(selected)
            times.update({name: now for name in selected})
        return bool(updates)

    def snapshot(self):
        if any(name not in self.position or name not in self.velocity for name in self.names):
            return None, None, 0.0
        return (tuple(self.position[name] for name in self.names),
                tuple(self.velocity[name] for name in self.names),
                min(min(self.position_at.values()), min(self.velocity_at.values())))

    def fresh_positions(self, now, timeout):
        """Position hold needs fresh positions even if velocity feedback expired."""
        if any(name not in self.position_at or now - self.position_at[name] >= timeout
               for name in self.names):
            return None
        return tuple(self.position[name] for name in self.names)

    def issues(self, now, timeout):
        issues = []
        for label, times in (("position", self.position_at), ("velocity", self.velocity_at)):
            missing = [name for name in self.names if name not in times]
            stale = [f"{name} ({now - times[name]:.3f}s old)" for name in self.names
                     if name in times and now - times[name] >= timeout]
            if missing:
                issues.append(f"missing {label}: {', '.join(missing)}")
            if stale:
                issues.append(f"stale {label} (limit {timeout:.3f}s): {', '.join(stale)}")
        return issues
