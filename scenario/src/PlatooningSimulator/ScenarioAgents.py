"""Scenario-level decision interfaces for scripted and future AI agents."""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, Iterable, List, Optional


class PlatoonCommandType(Enum):
    NONE = auto()
    MERGE_TAIL_TO = auto()
    SPLIT_TAIL = auto()


@dataclass(frozen=True)
class PlatoonCommand:
    kind: PlatoonCommandType = PlatoonCommandType.NONE
    target_platoon_id: Optional[str] = None
    reason: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PlatoonSnapshot:
    platoon_id: str
    size: int
    lead_speed_kmh: float
    tail_speed_kmh: float
    tail_gap_m: Optional[float]
    road_id: Optional[int]
    lane_id: Optional[int]
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class WorldSnapshot:
    time_s: float
    phase: str
    platoon_sizes: Dict[str, int]
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class AgentMessage:
    kind: str
    sender_id: str
    recipient_id: Optional[str] = None
    payload: Dict[str, Any] = field(default_factory=dict)


class BasePlatoonAgent:
    """Minimal per-platoon decision interface for future AI integration."""

    def reset(self):
        """Reset any internal state before a new scenario run."""

    def emit_messages(self, snapshot, world_state, inbox=()):
        return []

    def decide(self, snapshot, world_state, inbox=()):
        raise NotImplementedError


class PassivePlatoonAgent(BasePlatoonAgent):
    """Agent that never issues any command."""

    def decide(self, snapshot, world_state, inbox=()):
        return PlatoonCommand()


class ScriptedMergeAgent(BasePlatoonAgent):
    """Issue a merge-tail command when a scripted approach window is satisfied."""

    def __init__(
        self,
        target_platoon_id,
        min_trigger_time_s=0.0,
        required_phase="CRUISE",
        offset_metadata_key=None,
        straight_metadata_key=None,
        trigger_offset_window_m=(-35.0, 5.0),
    ):
        self.target_platoon_id = target_platoon_id
        self.min_trigger_time_s = float(min_trigger_time_s)
        self.required_phase = required_phase
        self.offset_metadata_key = offset_metadata_key
        self.straight_metadata_key = straight_metadata_key
        self.trigger_offset_window_m = tuple(trigger_offset_window_m)

    def reset(self):
        return None

    def decide(self, snapshot, world_state, inbox=()):
        if snapshot.size < 3:
            return PlatoonCommand()

        if self.required_phase is not None and world_state.phase != self.required_phase:
            return PlatoonCommand()

        if world_state.time_s < self.min_trigger_time_s:
            return PlatoonCommand()

        if self.straight_metadata_key is not None:
            if not world_state.metadata.get(self.straight_metadata_key, False):
                return PlatoonCommand()

        if self.offset_metadata_key is not None:
            offset = world_state.metadata.get(self.offset_metadata_key)
            if offset is None:
                return PlatoonCommand()

            min_offset, max_offset = self.trigger_offset_window_m
            if not (min_offset <= offset <= max_offset):
                return PlatoonCommand()

        return PlatoonCommand(
            kind=PlatoonCommandType.MERGE_TAIL_TO,
            target_platoon_id=self.target_platoon_id,
            reason=f"scripted trigger at {world_state.time_s:.1f}s",
        )


class DestinationNegotiationAgent(BasePlatoonAgent):
    """Per-platoon agent that negotiates a tail transfer using route compatibility."""

    def __init__(self, platoon_destination, member_destinations):
        self.platoon_destination = platoon_destination
        self.member_destinations = list(member_destinations)
        self._proposal_targets = set()
        self._accepted_peers = set()
        self._commanded_targets = set()
        self._announced_accepts = set()

    def reset(self):
        self._proposal_targets.clear()
        self._accepted_peers.clear()
        self._commanded_targets.clear()
        self._announced_accepts.clear()

    def _tail_destination(self, snapshot):
        idx = min(max(snapshot.size - 1, 0), len(self.member_destinations) - 1)
        return self.member_destinations[idx]

    def emit_messages(self, snapshot, world_state, inbox=()):
        if snapshot.size < 1:
            return []

        own_id = snapshot.platoon_id
        tail_destination = self._tail_destination(snapshot)
        messages = [
            AgentMessage(
                kind="STATUS",
                sender_id=own_id,
                payload={
                    "platoon_destination": self.platoon_destination,
                    "tail_destination": tail_destination,
                    "size": snapshot.size,
                },
            )
        ]

        for msg in inbox:
            if msg.kind == "STATUS":
                peer_destination = msg.payload.get("platoon_destination")
                if (
                    snapshot.size >= 3
                    and tail_destination == peer_destination
                    and self.platoon_destination != peer_destination
                    and msg.sender_id not in self._proposal_targets
                ):
                    self._proposal_targets.add(msg.sender_id)
                    messages.append(
                        AgentMessage(
                            kind="TAIL_TRANSFER_REQUEST",
                            sender_id=own_id,
                            recipient_id=msg.sender_id,
                            payload={
                                "tail_destination": tail_destination,
                                "platoon_destination": self.platoon_destination,
                            },
                        )
                    )

            elif msg.kind == "TAIL_TRANSFER_REQUEST":
                requested_destination = msg.payload.get("tail_destination")
                if requested_destination == self.platoon_destination and msg.sender_id not in self._announced_accepts:
                    self._announced_accepts.add(msg.sender_id)
                    messages.append(
                        AgentMessage(
                            kind="TAIL_TRANSFER_ACCEPT",
                            sender_id=own_id,
                            recipient_id=msg.sender_id,
                            payload={
                                "platoon_destination": self.platoon_destination,
                                "accepted_tail_destination": requested_destination,
                            },
                        )
                    )

            elif msg.kind == "TAIL_TRANSFER_ACCEPT":
                self._accepted_peers.add(msg.sender_id)

        return messages

    def decide(self, snapshot, world_state, inbox=()):
        if snapshot.size < 3 or world_state.phase != "APPROACH":
            return PlatoonCommand()

        tail_destination = self._tail_destination(snapshot)
        ready_pairs = world_state.metadata.get("merge_ready_pairs", {})

        for msg in inbox:
            if msg.kind != "TAIL_TRANSFER_ACCEPT":
                continue

            peer_id = msg.sender_id
            pair_key = f"{snapshot.platoon_id}->{peer_id}"
            if peer_id in self._commanded_targets:
                continue
            if not ready_pairs.get(pair_key, False):
                continue
            if tail_destination != msg.payload.get("accepted_tail_destination"):
                continue

            self._commanded_targets.add(peer_id)
            return PlatoonCommand(
                kind=PlatoonCommandType.MERGE_TAIL_TO,
                target_platoon_id=peer_id,
                reason=(
                    f"agent negotiation: tail destination {tail_destination} matches "
                    f"{peer_id} platoon destination"
                ),
                metadata={"tail_destination": tail_destination},
            )

        return PlatoonCommand()


__all__ = [
    "AgentMessage",
    "BasePlatoonAgent",
    "DestinationNegotiationAgent",
    "PassivePlatoonAgent",
    "PlatoonCommand",
    "PlatoonCommandType",
    "PlatoonSnapshot",
    "ScriptedMergeAgent",
    "WorldSnapshot",
]
