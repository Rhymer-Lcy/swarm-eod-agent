"""Team message protocol.

Every drone broadcasts one message per step (``define_message_for_all``);
the framework delivers it, one step later, to teammates within the
communication radius. Messages are plain dictionaries of immutable values
(numbers, strings, tuples, ``bytes``) tagged with a protocol version. A
receiver copies what it needs into its own state and never keeps a reference
to another drone's objects, so sharing behaves like a value transfer even
though the simulator passes Python objects in-process.

Common fields: ``v`` protocol tag, ``id`` sender, ``t`` sender step, ``p``
estimated position. Role-specific fields are documented in the brains.
Invalid or foreign messages are ignored.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

PROTOCOL = "eod1"


def make_message(sender: int, step: int, position: Tuple[float, float],
                 **fields: Any) -> Dict[str, Any]:
    msg = {"v": PROTOCOL, "id": int(sender), "t": int(step),
           "p": (round(float(position[0]), 1), round(float(position[1]), 1))}
    msg.update(fields)
    return msg


def valid_messages(messages: Iterable[Any], own_id: int) -> List[Dict[str, Any]]:
    out = []
    for msg in messages:
        if not isinstance(msg, dict) or msg.get("v") != PROTOCOL:
            continue
        sender = msg.get("id")
        if not isinstance(sender, int) or sender == own_id:
            continue
        pos = msg.get("p")
        if not (isinstance(pos, tuple) and len(pos) == 2):
            continue
        out.append(msg)
    return out


@dataclass
class TeammateState:
    x: float
    y: float
    step: int           # receiver step at which the message arrived
    sender_step: int
    carrying: bool = False
    target: Optional[Tuple[float, float]] = None
    state: str = ""


class TeamTable:
    """Last known position and intent of each teammate."""

    def __init__(self) -> None:
        self.members: Dict[int, TeammateState] = {}

    def update(self, msg: Dict[str, Any], now: int) -> None:
        sender = msg["id"]
        prev = self.members.get(sender)
        if prev is not None and prev.sender_step > msg["t"]:
            return
        target = msg.get("tg")
        self.members[sender] = TeammateState(
            x=float(msg["p"][0]), y=float(msg["p"][1]), step=now,
            sender_step=int(msg["t"]), carrying=bool(msg.get("c", False)),
            target=(float(target[0]), float(target[1])) if isinstance(target, tuple) else None,
            state=str(msg.get("s", "")))

    def recent(self, now: int, max_age: int = 5) -> List[Tuple[int, TeammateState]]:
        return [(k, m) for k, m in self.members.items() if now - m.step <= max_age]

    def positions(self, now: int, max_age: int = 5) -> List[Tuple[float, float]]:
        return [(m.x, m.y) for _, m in self.recent(now, max_age)]

    def targets(self, now: int, max_age: int = 60) -> List[Tuple[int, Tuple[float, float]]]:
        return [(k, m.target) for k, m in self.members.items()
                if m.target is not None and now - m.step <= max_age]
