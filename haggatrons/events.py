"""Timestamped event bus shared by the fleet, mission, HTTP API, and run logs."""

from __future__ import annotations

import itertools
import json
import queue
import threading
import time
from collections import deque
from pathlib import Path


class EventBus:
    def __init__(self, history: int = 400) -> None:
        self._subscribers: list[queue.Queue] = []
        self._history: deque[dict] = deque(maxlen=history)
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self._log: Path | None = None

    def log_to(self, path: Path | None) -> None:
        with self._lock:
            self._log = path
            if path:
                path.parent.mkdir(parents=True, exist_ok=True)

    def publish(self, kind: str, /, **data) -> dict:
        if {"id", "ts", "kind"} & data.keys():
            raise ValueError("event data may not override id, ts, or kind")
        event = {"id": next(self._ids), "ts": time.time(), "kind": kind, **data}
        with self._lock:
            self._history.append(event)
            subscribers = list(self._subscribers)
            log = self._log
        if log and not kind.startswith("telemetry"):
            with log.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, default=str) + "\n")
        for subscriber in subscribers:
            try:
                subscriber.put_nowait(event)
            except queue.Full:
                pass  # slow consumers drop events rather than stall robots
        return event

    def subscribe(self, replay: bool = True) -> queue.Queue:
        subscriber: queue.Queue = queue.Queue(maxsize=2000)
        with self._lock:
            if replay:
                for event in self._history:
                    if not event["kind"].startswith("telemetry"):
                        subscriber.put_nowait(event)
            self._subscribers.append(subscriber)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue) -> None:
        with self._lock:
            if subscriber in self._subscribers:
                self._subscribers.remove(subscriber)

    def recent(self, limit: int = 200) -> list[dict]:
        with self._lock:
            return [e for e in self._history if not e["kind"].startswith("telemetry")][-limit:]
