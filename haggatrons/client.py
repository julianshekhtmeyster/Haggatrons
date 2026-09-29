"""Small HTTP client for the local backend API (used by Flower workers and tests)."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from pathlib import Path

# Written by the running backend (owner-only); a fallback for worker processes
# that do not inherit HAGGATRONS_URL / HAGGATRONS_TOKEN.
BACKEND_FILE = Path(__file__).resolve().parents[1] / "runs" / "backend.json"


class BackendError(RuntimeError):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


class BackendClient:
    def __init__(self, url: str | None = None, token: str | None = None, timeout: float = 30) -> None:
        url = url or os.environ.get("HAGGATRONS_URL", "")
        token = token or os.environ.get("HAGGATRONS_TOKEN", "")
        if (not url or not token) and BACKEND_FILE.exists():
            saved = json.loads(BACKEND_FILE.read_text(encoding="utf-8"))
            url, token = url or saved["url"], token or saved["token"]
        self.url = url.rstrip("/")
        self.token = token
        self.timeout = timeout
        if not self.url or not self.token:
            raise RuntimeError("HAGGATRONS_URL and HAGGATRONS_TOKEN must be set for backend access")

    def _request(self, method: str, path: str, body: dict | None = None, raw: bool = False,
                 timeout: float | None = None):
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.url + path, data=data, method=method,
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
                payload = response.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read(2000).decode("utf-8", "replace")
            try:
                detail = json.loads(detail).get("error", detail)
            except ValueError:
                pass
            raise BackendError(exc.code, detail) from None
        return payload if raw else json.loads(payload or b"{}")

    def get(self, path: str, **kwargs):
        return self._request("GET", path, **kwargs)

    def post(self, path: str, body: dict | None = None, **kwargs):
        return self._request("POST", path, body or {}, **kwargs)
