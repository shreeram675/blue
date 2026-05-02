"""
Firebase Realtime Database publisher for robot command queues.

Set FIREBASE_CREDENTIALS (path to service-account JSON) and
FIREBASE_DATABASE_URL in the environment (or a .env file next to this file)
to enable. When credentials are missing the publisher silently no-ops so
the Flask server works without Firebase configured.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


def _load_local_env() -> None:
    for name in (".env.local", ".env"):
        env_file = Path(__file__).parent / name
        if not env_file.exists():
            continue
        for raw in env_file.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            os.environ.setdefault(key.strip(), value.strip())


class FirebaseQueuePublisher:
    def __init__(self) -> None:
        self.enabled = False
        self.error: Optional[str] = None
        self._db = None
        self._initialize()

    def _initialize(self) -> None:
        _load_local_env()
        credentials_path = os.getenv("FIREBASE_CREDENTIALS")
        database_url = os.getenv("FIREBASE_DATABASE_URL")
        if not credentials_path or not database_url:
            self.error = "Firebase credentials not configured."
            return

        try:
            import firebase_admin
            from firebase_admin import credentials, db

            if not firebase_admin._apps:
                firebase_admin.initialize_app(
                    credentials.Certificate(credentials_path),
                    {"databaseURL": database_url},
                )

            self._db = db
            self.enabled = True
            self.error = None
        except Exception as exc:
            self.error = str(exc)
            self.enabled = False

    def publish_queue(self, robot_id: str, commands: list[str]) -> None:
        if not self.enabled or self._db is None:
            return
        # ⚠️  Keys MUST start with a non-numeric prefix (e.g. "cmd_1").
        # Firebase converts objects with sequential integer keys 0..N into
        # JSON arrays, which the ESP32 firmware receives as dataType="array"
        # and rejects. The "cmd_" prefix keeps it a JSON object.
        payload = {
            f"cmd_{i + 1}": {
                "command":  cmd,
                "status":   "PENDING",
                "sequence": i + 1,
            }
            for i, cmd in enumerate(commands)
        }
        self._db.reference(f"/robots/{robot_id}/queue").set(payload)
        self._db.reference(f"/robots/{robot_id}/meta").set(
            {"active_sequence": 1 if commands else None}
        )
        print(f"   Firebase: published {len(commands)} command(s) → {list(payload.keys())}")

    def cancel_queue(self, robot_id: str) -> None:
        if not self.enabled or self._db is None:
            return
        self._db.reference(f"/robots/{robot_id}/queue").set({})

    def publish_status(self, robot_id: str, pose: dict,
                       current_command: Optional[str] = None) -> None:
        if not self.enabled or self._db is None:
            return
        self._db.reference(f"/robots/{robot_id}/status").update({
            "pose":            pose,
            "current_command": current_command,
        })


firebase_queue = FirebaseQueuePublisher()