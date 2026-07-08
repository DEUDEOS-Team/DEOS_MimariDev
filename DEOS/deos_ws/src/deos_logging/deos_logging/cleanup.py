from pathlib import Path
from datetime import datetime, timedelta


def cleanup_logs(hours: int = 12):
    log_root = Path("logs")
    if not log_root.exists():
        return

    now = datetime.now()
    limit = timedelta(hours=hours)

    for file in log_root.rglob("*.log"):
        try:
            modified_time = datetime.fromtimestamp(file.stat().st_mtime)
            if (now - modified_time) > limit:
                file.unlink()
        except Exception:
            continue
