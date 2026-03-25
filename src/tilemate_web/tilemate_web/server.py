#!/usr/bin/env python3
import asyncio
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
import uvicorn

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
DATA_DIR.mkdir(exist_ok=True)

LATEST_PATH = DATA_DIR / "latest.json"
HISTORY_DIR = DATA_DIR / "history"
HISTORY_DIR.mkdir(exist_ok=True)

app = FastAPI(title="Wall Tile Inspection Web")

# SSE 구독자별 큐
subscribers: list[asyncio.Queue] = []


@app.get("/")
async def index():
    return FileResponse(str(BASE_DIR / "report_page.html"))


@app.get("/api/inspect/latest")
async def get_latest():
    if not LATEST_PATH.exists():
        return JSONResponse(
            {
                "success": False,
                "message": "no inspection result yet",
                "frame_id": "-",
                "timestamp_sec": 0,
                "wall": None,
                "tiles": [],
            },
            status_code=200,
        )

    with open(LATEST_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)

    return JSONResponse(data)


@app.get("/api/inspect/events")
async def inspect_events():
    """
    브라우저가 EventSource로 접속하는 SSE 엔드포인트
    """
    queue: asyncio.Queue = asyncio.Queue()
    subscribers.append(queue)

    async def event_generator():
        try:
            yield "event: connected\ndata: {\"success\": true}\n\n"

            while True:
                msg = await queue.get()
                event_name = msg.get("type", "message")
                yield f"event: {event_name}\ndata: {json.dumps(msg, ensure_ascii=False)}\n\n"
        except asyncio.CancelledError:
            pass
        finally:
            if queue in subscribers:
                subscribers.remove(queue)

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",  # nginx buffering 방지용
        },
    )


@app.post("/api/inspect/result")
async def post_inspect_result(request: Request):
    payload: Dict[str, Any] = await request.json()

    now = datetime.now()
    ts = now.strftime("%Y%m%d_%H%M%S_%f")
    history_path = HISTORY_DIR / f"inspect_{ts}.json"

    with open(LATEST_PATH, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    with open(history_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    event_data = {
        "type": "inspect_result",
        "timestamp": ts,
        "payload": payload,
    }

    for q in subscribers[:]:
        try:
            q.put_nowait(event_data)
        except Exception:
            subscribers.remove(q)

    return {
        "success": True,
        "message": "inspection result saved",
        "latest_path": str(LATEST_PATH),
        "history_path": str(history_path),
    }


if __name__ == "__main__":
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=True)