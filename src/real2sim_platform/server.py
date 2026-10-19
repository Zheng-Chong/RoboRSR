"""Real2Sim data-collection platform: scene reconstruction, object assets, task collection and data QA in one web page.

Stdlib only (any python3 >= 3.10, no Isaac). On start, and on "从项目同步" in the page, it scans:
    real2sim/scenes/<dir>/                reference photos, camera fit, USD, check_report.json, previews
    task/RoboDojo/config/<task>.yml       tasks (scripts/internal/task_inventory.py), objects of the G1-D tasks
    <assets>/Object/RoboDojo/<id>/        metadata.json / description.json of those objects
    <assets>/Eval_Layout/RoboDojo/<cfg>/  eval layouts per task and env config
    <eval-root>/**/_result.json           collected episodes and their camera videos (src/eval_client/eval_env.py)
Scan results go to each record's "auto"; what is entered in the page (measurements, check verdicts, QA) sits next to it
and survives rescans. Records live in one SQLite file (--db). With REAL2SIM_PLATFORM_AUTH=user:password set, every
request needs that HTTP Basic login (set it whenever the port is reachable from outside).

Usage:
    python3 src/real2sim_platform/server.py         # http://127.0.0.1:8765
    REAL2SIM_PLATFORM_AUTH=user:password python3 src/real2sim_platform/server.py --assets /path/to/shared/Assets \
        --eval-root eval_result --eval-root /data2/tmp/robodojo/agent-remote-runs
"""

import argparse
import base64
from collections import Counter
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import re
import sqlite3
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT / "scripts" / "internal"), str(ROOT / "scripts" / "g1d")]
import build_asset  # SCENES: env config -> real2sim scene
import task_inventory

KINDS = ("scene", "object", "task", "episode")
OBJECT_ID = re.compile(r"\w+/\w[\w-]*/\d+")  # <type>/<category>/<index>, e.g. Rigid/measuring_cup/00001
EXPECTED_CAMERAS = {"head", "left_wrist", "right_wrist"}  # same acceptance as scripts/internal/summarize_result.py
VIDEO_RE = re.compile(r"episode_(\d+)_cam_(.+)_(success|fail)\.mp4")
SKIP_DIRS = {".git", "_stream", "Assets", "third_party", "XPolicyLab", "__pycache__"}
SERVED_SUFFIXES = {".jpg", ".jpeg", ".png", ".mp4", ".webm", ".md", ".json"}


def load_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


class Store:
    """JSON documents keyed by (kind, id). Scans own doc["auto"]; every other key is user input."""

    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute("CREATE TABLE IF NOT EXISTS items (kind TEXT, id TEXT, doc TEXT, PRIMARY KEY (kind, id))")
        self.lock = threading.Lock()

    def _get(self, kind, id_):
        row = self.db.execute("SELECT doc FROM items WHERE kind = ? AND id = ?", (kind, id_)).fetchone()
        return json.loads(row[0]) if row else None

    def _put(self, kind, doc):
        self.db.execute("INSERT OR REPLACE INTO items VALUES (?, ?, ?)", (kind, doc["id"], json.dumps(doc)))

    def list(self, kind):
        with self.lock:
            rows = self.db.execute("SELECT doc FROM items WHERE kind = ?", (kind,)).fetchall()
        return [json.loads(row[0]) for row in rows]

    def update(self, kind, id_, fields):
        with self.lock, self.db:
            doc = self._get(kind, id_) or {"id": id_, "auto": {}}
            doc["_by"] = str(fields.pop("_by", ""))
            doc.update({k: v for k, v in fields.items() if k not in ("id", "auto")})
            doc["_at"] = time.time()
            self._put(kind, doc)
        return doc

    def delete(self, kind, id_):
        with self.lock, self.db:
            self.db.execute("DELETE FROM items WHERE kind = ? AND id = ?", (kind, id_))

    def sync(self, kind, found):
        """Set "auto" of the scanned docs. Docs no longer found are dropped, or flagged missing if they hold input."""
        with self.lock, self.db:
            for (doc,) in self.db.execute("SELECT doc FROM items WHERE kind = ?", (kind,)).fetchall():
                doc = json.loads(doc)
                if doc["id"] in found:
                    continue
                if set(doc) <= {"id", "auto"}:
                    self.db.execute("DELETE FROM items WHERE kind = ? AND id = ?", (kind, doc["id"]))
                else:
                    doc["auto"]["missing"] = True
                    self._put(kind, doc)
            for id_, auto in found.items():
                doc = self._get(kind, id_) or {"id": id_}
                doc["auto"] = auto
                self._put(kind, doc)


def scan_scenes():
    configs = {}
    for config, (scene_dir, *_) in build_asset.SCENES.items():
        configs.setdefault(scene_dir, []).append(config)
    found = {}
    for d in sorted(p for p in build_asset.SCENES_DIR.iterdir() if p.is_dir()):
        table = (load_json(d / "usd" / "build_params.json") or {}).get("table", {})
        checks = (load_json(d / "check_report.json") or {}).get("checks", [])
        views = [v for v in (load_json(d / "fit_photo.json") or {}).values() if isinstance(v, dict) and "point_px" in v]
        residuals = [px for v in views for key in ("point_px", "line_px") for px in v.get(key, {}).values()]
        rooms = sorted(p.name for p in d.glob("*.usd"))
        found[d.name] = {
            "path": str(d),
            "configs": configs.get(d.name, []),
            "rooms": rooms,
            "built": bool(rooms) and (d / "usd" / "scene.usda").exists(),
            "scripts": sorted(p.name for p in d.glob("*.py")),
            "docs": sorted(str(p) for p in d.glob("*.md")),
            "photos": sorted(str(p) for p in (d / "reference").glob("*.jpg")),
            "previews": sorted(str(p) for p in (d / "preview").glob("*.jpg")),
            "top_z": table.get("top_z"),
            "size": [table.get("length"), table.get("width")],
            "checks_total": len(checks),
            "checks_failed": [c.get("name") for c in checks if not c.get("ok")],
            # real_scene1 has SfM cameras from a video instead of a photo fit
            "fit_views": len(views) or len(load_json(d / "reference" / "cameras_scene.json") or {}),
            "fit_max_px": max(residuals) if residuals else None,
        }
    return found


def config_objects(path):
    """{asset id: labels} of a task yml's Rigid/Fluid groups, e.g. {"Rigid/plastic_cup/00000": ["source_cup"]}."""
    # ponytail: line reader for the flat task-config layout (PyYAML only exists in the sim env); yaml.safe_load if
    # configs ever use nested flow lists or anchors
    objects, section, group, category = {}, None, [], None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#")[0].rstrip()
        if m := re.match(r"(\w+):", line):
            section = m[1]
        elif section not in ("Rigid", "Fluid"):
            continue
        elif re.match(r"\s{2}-", line):
            group = []
        elif m := re.search(r"\bname:\s*\"?([\w-]+)", line):
            category = m[1]
        elif m := re.search(r"\bindex:\s*\[(.*)\]", line):
            for index in re.findall(r"\d+", m[1]):
                group.append(f"{section}/{category}/{int(index):05d}")
                objects.setdefault(group[-1], [])
        elif m := re.search(r"\blabel:\s*\[(.*)\]", line):
            for oid in group:
                objects[oid] += [label for label in re.findall(r"[\w-]+", m[1]) if label not in objects[oid]]
    return objects


def scan_tasks(assets):
    layout_root = assets / "Eval_Layout" / "RoboDojo"
    layouts = Counter()
    for f in layout_root.glob("*/**/*.json"):
        if m := re.fullmatch(r"(.+)_\d+", f.stem):
            layouts[m[1], f.relative_to(layout_root).parts[0]] += 1
    found = {}
    for t in task_inventory.build_inventory()["tasks"]:
        step = re.search(r"self\.step_lim\s*=\s*(\d+)", (ROOT / t["module"]).read_text(encoding="utf-8"))
        found[t["name"]] = {
            "dimension": t["dimension"],
            "runnable": t["runnable"],
            "module": t["module"],
            "config": t["config"],
            "step_lim": int(step[1]) if step else None,
            "objects": config_objects(ROOT / t["config"]) if t["config"] and t["dimension"] == "custom" else {},
            "layouts": {cfg: n for (task, cfg), n in sorted(layouts.items()) if task == t["name"]},
        }
    return found


def scan_objects(assets, tasks, keep):
    """Objects used by `tasks` plus the ids in `keep` (registered by hand)."""
    used = {}
    for name, task in tasks.items():
        for oid, labels in task["objects"].items():
            entry = used.setdefault(oid, {"tasks": [], "labels": []})
            entry["tasks"].append(name)
            entry["labels"] += [label for label in labels if label not in entry["labels"]]
    generators = {p: p.read_text(encoding="utf-8") for p in sorted((ROOT / "scripts" / "objects").glob("make_*.py"))}
    found = {}
    for oid in sorted(set(used) | {k for k in keep if OBJECT_ID.fullmatch(k)}):
        kind, category, index = oid.split("/")
        d = assets / "Object" / "RoboDojo" / oid
        meta = load_json(d / "metadata.json")
        physics = (meta or {}).get("physics", {})
        names = ((load_json(d / "description.json") or {}).get("caption") or {}).get("name") or [None]
        pattern = re.compile(rf"Rigid/{category}\b|\"{category}\"")
        generator = next((p for p, text in generators.items() if pattern.search(text)), None)
        found[oid] = {
            "type": kind,
            "category": category,
            "index": index,
            "dir": str(d),
            **used.get(oid, {"tasks": [], "labels": []}),
            "generator": str(generator.relative_to(ROOT)) if generator else None,
            "in_library": meta is not None,
            "name": names[0],
            "mass": physics.get("mass"),
            "size": physics.get("size"),
            "static_friction": physics.get("static_friction"),
            "dynamic_friction": physics.get("dynamic_friction"),
        }
    return found


def scan_episodes(roots):
    found = {}
    for root in roots:
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [n for n in dirnames if n not in SKIP_DIRS]
            if "_result.json" not in filenames:
                continue
            run = Path(dirpath)
            result = load_json(run / "_result.json") or {}
            videos = {}
            for name in sorted(filenames):
                if m := VIDEO_RE.fullmatch(name):
                    videos.setdefault(int(m[1]), {})[m[2]] = (str(run / name), m[3])
            # eval_result/<benchmark>/<task>/<policy>/<config>/<seed>_<info>/<run_id>/ (src/eval_client/eval_env.py)
            parts = run.relative_to(root).parts
            task, policy, config, seed_info, run_id = parts[-5:] if len(parts) >= 5 else ("", "", "", "", run.name)
            seed = re.match(r"(\d+)_", seed_info)
            for key, detail in (result.get("details") or {}).items():
                if not str(key).isdigit():
                    continue
                cams = videos.get(int(key), {})
                found[f"{run}#{int(key)}"] = {
                    "run": str(run),
                    "index": int(key),
                    "task": task,
                    "policy": policy,
                    "config": config,
                    "seed": int(seed[1]) if seed else None,
                    "run_id": run_id,
                    "layout_id": detail.get("layout_id"),
                    "success": bool(detail.get("success")),
                    "score": detail.get("score"),
                    "eval_time": result.get("eval_time"),
                    "videos": {cam: path for cam, (path, _) in cams.items()},
                    "video_tags": sorted({tag for _, tag in cams.values()}),
                    "missing_cameras": sorted(EXPECTED_CAMERAS - set(cams)),
                    "mtime": (run / "_result.json").stat().st_mtime,
                }
    return found


def scan(store, assets, eval_roots):
    tasks = scan_tasks(assets)
    custom = {name: t for name, t in tasks.items() if t["dimension"] == "custom"}
    keep = [d["id"] for d in store.list("object") if set(d) - {"id", "auto"}]
    found = {
        "scene": scan_scenes(),
        "task": tasks,
        "object": scan_objects(assets, custom, keep),
        "episode": scan_episodes(eval_roots),
    }
    for kind, items in found.items():
        store.sync(kind, items)
    return {kind: len(items) for kind, items in found.items()}


def servable(path, roots):
    p = Path(path).resolve()
    return p.suffix.lower() in SERVED_SUFFIXES and p.is_file() and any(p.is_relative_to(r.resolve()) for r in roots)


class Handler(BaseHTTPRequestHandler):
    store, scan_args, roots, info, auth = None, (), [], {}, None

    def parse_request(self):
        if not super().parse_request():
            return False
        if self.auth and not hmac.compare_digest(self.headers.get("Authorization", "").encode(), self.auth.encode()):
            self.send_response(401)
            self.send_header("WWW-Authenticate", 'Basic realm="Real2Sim platform"')
            self.send_header("Content-Length", "0")
            self.end_headers()
            return False
        return True

    def do_GET(self):
        url = urlparse(self.path)
        query = {k: v[0] for k, v in parse_qs(url.query).items()}
        if url.path == "/":
            self.send_file(HERE / "index.html")
        elif url.path == "/api/info":
            self.send_json(self.info)
        elif url.path == "/api/items" and query.get("kind") in KINDS:
            self.send_json(self.store.list(query["kind"]))
        elif url.path == "/file" and servable(query.get("path", ""), self.roots):
            self.send_file(Path(query["path"]).resolve())
        else:
            self.send_error(404)

    def do_POST(self):
        if urlparse(self.path).path != "/api/scan":
            return self.send_error(404)
        self.send_json(scan(self.store, *self.scan_args))

    def do_PUT(self):
        kind, id_ = self.target()
        try:
            fields = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        except ValueError:
            fields = None
        if not kind or not isinstance(fields, dict) or (kind == "object" and not OBJECT_ID.fullmatch(id_)):
            return self.send_error(400)
        self.send_json(self.store.update(kind, id_, fields))

    def do_DELETE(self):
        kind, id_ = self.target()
        if not kind:
            return self.send_error(400)
        self.store.delete(kind, id_)
        self.send_json({"ok": True})

    def target(self):
        url = urlparse(self.path)
        query = {k: v[0] for k, v in parse_qs(url.query).items()}
        if url.path == "/api/items" and query.get("kind") in KINDS and query.get("id"):
            return query["kind"], query["id"]
        return None, None

    def send_json(self, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path):
        """Serve a file with HTTP Range support, which <video> needs for seeking."""
        size = path.stat().st_size
        start, end = 0, size - 1
        m = re.fullmatch(r"bytes=(\d*)-(\d*)", self.headers.get("Range", ""))
        if m and (m[1] or m[2]):
            start, end = (int(m[1]), int(m[2] or end)) if m[1] else (max(size - int(m[2]), 0), end)
            end = min(end, size - 1)
            if start > end:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
            self.send_response(206)
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        else:
            self.send_response(200)
        kind = "text/plain; charset=utf-8" if path.suffix == ".md" else mimetypes.guess_type(path.name)[0]
        self.send_header("Content-Type", kind or "application/octet-stream")
        self.send_header("Content-Length", str(end - start + 1))
        self.send_header("Accept-Ranges", "bytes")
        self.end_headers()
        try:
            with path.open("rb") as f:
                f.seek(start)
                left = end - start + 1
                while left > 0 and (chunk := f.read(min(left, 1 << 20))):
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass  # the browser dropped a video range request while seeking


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="0.0.0.0 shares it on the LAN (set REAL2SIM_PLATFORM_AUTH)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--db", type=Path, default=ROOT / ".cache" / "real2sim_platform.sqlite3")
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets", help="asset library with Object/, Eval_Layout/")
    parser.add_argument(
        "--eval-root", type=Path, action="append", help="searched for _result.json; repeatable (default eval_result/)"
    )
    args = parser.parse_args()
    assets = args.assets.resolve()
    eval_roots = [p.resolve() for p in args.eval_root or [ROOT / "eval_result"]]
    login = os.environ.get("REAL2SIM_PLATFORM_AUTH")
    if login is not None and ":" not in login:
        parser.error("REAL2SIM_PLATFORM_AUTH must be user:password")
    store = Store(args.db)
    Handler.auth = f"Basic {base64.b64encode(login.encode()).decode()}" if login else None
    Handler.store, Handler.scan_args = store, (assets, eval_roots)
    Handler.roots = [ROOT / "real2sim", ROOT / "output", assets, *eval_roots]
    Handler.info = {
        "root": str(ROOT),
        "db": str(args.db.resolve()),
        "assets": str(assets),
        "assets_ok": (assets / "Object").is_dir(),
        "eval_roots": [str(p) for p in eval_roots],
    }
    print(f"scanned {scan(store, assets, eval_roots)}", flush=True)
    print(f"Real2Sim platform on http://{args.host}:{args.port}", flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
