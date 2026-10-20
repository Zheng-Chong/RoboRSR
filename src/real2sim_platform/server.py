"""Real2Sim data-collection platform: scene reconstruction, object assets, task collection and data QA in one web page.

Stdlib only (any python3 >= 3.10, no Isaac). On start, and on "从项目同步" in the page, it scans:
    real2sim/scenes/<dir>/                reference photos, camera fit, USD, check_report.json, previews
    task/RoboDojo/config/<task>.yml       tasks (scripts/internal/task_inventory.py), objects of the G1-D tasks
    <assets>/Object/RoboDojo/<id>/        metadata.json / description.json of those objects
    <assets>/Eval_Layout/RoboDojo/<cfg>/  eval layouts per task and env config
    <eval-root>/**/_result.json           collected episodes and their camera videos (src/eval_client/eval_env.py)
Scan results go to each record's "auto"; what is entered in the page (measurements, check verdicts, QA) sits next to it
and survives rescans. A registered real object (twin) is also written to real2sim/objects/<id>/spec.json, next to its
photos, as the input for modeling it; the built asset reports its size per measured dimension (by name) in
metadata.json "measurements_mm", which the page checks against the measured values. Records live in one SQLite file
(--db), together with the accounts: everyone signs in, and every change is stamped with the signed-in account's name.
Admins manage accounts in the page (成员管理); the first admin is created on the server with --add-user NAME --admin.

Usage:
    python3 src/real2sim_platform/server.py --add-user alice --name 张三 --admin   # asks for the password
    python3 src/real2sim_platform/server.py         # http://127.0.0.1:8765
    python3 src/real2sim_platform/server.py --assets /path/to/shared/Assets \
        --eval-root eval_result --eval-root /data2/tmp/robodojo/agent-remote-runs
"""

import argparse
from collections import Counter
import getpass
import hashlib
import hmac
from http.cookies import CookieError, SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import re
import secrets
import sqlite3
import sys
import threading
import time
from urllib.parse import parse_qs, unquote, urlparse

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
OBJECTS_DIR = ROOT / "real2sim" / "objects"  # <id>/reference/: real-object photos (git LFS); <id>/preview/: renders
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp"}
SERVED_SUFFIXES = IMAGE_SUFFIXES | {".mp4", ".webm", ".md", ".json"}
MAX_UPLOAD = 25 << 20
# images and videos: fresh for a minute, then shown from cache at once while the ETag is rechecked in the background,
# so a re-rendered preview appears on the next visit; private, because they are only served to signed-in users
FILE_CACHE = "private, max-age=60, stale-while-revalidate=604800"
USERNAME = re.compile(r"[\w.-]{1,32}")
PBKDF2_ROUNDS = 200_000
COOKIE, SESSION_DAYS = "r2s_session", 14
LOCK_AFTER, LOCK_SECONDS, FAIL_DELAY = 5, 300, 0.5  # failed logins per username before a lockout; HTTP only, no TLS


def hash_password(password, salt=None):
    salt = salt or secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac("sha256", password.encode(), salt, PBKDF2_ROUNDS)
    return f"pbkdf2_sha256${PBKDF2_ROUNDS}${salt.hex()}${digest.hex()}"


def verify_password(password, stored):
    _, rounds, salt, digest = stored.split("$")
    got = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(salt), int(rounds))
    return hmac.compare_digest(got.hex(), digest)


DUMMY_HASH = hash_password(secrets.token_hex())  # unknown users cost the same time as wrong passwords


def token_key(token):
    return hashlib.sha256(token.encode()).hexdigest()  # the database only holds hashes of session tokens


def load_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def images(folder):
    return sorted(str(p) for p in Path(folder).glob("*") if p.suffix.lower() in IMAGE_SUFFIXES)


class Store:
    """JSON documents keyed by (kind, id). Scans own doc["auto"]; every other key is user input."""

    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute("CREATE TABLE IF NOT EXISTS items (kind TEXT, id TEXT, doc TEXT, PRIMARY KEY (kind, id))")
        self.db.execute("CREATE TABLE IF NOT EXISTS users (username TEXT PRIMARY KEY, name TEXT, pw TEXT)")
        if "admin" not in [col[1] for col in self.db.execute("PRAGMA table_info(users)")]:
            self.db.execute("ALTER TABLE users ADD COLUMN admin INTEGER NOT NULL DEFAULT 0")
        self.db.execute("CREATE TABLE IF NOT EXISTS sessions (token TEXT PRIMARY KEY, username TEXT, expires REAL)")
        self.db.commit()
        self.lock = threading.Lock()

    def add_user(self, username, password, name=None, admin=False):
        """Create the account; False if the username is taken."""
        with self.lock, self.db:
            row = (username, name or username, hash_password(password), int(admin))
            return self.db.execute("INSERT OR IGNORE INTO users VALUES (?, ?, ?, ?)", row).rowcount == 1

    def edit_user(self, username, name=None, password=None, admin=None):
        """Change the given fields; a new password ends the account's sessions. False if there is no such account."""
        with self.lock, self.db:
            found = self.db.execute(
                "UPDATE users SET name = COALESCE(?, name), pw = COALESCE(?, pw), admin = COALESCE(?, admin) "
                "WHERE username = ?",
                (name, hash_password(password) if password else None, None if admin is None else int(admin), username),
            ).rowcount
            if password:
                self.db.execute("DELETE FROM sessions WHERE username = ?", (username,))
        return found == 1

    def remove_user(self, username):
        with self.lock, self.db:
            self.db.execute("DELETE FROM sessions WHERE username = ?", (username,))
            return self.db.execute("DELETE FROM users WHERE username = ?", (username,)).rowcount

    def users(self):
        with self.lock:
            rows = self.db.execute("SELECT username, name, admin FROM users ORDER BY username").fetchall()
        return [{"username": u, "name": n or u, "admin": bool(a)} for u, n, a in rows]

    def check(self, username, password):
        """{username, name, admin} if the password is right, else None."""
        with self.lock:
            row = self.db.execute("SELECT name, pw, admin FROM users WHERE username = ?", (username,)).fetchone()
        right = verify_password(password, row[1] if row else DUMMY_HASH)
        return {"username": username, "name": row[0] or username, "admin": bool(row[2])} if row and right else None

    def open_session(self, username):
        token, now = secrets.token_urlsafe(32), time.time()
        with self.lock, self.db:
            self.db.execute("DELETE FROM sessions WHERE expires < ?", (now,))
            expires = now + SESSION_DAYS * 86400
            self.db.execute("INSERT INTO sessions VALUES (?, ?, ?)", (token_key(token), username, expires))
        return token

    def session_user(self, token):
        if not token:
            return None
        with self.lock:
            row = self.db.execute(
                "SELECT u.username, u.name, u.admin FROM sessions s JOIN users u USING (username) "
                "WHERE s.token = ? AND s.expires > ?",
                (token_key(token), time.time()),
            ).fetchone()
        return {"username": row[0], "name": row[1] or row[0], "admin": bool(row[2])} if row else None

    def end_session(self, token):
        with self.lock, self.db:
            self.db.execute("DELETE FROM sessions WHERE token = ?", (token_key(token or ""),))

    def close_sessions(self, username, keep=None):
        """End the given account's sessions, all but `keep` (a token) if given."""
        with self.lock, self.db:
            self.db.execute("DELETE FROM sessions WHERE username = ? AND token != ?", (username, token_key(keep or "")))

    def change_password(self, username, password, keep):
        with self.lock, self.db:
            self.db.execute("UPDATE users SET pw = ? WHERE username = ?", (hash_password(password), username))
        self.close_sessions(username, keep)

    def _get(self, kind, id_):
        row = self.db.execute("SELECT doc FROM items WHERE kind = ? AND id = ?", (kind, id_)).fetchone()
        return json.loads(row[0]) if row else None

    def _put(self, kind, doc):
        self.db.execute("INSERT OR REPLACE INTO items VALUES (?, ?, ?)", (kind, doc["id"], json.dumps(doc)))

    def list(self, kind):
        with self.lock:
            rows = self.db.execute("SELECT doc FROM items WHERE kind = ?", (kind,)).fetchall()
        return [json.loads(row[0]) for row in rows]

    def update(self, kind, id_, fields, by):
        with self.lock, self.db:
            doc = self._get(kind, id_) or {"id": id_, "auto": {}}
            doc.update({k: v for k, v in fields.items() if k not in ("id", "auto", "_by", "_at")})
            doc["_by"] = by
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
        # check_object.py --image-dir result, synced with the renders: a built asset, maybe not installed yet
        check = load_json(OBJECTS_DIR / oid / "preview" / "check.json") or {}
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
            "built": meta is not None or bool(check),
            "check": check,
            "mass": physics.get("mass", check.get("mass_kg")),
            "size": physics.get("size")
            or ((meta or {}).get("geometry", {}).get("aligned_bbox") or {}).get("extents")
            or check.get("size_m"),
            "static_friction": physics.get("static_friction"),
            "dynamic_friction": physics.get("dynamic_friction"),
            # model size per registered dimension name
            "measurements": (meta or {}).get("measurements_mm") or check.get("measurements_mm") or {},
            "photos": images(OBJECTS_DIR / oid / "reference"),
            "renders": images(OBJECTS_DIR / oid / "preview") + images(d),
        }
    return found


def object_indices(assets, store):
    """{"Rigid/cup": [0, 1, ...]}: indices taken per <type>/<category> in the library, real2sim/objects or the store."""
    roots = (assets / "Object" / "RoboDojo", OBJECTS_DIR)
    ids = [str(p.relative_to(root)) for root in roots for p in root.glob("*/*/*") if p.is_dir()]
    taken = {}
    for oid in ids + [d["id"] for d in store.list("object")]:
        if OBJECT_ID.fullmatch(oid):
            group, index = oid.rsplit("/", 1)
            taken.setdefault(group, set()).add(int(index))
    return {group: sorted(indices) for group, indices in sorted(taken.items())}


def write_spec(doc):
    """A twin's registration as real2sim/objects/<id>/spec.json, next to its photos: the input for modeling it."""
    path = OBJECTS_DIR / doc["id"] / "spec.json"
    if not doc.get("twin"):
        return path.unlink(missing_ok=True)
    spec = {
        "id": doc["id"],
        "name": doc.get("name") or doc["auto"].get("name"),
        "name_en": doc.get("name_en"),
        "tasks": doc["auto"].get("tasks"),  # from the task configs
        "material": doc.get("material"),
        "color": doc.get("color"),
        "mass_g": doc.get("real_mass"),
        "static_friction": doc.get("real_friction"),
        "dimensions_mm": doc.get("dimensions"),  # free text, e.g. "开口直径 82，底部直径 64，高 135"
        "features": doc.get("features"),
        "requirements": doc.get("requirements"),
        "notes": doc.get("notes"),
        # variants to build next to the twin: {"count", "color"/"material"/"size"/"shape": may vary, "requirements"}
        "generalization": {k: v for k, v in (doc.get("gen") or {}).items() if v not in (None, "", False, 0)},
        "updated_by": doc.get("_by"),
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(doc["_at"])),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps({k: v for k, v in spec.items() if v not in (None, "", [], {})}, ensure_ascii=False, indent=2)
    path.write_text(text + "\n", encoding="utf-8")


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
    keep += [str(p.relative_to(OBJECTS_DIR)) for p in OBJECTS_DIR.glob("*/*/*") if p.is_dir()]  # have photos or renders
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
    store, scan_args, roots, info = None, (), [], {}
    failures = {}  # username -> recent failed login times
    PUBLIC = {"/", "/api/login"}  # the page itself carries no data; it shows the sign-in form until /api/me answers
    protocol_version = "HTTP/1.1"  # keep-alive: a page of images or a video reuses connections through the tunnel
    timeout = 60  # idle keep-alive connections end after a minute

    def parse_request(self):
        if not super().parse_request():
            return False
        # a rejected request's body is never read, so these replies close the connection
        # some browsers reload "/#scenes" as "/%23scenes": send them back to the page with the tab in the fragment
        if tab := re.fullmatch(r"/#([\w-]*)", unquote(urlparse(self.path).path)):
            self.send_response(302)
            self.send_header("Location", f"/#{tab[1]}")
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()
            return False
        self.user = self.store.session_user(self.cookie())
        if self.user or urlparse(self.path).path in self.PUBLIC:
            return True
        self.send_json({"error": "请先登录"}, 401, close=True)
        return False

    def cookie(self):
        try:
            jar = SimpleCookie(self.headers.get("Cookie", ""))
        except CookieError:
            return None
        return jar[COOKIE].value if COOKIE in jar else None

    def body(self):
        try:
            data = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        except ValueError:
            return None
        return data if isinstance(data, dict) else None

    def login(self):
        data, now = self.body() or {}, time.time()
        username, password = str(data.get("username", "")).strip(), str(data.get("password", ""))
        if len(self.failures) > 10_000:
            self.failures.clear()  # ponytail: in-memory lockout, forgets on restart; enough for a team tool
        recent = [t for t in self.failures.get(username, []) if now - t < LOCK_SECONDS]
        if len(recent) >= LOCK_AFTER:
            return self.send_json({"error": f"连续输错 {LOCK_AFTER} 次，请 {LOCK_SECONDS // 60} 分钟后再试"}, 429)
        user = self.store.check(username, password) if USERNAME.fullmatch(username) else None
        if not user:
            self.failures[username] = [*recent, now]
            time.sleep(FAIL_DELAY)
            return self.send_json({"error": "用户名或密码错误"}, 401)
        self.failures.pop(username, None)
        token = self.store.open_session(username)
        self.send_json(user, cookie=f"{COOKIE}={token}; Max-Age={SESSION_DAYS * 86400}")

    def accounts(self, method):
        """Admins only: GET lists, POST creates, PUT ?username= edits, DELETE ?username= removes; replies the list."""

        def fail(message, status=400):
            self.send_json({"error": message}, status)

        if not self.user["admin"]:
            return fail("需要管理员权限", 403)
        if method == "GET":
            return self.send_json(self.store.users())
        data = self.body() if method != "DELETE" else {}
        if data is None:
            return fail("请求格式不对")
        target = parse_qs(urlparse(self.path).query).get("username", [""])[0]
        username = str(data.get("username", "")).strip() if method == "POST" else target
        name = str(data.get("name") or "").strip()[:32] or None
        password = str(data.get("password") or "") or None
        admin = None if data.get("admin") is None else bool(data["admin"])
        if (password and len(password) < 8) or (method == "POST" and not password):
            return fail("密码至少 8 位")
        if method == "POST":
            if not USERNAME.fullmatch(username):
                return fail("用户名为 1–32 位字母、数字或 _ . -")
            if not self.store.add_user(username, password, name, bool(admin)):
                return fail(f"用户名 {username} 已存在", 409)
        elif username == self.user["username"] and (method == "DELETE" or admin is False):
            return fail("不能删除自己，也不能取消自己的管理员权限")
        elif method == "PUT":
            if not self.store.edit_user(username, name, password, admin):
                return fail(f"没有账号 {username}", 404)
        elif not self.store.remove_user(username):
            return fail(f"没有账号 {username}", 404)
        self.send_json(self.store.users())

    def do_GET(self):
        url = urlparse(self.path)
        query = {k: v[0] for k, v in parse_qs(url.query).items()}
        if url.path == "/":
            self.send_file(HERE / "index.html", cache="no-cache")  # revalidated every load, so a deploy shows at once
        elif url.path == "/api/me":
            self.send_json(self.user)
        elif url.path == "/api/users":
            self.accounts("GET")
        elif url.path == "/api/info":
            self.send_json(self.info)
        elif url.path == "/api/object-indices":
            self.send_json(object_indices(self.scan_args[0], self.store))
        elif url.path == "/api/items" and query.get("kind") in KINDS:
            self.send_json(self.store.list(query["kind"]))
        elif url.path == "/file" and servable(query.get("path", ""), self.roots):
            self.send_file(Path(query["path"]).resolve())
        else:
            self.send_error(404)

    def do_POST(self):
        url = urlparse(self.path)
        if url.path == "/api/login":
            return self.login()
        if url.path == "/api/users":
            return self.accounts("POST")
        if url.path == "/api/logout":
            self.store.end_session(self.cookie())
            return self.send_json({"ok": True}, cookie=f"{COOKIE}=; Max-Age=0")
        if url.path == "/api/password":
            data = self.body() or {}
            old, new = str(data.get("old", "")), str(data.get("new", ""))
            if not self.store.check(self.user["username"], old):
                return self.send_json({"error": "当前密码不对"}, 400)
            if len(new) < 8:
                return self.send_json({"error": "新密码至少 8 位"}, 400)
            self.store.change_password(self.user["username"], new, keep=self.cookie())
            return self.send_json({"ok": True})
        if url.path == "/api/scan":
            return self.send_json(scan(self.store, *self.scan_args))
        if url.path != "/api/photo":
            return self.send_error(404)
        query = {k: v[0] for k, v in parse_qs(url.query).items()}
        oid, name = query.get("id", ""), Path(query.get("name", "")).name
        size, suffix = int(self.headers.get("Content-Length", 0)), Path(name).suffix.lower()
        if not OBJECT_ID.fullmatch(oid) or suffix not in IMAGE_SUFFIXES or not 0 < size <= MAX_UPLOAD:
            return self.send_error(400)
        folder = OBJECTS_DIR / oid / "reference"
        folder.mkdir(parents=True, exist_ok=True)
        stem = re.sub(r"[^\w.-]", "_", Path(name).stem)[:60] or "photo"
        path, n = folder / f"{stem}{suffix}", 1
        while path.exists():
            path, n = folder / f"{stem}-{n}{suffix}", n + 1
        path.write_bytes(self.rfile.read(size))
        self.send_json({"path": str(path)})

    def do_PUT(self):
        if urlparse(self.path).path == "/api/users":
            return self.accounts("PUT")
        kind, id_ = self.target()
        fields = self.body()
        if not kind or fields is None or (kind == "object" and not OBJECT_ID.fullmatch(id_)):
            return self.send_error(400)
        doc = self.store.update(kind, id_, fields, by=self.user["name"])
        if kind == "object":
            write_spec(doc)
        self.send_json(doc)

    def do_DELETE(self):
        url = urlparse(self.path)
        if url.path == "/api/users":
            return self.accounts("DELETE")
        if url.path == "/api/photo":
            path = Path(parse_qs(url.query).get("path", [""])[0]).resolve()
            photo = path.parent.name == "reference" and path.suffix.lower() in IMAGE_SUFFIXES
            if not (photo and path.is_file() and path.is_relative_to(OBJECTS_DIR.resolve())):
                return self.send_error(400)
            path.unlink()
            return self.send_json({"ok": True})
        kind, id_ = self.target()
        if not kind:
            return self.send_error(400)
        self.store.delete(kind, id_)
        if kind == "object" and OBJECT_ID.fullmatch(id_):
            (OBJECTS_DIR / id_ / "spec.json").unlink(missing_ok=True)
        self.send_json({"ok": True})

    def target(self):
        url = urlparse(self.path)
        query = {k: v[0] for k, v in parse_qs(url.query).items()}
        if url.path == "/api/items" and query.get("kind") in KINDS and query.get("id"):
            return query["kind"], query["id"]
        return None, None

    def send_json(self, obj, status=200, cookie=None, close=False):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if cookie:  # no Secure flag: the platform is served over plain HTTP
            self.send_header("Set-Cookie", f"{cookie}; Path=/; HttpOnly; SameSite=Lax")
        if close:
            self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path, cache=FILE_CACHE):
        """Serve a file with HTTP Range support, which <video> needs for seeking, and an ETag to revalidate caches."""
        st = path.stat()
        size, etag = st.st_size, f'"{st.st_mtime_ns:x}-{st.st_size:x}"'
        if etag in [tag.strip() for tag in self.headers.get("If-None-Match", "").split(",")]:
            self.send_response(304)
            self.send_header("ETag", etag)
            self.send_header("Cache-Control", cache)
            self.end_headers()
            return
        start, end = 0, size - 1
        m = re.fullmatch(r"bytes=(\d*)-(\d*)", self.headers.get("Range", ""))
        if m and (m[1] or m[2]):
            start, end = (int(m[1]), int(m[2] or end)) if m[1] else (max(size - int(m[2]), 0), end)
            end = min(end, size - 1)
            if start > end:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.send_header("Content-Length", "0")
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
        self.send_header("ETag", etag)
        self.send_header("Last-Modified", self.date_time_string(st.st_mtime))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        try:
            with path.open("rb") as f:
                f.seek(start)
                left = end - start + 1
                while left > 0 and (chunk := f.read(min(left, 1 << 20))):
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True  # the browser dropped a video range request while seeking


def manage_users(parser, store, args):
    if args.list_users:
        for user in store.users():
            print(f"{user['username']}\t{user['name']}" + ("\tadmin" if user["admin"] else ""))
    if args.remove_user:
        print("removed" if store.remove_user(args.remove_user) else f"no account {args.remove_user}")
    if args.add_user:
        if not USERNAME.fullmatch(args.add_user):
            parser.error("username: 1-32 letters, digits, '_', '.', '-'")
        if sys.stdin.isatty():
            password = getpass.getpass("密码：")
            if password != getpass.getpass("再输一次："):
                parser.error("两次输入的密码不一致")
        else:
            password = sys.stdin.readline().rstrip("\n")  # scripted: the password on the first line of stdin
        if len(password) < 8:
            parser.error("密码至少 8 位")
        if not store.add_user(args.add_user, password, args.name, args.admin):
            store.edit_user(args.add_user, args.name, password, args.admin or None)
        print(f"account {args.add_user} ready" + (" (admin)" if args.admin else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="0.0.0.0 shares it on the LAN")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--db", type=Path, default=ROOT / ".cache" / "real2sim_platform.sqlite3")
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets", help="asset library with Object/, Eval_Layout/")
    parser.add_argument(
        "--eval-root", type=Path, action="append", help="searched for _result.json; repeatable (default eval_result/)"
    )
    parser.add_argument("--add-user", metavar="USERNAME", help="create the account or reset its password, then exit")
    parser.add_argument("--name", help="with --add-user: display name stamped on that account's changes")
    parser.add_argument("--admin", action="store_true", help="with --add-user: admins manage accounts in the page")
    parser.add_argument("--remove-user", metavar="USERNAME", help="delete the account, then exit")
    parser.add_argument("--list-users", action="store_true", help="list the accounts, then exit")
    args = parser.parse_args()
    store = Store(args.db)
    if args.add_user or args.remove_user or args.list_users:
        return manage_users(parser, store, args)
    if not any(user["admin"] for user in store.users()):
        print(f"no admin account yet: python3 {Path(__file__).relative_to(ROOT)} --add-user NAME --admin", flush=True)
    assets = args.assets.resolve()
    eval_roots = [p.resolve() for p in args.eval_root or [ROOT / "eval_result"]]
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
