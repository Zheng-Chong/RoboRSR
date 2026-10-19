"""Self-check of the platform scanners, store and file server.

Usage:
    python3 src/real2sim_platform/test_server.py    # or pytest
"""

import http.client
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading

sys.path.insert(0, str(Path(__file__).resolve().parent))
import server


def test_config_objects():
    objects = server.config_objects(server.ROOT / "task/RoboDojo/config/g1d_pour_beans_cup_to_cup.yml")
    assert objects == {"Rigid/plastic_cup/00000": ["source_cup", "target_cup"], "Fluid/wuliangye/00000": ["beans"]}


def test_repo_scan():
    scenes = server.scan_scenes()
    assert {"real_scene1_office", "real_scene2_counter", "real_scene3_lab"} <= set(scenes)
    assert scenes["real_scene3_lab"]["configs"] == ["g1d_lab"] and scenes["real_scene3_lab"]["checks_total"] > 0
    tasks = server.scan_tasks(Path(tempfile.mkdtemp()))
    assert tasks["g1d_stack_blocks"]["step_lim"] == 660
    custom = {k: t for k, t in tasks.items() if t["dimension"] == "custom"}
    objects = server.scan_objects(Path(tempfile.mkdtemp()), custom, ["Rigid/../00001"])
    assert objects["Rigid/measuring_cup/00001"]["generator"] == "scripts/objects/make_plastic_cups.py"
    assert objects["Rigid/measuring_cup/00001"]["labels"] == ["source_cup"]
    assert not any(".." in oid for oid in objects)


def make_run(root, cams=("head", "left_wrist"), tag="success"):
    run = root / "RoboDojo/g1d_stack_blocks/scripted_demo/g1d_lab/0_scripted/2026-10-19_10-00-00"
    run.mkdir(parents=True)
    (run / "_result.json").write_text(json.dumps({"eval_time": 1, "details": {"0": {"layout_id": 3, "success": True}}}))
    for cam in cams:
        (run / f"episode_0000000_cam_{cam}_{tag}.mp4").write_bytes(b"0123456789")
    return run


def test_episodes_and_store():
    tmp = Path(tempfile.mkdtemp())
    run = make_run(tmp / "eval_result", tag="fail")
    found = server.scan_episodes([tmp / "eval_result"])
    ep = found[f"{run}#0"]
    assert (ep["task"], ep["policy"], ep["config"], ep["seed"], ep["layout_id"]) == (
        "g1d_stack_blocks",
        "scripted_demo",
        "g1d_lab",
        0,
        3,
    )
    assert ep["missing_cameras"] == ["right_wrist"] and ep["video_tags"] == ["fail"]

    store = server.Store(tmp / "db.sqlite3")
    store.sync("episode", found)
    review = {"qa": "reject", "tags": ["够不到"], "auto": {"x": 1}, "_by": "forged"}
    store.update("episode", f"{run}#0", review, by="张三")
    store.sync("episode", found)  # rescan keeps the review
    (doc,) = store.list("episode")
    assert doc["qa"] == "reject" and doc["_by"] == "张三" and doc["auto"]["task"] == "g1d_stack_blocks"
    store.sync("episode", {})  # run deleted: the reviewed doc stays, flagged
    assert store.list("episode")[0]["auto"]["missing"]
    store.sync("scene", {"a": {}})
    store.sync("scene", {})  # nothing entered by hand: dropped
    assert store.list("scene") == []


def test_http():
    tmp = Path(tempfile.mkdtemp())
    run = make_run(tmp)
    (tmp / "secret.txt").write_text("x")
    store = server.Store(tmp / "db.sqlite3")
    store.set_user("alice", "password1", "张三")
    server.Handler.store, server.Handler.scan_args, server.Handler.roots = store, (tmp, [tmp]), [tmp]
    server.Handler.failures, server.FAIL_DELAY = {}, 0
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    session = {}

    def request(method, path, body=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", httpd.server_port)
        conn.request(method, path, body, {**session, **(headers or {})})
        resp = conn.getresponse()
        return resp.status, resp.read(), resp

    def login(username, password):
        status, body, resp = request("POST", "/api/login", json.dumps({"username": username, "password": password}))
        return status, json.loads(body), (resp.getheader("Set-Cookie") or "").split(";")[0]

    try:
        assert b"Real2Sim" in request("GET", "/")[1]  # the page itself is public, its data is not
        assert request("GET", "/api/info")[0] == 401 and request("GET", "/api/me")[0] == 401
        assert login("alice", "wrong-pass")[0] == 401 and login("nobody", "password1")[0] == 401
        status, me, cookie = login("alice", "password1")
        assert status == 200 and me == {"username": "alice", "name": "张三"} and cookie.startswith("r2s_session=")
        session["Cookie"] = cookie
        assert json.loads(request("GET", "/api/me")[1])["name"] == "张三"

        video = run / "episode_0000000_cam_head_success.mp4"
        status, body, resp = request("GET", f"/file?path={video}", headers={"Range": "bytes=2-5"})
        assert (status, body, resp.getheader("Content-Range")) == (206, b"2345", "bytes 2-5/10")
        assert request("GET", f"/file?path={tmp / 'secret.txt'}")[0] == 404
        assert request("GET", "/file?path=/etc/hosts")[0] == 404
        assert request("PUT", "/api/items?kind=object&id=Rigid/../00001", b"{}")[0] == 400
        forged = json.dumps({"target": 50, "_by": "x"})
        status, body, _ = request("PUT", "/api/items?kind=task&id=g1d_stack_blocks", forged)
        assert status == 200 and json.loads(body)["target"] == 50 and json.loads(body)["_by"] == "张三"
        assert json.loads(request("POST", "/api/scan")[1])["episode"] == 1
        assert json.loads(request("GET", "/api/items?kind=task")[1])
        server.OBJECTS_DIR = tmp / "objects"
        status, body, _ = request("POST", "/api/photo?id=Rigid/cup/00000&name=../a%20b.JPG", b"jpg")
        photo = json.loads(body)["path"]
        assert status == 200 and photo == str(tmp / "objects/Rigid/cup/00000/reference/a_b.jpg")
        assert request("POST", "/api/photo?id=Rigid/cup/00000&name=x.exe", b"x")[0] == 400
        assert server.scan_objects(tmp, {}, ["Rigid/cup/00000"])["Rigid/cup/00000"]["photos"] == [photo]
        assert request("DELETE", f"/api/photo?path={tmp / 'secret.txt'}")[0] == 400
        assert request("DELETE", f"/api/photo?path={photo}")[0] == 200 and not Path(photo).exists()

        other = login("alice", "password1")[2]  # a second browser
        change = json.dumps({"old": "password1", "new": "password2"})
        assert request("POST", "/api/password", json.dumps({"old": "nope", "new": "password2"}))[0] == 400
        assert request("POST", "/api/password", json.dumps({"old": "password1", "new": "short"}))[0] == 400
        assert request("POST", "/api/password", change)[0] == 200
        assert request("GET", "/api/me")[0] == 200  # this session stays, the other one ends
        assert request("GET", "/api/me", headers={"Cookie": other})[0] == 401
        assert request("POST", "/api/logout")[0] == 200 and request("GET", "/api/me")[0] == 401
        assert [login("alice", "bad-pass")[0] for _ in range(5)] == [401] * 5
        assert login("alice", "password2")[0] == 429  # locked out after 5 failures, even with the right password
    finally:
        httpd.shutdown()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok {name}")
