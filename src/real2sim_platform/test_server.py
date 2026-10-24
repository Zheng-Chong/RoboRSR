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
    assert all(s["name"] for k, s in scenes.items() if k.startswith("real_scene"))  # Chinese name, scene_spec.json
    tasks = server.scan_tasks(Path(tempfile.mkdtemp()))
    assert tasks["g1d_stack_blocks"]["step_lim"] == 660
    bench = server.scan_benchmark_scenes(tasks)  # env_cfg/scene/*.yml, picked per task by _task.yml scene_config
    assert bench["robodojo_default"]["top_z"] == 0.765 and bench["robodojo_default"]["size"] == [1.4, 1.1]
    default = bench["robodojo_default"]
    assert default["room"] == "Simple_Room_nolight" and "stack_bowls" in default["tasks"]
    conveyor = bench["robodojo_conveyor"]
    assert conveyor["tasks"] == ["match_and_pick_from_conveyor", "pick_from_conveyor_by_image"]
    assert conveyor["top_z"] is None and conveyor["room_random"] is False  # "random: False" is a bool, not a string
    assert not any(t.startswith("g1d_") for s in bench.values() for t in s["tasks"])
    custom = {k: t for k, t in tasks.items() if t["dimension"] == "custom"}
    objects = server.scan_objects(Path(tempfile.mkdtemp()), custom, ["Rigid/../00001"])
    assert objects["Rigid/measuring_cup/00001"]["generator"] == "scripts/objects/make_plastic_cups.py"
    assert objects["Rigid/measuring_cup/00001"]["labels"] == ["source_cup"]
    assert not any(".." in oid for oid in objects)


def test_randomization_profile():
    pools = server.instance_pools("stack_bowls_random")  # the base task's instances train, the variant's are held out
    assert pools["base"] == "stack_bowls" and pools["train"]["bowl"] == [1, 2, 3, 5, 7, 8, 9, 10]
    assert pools["holdout"] == {"bowl": [11, 12, 13, 14, 15]}
    assert server.instance_pools("align_blocks")["holdout"] == {}  # no _random variant
    assert set(pools["new"]) == {"train", "exam"}  # instances of the task's categories no official layout uses
    assets, profiles = Path(tempfile.mkdtemp()), server.rsi_profile.PROFILES_DIR
    for d in (
        "Room/Simple_Room_red",
        "Room/Simple_Room_teal",
        "Material/material_0122",
        "Object/RoboDojo/Clutter/pen/00000",
    ):
        (assets / d).mkdir(parents=True)
    (assets / "Object/RoboDojo/Clutter/pen/00000/metadata.json").write_text("{}")
    (assets / "Background").mkdir()
    (assets / "Background/studio.hdr").write_text("")
    empty = {"rooms": [], "materials": [], "hdrs": [], "distractors": [], "instances": []}
    tiers = {
        "seen": ["material:material_0122"],
        "t0": [],
        "pools": {
            "train": empty | {"rooms": ["Simple_Room_red"], "hdrs": ["studio.hdr"], "distractors": ["clutter:pen/0"]},
            "exam": empty | {"rooms": ["Simple_Room_teal"]},
        },
    }
    server.rsi_profile.PROFILES_DIR = Path(tempfile.mkdtemp())

    def write(profile):
        return server.write_randomization("stack_bowls", profile, assets, tiers)

    try:
        profile = {"position": True, "color": True, "scale": [0.9, 1.1], "rooms": ["Simple_Room_red"]}
        profile |= {"tables": ["material_0122"], "background": {"hdrs": ["studio.hdr"], "intensity": [600, 1400]}}
        profile |= {
            "clutter": {"count": [2, 5]},
            "lights": {"count": [1, 2], "types": ["Sphere"]},
            "camera": {"ori_deg": 2},
        }
        assert write(profile) is None
        path = server.rsi_profile.PROFILES_DIR / "stack_bowls/randomization.json"
        assert json.loads(path.read_text()) == profile
        assert "不能用" in write({"rooms": ["Simple_Room_x"]})
        assert "不能用" in write({"rooms": ["Simple_Room_teal"]})  # the exam pool's room
        assert write({"pool": "exam", "rooms": ["Simple_Room_teal"]}) is None
        assert "pool 只能是" in write({"pool": "both"})
        assert "超出范围" in write({"clutter": {"count": [5, 2]}})
        assert "灯光类型" in write({"lights": {"count": [1, 1], "types": ["Dome"]}})
        assert "干扰物池" in write({"clutter": {"count": [1, 2], "categories": ["x"]}})
        assert "不认识" in write({"lighting": True})
        assert write(None) is None and not path.exists()
    finally:
        server.rsi_profile.PROFILES_DIR = profiles


def test_appearance_settings():
    tmp, scenes_dir = Path(tempfile.mkdtemp()), server.build_asset.SCENES_DIR
    for scene in ("real_scene3_lab", "real_scene2_counter"):
        (tmp / scene / "usd").mkdir(parents=True)
        (tmp / scene / "usd/parts.json").write_text((scenes_dir / scene / "usd/parts.json").read_text())
    server.build_asset.SCENES_DIR = tmp
    try:
        lab = {"fixed": {"floor": "mdl/material_0124", "wall": None, "table_top_z": 0.9}}
        lab["random"] = {"tabletop": ["original", "real_scene1_office/tabletop/table_white"]}
        lab["random"]["table_top_z"] = [0.7, 0.95]
        assert server.write_appearance("real_scene3_lab", lab) is None
        written = json.loads((tmp / "real_scene3_lab/appearance.json").read_text())
        assert written == {"fixed": {"floor": "mdl/material_0124", "table_top_z": 0.9}, "random": lab["random"]}
        assert "没有 nope" in server.write_appearance("real_scene3_lab", {"fixed": {"floor": "nope"}})
        unfit = {"random": {"wall": ["real_scene3_lab/floor/carpet"]}}  # a scene's floor stays a floor
        assert "不适合" in server.write_appearance("real_scene3_lab", unfit)
        assert "行程" in server.write_appearance("real_scene3_lab", {"fixed": {"table_top_z": 1.5}})
        assert "两个值" in server.write_appearance("real_scene3_lab", {"random": {"table_top_z": [0.9, None]}})
        assert "数字" in server.write_appearance("real_scene3_lab", {"fixed": {"table_top_z": "0.9"}})
        assert "不能升降" in server.write_appearance("real_scene2_counter", {"fixed": {"table_top_z": 0.9}})
        assert server.write_appearance("real_scene2_counter", {"fixed": {"tabletop": "mdl/material_0083"}}) is None
        assert server.write_appearance("real_scene3_lab", {"fixed": {}, "random": {}}) is None
        assert not (tmp / "real_scene3_lab/appearance.json").exists()  # nothing set: the twin as built
        assert "parts.json" in server.write_appearance("real_scene9_x", {})
    finally:
        server.build_asset.SCENES_DIR = scenes_dir
    summary = server.parts_summary(json.loads((scenes_dir / "real_scene3_lab/usd/parts.json").read_text()))
    assert set(summary["roles"]) == {"floor", "wall", "tabletop"} and summary["lift"]["range"] == [0.62, 1.27]
    library = server.material_library()
    assert len(library) > 500 and all("thumb" in m for m in library)
    roles = {m["id"]: m["roles"] for m in library}
    assert roles["real_scene3_lab/wall/paint"] == ["wall"] and "floor" not in roles["mdl/material_0110"]  # plaster


def test_prototypes():
    found = server.scan_prototypes()  # real2sim/prototypes/<id>/platform.json; previews are not in git
    assert {"aurora_lite", "cloth_patch", "cable"} <= set(found)
    assert not any(p["runtime_ready"] for p in found.values())  # never offered as task objects
    assert all(Path(e).is_relative_to(server.PROTOTYPES_DIR) for p in found.values() for e in p["evidence"])
    tmp = Path(tempfile.mkdtemp())
    (tmp / "Bad-Name").mkdir()
    (tmp / "Bad-Name/platform.json").write_text("{}")
    (tmp / "ok").mkdir()
    (tmp / "ok/platform.json").write_text(json.dumps({"name": "x", "evidence": ["../../etc/passwd", "a.md"]}))
    (tmp / "ok/a.md").write_text("x")
    assert list(server.scan_prototypes(tmp)) == ["ok"] and server.scan_prototypes(tmp)["ok"]["evidence"] == [
        str((tmp / "ok/a.md").resolve())
    ]


def test_embodiments():
    bodies = server.scan_embodiments()  # robot_config modules; per-scene stands of one body count as one
    assert {"g1d", "x5", "franka"} <= set(bodies) and bodies["g1d"]["in_code"]
    lab = next(s for s in bodies["g1d"]["setups"] if s["robot_cfg"] == "g1d_lab")
    assert lab["env_cfgs"] == ["g1d_lab", "g1d_lab_beans_collect"]
    assert lab["scenes"] == ["real_scene3_lab"] and lab["arm_dim"] == [7, 7]
    assert next(s for s in bodies["x5"]["setups"] if s["robot_cfg"] == "dual_x5")["count"] == 2


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


def test_account_migration():
    path = Path(tempfile.mkdtemp()) / "db.sqlite3"
    old = server.sqlite3.connect(path)  # accounts table from before admins existed
    old.execute("CREATE TABLE users (username TEXT PRIMARY KEY, name TEXT, pw TEXT)")
    old.execute("INSERT INTO users VALUES ('bob', 'Bob', ?)", (server.hash_password("password1"),))
    old.commit()
    old.close()
    store = server.Store(path)
    assert store.users() == [{"username": "bob", "name": "Bob", "admin": False}]
    assert store.check("bob", "password1") == {"username": "bob", "name": "Bob", "admin": False}
    assert store.edit_user("bob", admin=True) and store.users()[0]["admin"]
    assert not store.add_user("bob", "password2") and store.check("bob", "password1")


def test_http():
    tmp = Path(tempfile.mkdtemp())
    run = make_run(tmp)
    (tmp / "secret.txt").write_text("x")
    store = server.Store(tmp / "db.sqlite3")
    store.add_user("alice", "password1", "张三", admin=True)
    server.Handler.store, server.Handler.scan_args, server.Handler.roots = store, (tmp, [tmp]), [tmp]
    server.Handler.failures, server.FAIL_DELAY = {}, 0
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    evals_dir = server.EVALS_DIR
    session, scenes_dir, tasks_dir, bodies_dir = (
        {},
        server.build_asset.SCENES_DIR,
        server.TASKS_DIR,
        server.EMBODIMENTS_DIR,
    )

    def request(method, path, body=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", httpd.server_port)
        conn.request(method, path, body, {**session, **(headers or {})})
        resp = conn.getresponse()
        return resp.status, resp.read(), resp

    def login(username, password):
        status, body, resp = request("POST", "/api/login", json.dumps({"username": username, "password": password}))
        return status, json.loads(body), (resp.getheader("Set-Cookie") or "").split(";")[0]

    try:
        status, page, resp = request("GET", "/")
        assert b"Real2Sim" in page and resp.getheader("Cache-Control") == "no-cache"  # public page, revalidated
        conn = http.client.HTTPConnection("127.0.0.1", httpd.server_port)
        conn.request("POST", "/api/photo?id=Rigid/cup/00000&name=a.jpg", b"jpg")  # signed out: body left unread
        resp = conn.getresponse()
        assert (resp.status, resp.getheader("Connection")) == (401, "close")  # so the connection is not reused
        status, _, resp = request("GET", "/%23scenes")  # a reload that encoded the tab fragment
        assert (status, resp.getheader("Location")) == (302, "/#scenes")
        assert request("GET", "/%23a%0D%0ASet-Cookie:x")[0] == 401  # no header injection through the redirect
        assert request("GET", "/api/info")[0] == 401 and request("GET", "/api/me")[0] == 401
        assert login("alice", "wrong-pass")[0] == 401 and login("nobody", "password1")[0] == 401
        status, me, cookie = login("alice", "password1")
        assert status == 200 and me == {"username": "alice", "name": "张三", "admin": True}
        assert cookie.startswith("r2s_session=")
        session["Cookie"] = cookie
        assert json.loads(request("GET", "/api/me")[1])["name"] == "张三"

        video = run / "episode_0000000_cam_head_success.mp4"
        status, body, resp = request("GET", f"/file?path={video}", headers={"Range": "bytes=2-5"})
        assert (status, body, resp.getheader("Content-Range")) == (206, b"2345", "bytes 2-5/10")
        status, _, resp = request("GET", f"/file?path={video}")
        etag = resp.getheader("ETag")
        assert status == 200 and etag and resp.getheader("Cache-Control") == server.FILE_CACHE
        assert request("GET", f"/file?path={video}", headers={"If-None-Match": etag})[:2] == (304, b"")
        model = run / "scene.glb"  # rebuilt in place: rechecked on every view
        model.write_bytes(b"glTF")
        status, _, resp = request("GET", f"/file?path={model}")
        assert status == 200 and resp.getheader("Cache-Control") == server.PREVIEW_CACHE
        conn = http.client.HTTPConnection("127.0.0.1", httpd.server_port)  # keep-alive: one socket, also after a 416
        steps = [({"Range": "bytes=20-30"}, 416, b""), ({}, 200, b"0123456789")]
        steps.append(({"If-None-Match": f'W/"x", {etag}'}, 304, b""))
        socks = []
        for headers, want_status, want_body in steps:
            conn.request("GET", f"/file?path={video}", headers={**session, **headers})
            resp = conn.getresponse()
            assert (resp.status, resp.read()) == (want_status, want_body)
            socks.append(conn.sock)
        assert socks[0] is not None and all(s is socks[0] for s in socks)
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

        built = tmp / "Object/RoboDojo/Rigid/cup/00003"  # a library asset reporting its size per dimension name
        built.mkdir(parents=True)
        (built / "metadata.json").write_text(json.dumps({"physics": {"mass": 0.05}, "measurements_mm": {"高": 95.5}}))
        assert server.scan_objects(tmp, {}, ["Rigid/cup/00003"])["Rigid/cup/00003"]["measurements"] == {"高": 95.5}
        preview = tmp / "objects/Rigid/cup/00004/preview"  # built and checked, not in the library yet
        preview.mkdir(parents=True)
        check = {"settle": "pass", "mass_kg": 0.038, "size_m": [0.05, 0.04, 0.02], "measurements_mm": {"高": 46.4}}
        (preview / "check.json").write_text(json.dumps(check))
        built = server.scan_objects(tmp, {}, ["Rigid/cup/00004"])["Rigid/cup/00004"]
        assert not built["in_library"] and built["built"] and built["check"]["settle"] == "pass"
        assert (built["mass"], built["size"], built["measurements"]) == (0.038, [0.05, 0.04, 0.02], {"高": 46.4})
        assert built["variant_of"] is None
        (preview / "check.json").write_text(json.dumps({**check, "variant_of": "Rigid/cup/00003"}))
        assert server.scan_objects(tmp, {}, ["Rigid/cup/00004"])["Rigid/cup/00004"]["variant_of"] == "Rigid/cup/00003"
        (tmp / "Object/RoboDojo/Rigid/cup/00003/metadata.json").write_text(
            json.dumps({"variant_of": "Rigid/cup/00000"})
        )
        assert server.scan_objects(tmp, {}, ["Rigid/cup/00003"])["Rigid/cup/00003"]["variant_of"] == "Rigid/cup/00000"
        twin = {"twin": True, "name": "量杯", "name_en": "Measuring Cup", "real_mass": 55, "material": "PP 塑料"}
        twin["gen"] = {"count": 3, "color": True, "size": False, "requirements": "形状不变，只换材质"}
        twin["gen"] |= {"rt_color": True, "rt_scale": 15}
        twin["dimensions"] = "开口直径 82，高 158"
        assert request("PUT", "/api/items?kind=object&id=Rigid/cup/00000", json.dumps({**twin, "color": ""}))[0] == 200
        spec = json.loads((tmp / "objects/Rigid/cup/00000/spec.json").read_text(encoding="utf-8"))
        assert spec.pop("updated_at") and spec == {  # empty fields left out
            "id": "Rigid/cup/00000",
            "name": "量杯",
            "name_en": "Measuring Cup",
            "material": "PP 塑料",
            "mass_g": 55,
            "dimensions_mm": "开口直径 82，高 158",
            "generalization": {
                "count": 3,
                "color": True,
                "requirements": "形状不变，只换材质",
                "rt_color": True,
                "rt_scale": 15,
            },
            "updated_by": "张三",
        }
        assert {0, 3} <= set(json.loads(request("GET", "/api/object-indices")[1])["Rigid/cup"])  # registered, library
        request("PUT", "/api/items?kind=object&id=Rigid/cup/00000", json.dumps({"twin": False}))
        assert not (tmp / "objects/Rigid/cup/00000/spec.json").exists()  # only real objects get a modeling input
        request("PUT", "/api/items?kind=object&id=Rigid/cup/00000", json.dumps({"twin": True}))
        assert request("DELETE", "/api/items?kind=object&id=Rigid/cup/00000")[0] == 200
        assert not (tmp / "objects/Rigid/cup/00000/spec.json").exists()

        server.build_asset.SCENES_DIR = tmp / "scenes"
        scene = "/api/items?kind=scene&id=real_scene9_kitchen"
        reg = {"name": "厨房台面", "top_z_cm": 91.5, "length_cm": 120, "width_cm": 60, "offset_cm": 0}
        reg |= {"measured_on": "2026-10-20", "charuco": False, "tasks": [], "notes": ""}
        reg |= {"light_sources": "吸顶面板灯 4 盏", "color_temp_k": 4000}
        assert request("PUT", "/api/items?kind=scene&id=Bad/../x", json.dumps({"registered": True}))[0] == 400
        assert request("PUT", scene, json.dumps({"registered": True, "reg": reg}))[0] == 200
        written = json.loads((tmp / "scenes/real_scene9_kitchen/registration.json").read_text(encoding="utf-8"))
        tape = lambda m: {"value_m": m, "tool": "卷尺", "date": "2026-10-20"}  # noqa: E731
        assert written.pop("updated_at") and written == {  # cm -> scene_spec.json measurements; a 0 offset is kept
            "scene": "real_scene9_kitchen",
            "name": "厨房台面",
            "measurements": {
                "table_top_z": tape(0.915),
                "table_length": tape(1.2),
                "table_width": tape(0.6),
                "table_centre_x_from_robot_root": tape(0.0),
            },
            "lighting": {"sources": "吸顶面板灯 4 盏", "color_temp_k": 4000},
            "updated_by": "张三",
        }
        status, body, _ = request("POST", "/api/photo?kind=scene&id=real_scene9_kitchen&name=front.webp", b"webp")
        front = tmp / "scenes/real_scene9_kitchen/reference/front.webp"
        assert status == 200 and json.loads(body)["path"] == str(front)
        assert request("POST", "/api/photo?kind=scene&id=Rigid/cup/00000&name=a.jpg", b"jpg")[0] == 400
        found = server.scan_scenes()["real_scene9_kitchen"]
        assert found["registered"] and not found["built"] and len(found["photos"]) == 1
        assert found["renders"] == [] and found["model"] is None
        preview = tmp / "scenes/real_scene9_kitchen/preview"
        (preview / "render").mkdir(parents=True)
        (preview / "render/sim.jpg").write_bytes(b"jpg")
        (preview / "scene.glb").write_bytes(b"glTF")
        found = server.scan_scenes()["real_scene9_kitchen"]
        assert found["renders"] == [str(preview / "render/sim.jpg")] and found["model"] == str(preview / "scene.glb")
        assert found["previews"] == []  # the render is not listed again among the comparison previews
        status, body, _ = request("GET", f"/file?path={preview / 'scene.glb'}")
        assert status == 200 and body == b"glTF"
        assert request("DELETE", f"/api/photo?path={found['photos'][0]}")[0] == 200
        store.add_user("dave", "password1")
        member = login("dave", "password1")[2]
        assert request("DELETE", scene, headers={"Cookie": member})[0] == 403  # admins only
        store.remove_user("dave")
        modeled = tmp / "scenes/real_scene8_lab"
        modeled.mkdir()
        (modeled / "room.usd").write_bytes(b"usd")
        assert request("DELETE", "/api/items?kind=scene&id=real_scene8_lab")[0] == 409  # lives in code and git
        assert request("DELETE", "/api/items?kind=scene&id=robodojo_default")[0] == 409  # env_cfg/scene/default.yml
        assert request("DELETE", scene)[0] == 200
        assert not (tmp / "scenes/real_scene9_kitchen").exists()  # registration, photos and previews

        for inst in ("cup/00000", "cup/00001", "ball/00000"):  # the Clutter library, as in the asset mirror
            (tmp / "Object/RoboDojo/Clutter" / inst).mkdir(parents=True)
            (tmp / "Object/RoboDojo/Clutter" / inst / "metadata.json").write_text("{}")
        assert json.loads(request("GET", "/api/clutter")[1]) == {"ball": 1, "cup": 2}

        server.EMBODIMENTS_DIR = tmp / "embodiments"
        body = "/api/items?kind=embodiment&id=g1d_mobile"
        assert request("PUT", "/api/items?kind=embodiment&id=G1D/../x", json.dumps({"registered": True}))[0] == 400
        reg = {"name": "G1-D 移动版", "dof": "双臂各 7 + 差速底盘", "calib_camera": "done", "units": ""}
        assert request("PUT", body, json.dumps({"registered": True, "reg": reg, "owner": "alice"}))[0] == 200
        written = json.loads((tmp / "embodiments/g1d_mobile/registration.json").read_text(encoding="utf-8"))
        assert written.pop("updated_at") and written == {
            "embodiment": "g1d_mobile",
            "name": "G1-D 移动版",
            "dof": "双臂各 7 + 差速底盘",
            "calib_camera": "done",
            "owner": {"username": "alice", "name": "张三"},
            "updated_by": "张三",
        }
        status, body_photo, _ = request("POST", "/api/photo?kind=embodiment&id=g1d_mobile&name=side.jpg", b"jpg")
        assert status == 200 and json.loads(body_photo)["path"].endswith("embodiments/g1d_mobile/reference/side.jpg")
        found = server.scan_embodiments()["g1d_mobile"]
        assert found["registered"] and len(found["photos"]) == 1 and "setups" in found
        (tmp / "embodiments/g1d_mobile/preview").mkdir()
        (tmp / "embodiments/g1d_mobile/preview/sim.png").write_bytes(b"png")
        sim = tmp / "embodiments/g1d_mobile/preview/sim.png"
        assert server.scan_embodiments()["g1d_mobile"]["renders"] == [str(sim)]
        assert request("DELETE", body)[0] == 200 and not (tmp / "embodiments/g1d_mobile/registration.json").exists()

        server.EVALS_DIR = tmp / "evals"
        ev = "/api/items?kind=evaluation&id=openwam-1000-20261021"
        assert request("PUT", "/api/items?kind=evaluation&id=Bad/../x", json.dumps({"registered": True}))[0] == 400
        reg = {
            "model_name": "OpenWAM step1000",
            "checkpoint": "5090-2:/data/ckpt",
            "tasks": ["g1d_stack_blocks"],
            "env": "",
        }
        assert request("PUT", ev, json.dumps({"registered": True, "reg": reg, "submitter": "forged"}))[0] == 200
        written = json.loads((tmp / "evals/openwam-1000-20261021/request.json").read_text(encoding="utf-8"))
        assert written.pop(
            "updated_at"
        ) and written == {  # submitter is the signed-in account, new requests start submitted
            "evaluation": "openwam-1000-20261021",
            "status": "submitted",
            "model_name": "OpenWAM step1000",
            "checkpoint": "5090-2:/data/ckpt",
            "tasks": ["g1d_stack_blocks"],
            "submitter": {"username": "alice", "name": "张三"},
            "updated_by": "张三",
        }
        request("PUT", ev, json.dumps({"status": "evaluating", "assignee": "alice", "submitter": "forged"}))
        written = json.loads((tmp / "evals/openwam-1000-20261021/request.json").read_text(encoding="utf-8"))
        assert written["status"] == "evaluating" and written["assignee"]["name"] == "张三"
        assert written["submitter"]["username"] == "alice"  # a later update cannot change who submitted
        result = {"summary": "ok", "runs": [{"task": "g1d_stack_blocks", "episodes": 6, "success_rate": 0.5}]}
        (tmp / "evals/openwam-1000-20261021/result.json").write_text(json.dumps(result))
        assert server.scan_evaluations()["openwam-1000-20261021"]["result"] == result
        assert request("DELETE", ev)[0] == 200 and not (tmp / "evals/openwam-1000-20261021/request.json").exists()

        server.TASKS_DIR = tmp / "tasks"
        task = "/api/items?kind=task&id=g1d_wipe_table"
        roles = [
            {
                "label": "sponge",
                "candidates": ["Rigid/sponge/00000"],
                "count": 1,
                "x": None,
                "holdout": ["Rigid/sponge/00001"],
            }
        ]
        reg = {"name": "擦桌子", "definition": "用海绵把桌上的污渍擦掉", "success": "污渍清除", "roles": roles}
        reg["instructions"] = {"count": 5, "languages": ["zh", "en"]}
        reg["randomize"] = ["position"]
        reg["appearance"] = ["light"]
        reg |= {"scenes": {"real_scene3_lab": "train", "real_scene1_office": "holdout", "real_scene2_counter": ""}}
        reg |= {"levels": {"L0": {"target": 25, "range": "孪生"}}, "distractors": {"count": None, "candidates": []}}
        assert request("PUT", "/api/items?kind=task&id=../x", json.dumps({"registered": True}))[0] == 400
        assert request("PUT", task, json.dumps({"registered": True, "reg": reg, "owner": "alice"}))[0] == 200
        written = json.loads((tmp / "tasks/g1d_wipe_table/registration.json").read_text(encoding="utf-8"))
        assert written["owner"] == {"username": "alice", "name": "张三"}  # an account, written with its display name
        roles[0].pop("x")  # empty fields left out, also inside lists
        assert written["objects"] == roles
        assert written["scenes"] == {  # with the env configs that load each scene
            "train": [{"scene": "real_scene3_lab", "env_cfg": ["g1d_lab"]}],
            "holdout": [{"scene": "real_scene1_office", "env_cfg": ["g1d", "g1d_mobile"]}],
        }
        assert "distractors" not in written and written["levels"] == {"L0": {"target": 25, "range": "孪生"}}
        assert written["instructions"] == {"count": 5, "languages": ["zh", "en"]} and "instruction" not in written
        assert written["randomize"] == ["position"] and written["appearance"] == ["light"]
        assert server.scan_tasks(tmp)["g1d_wipe_table"]["instructions"] is None  # not written by the agent yet
        transcribed = {"zh": ["把桌子擦干净"], "en": ["Wipe the table clean."]}
        (tmp / "tasks/g1d_wipe_table/instructions.json").write_text(json.dumps(transcribed, ensure_ascii=False))
        listed = server.scan_tasks(tmp)["g1d_wipe_table"]  # registered, not implemented: listed, not runnable
        assert listed["registered"] and not listed["runnable"] and listed["module"] is None
        assert listed["instructions"] == transcribed
        (tmp / "tasks/g1d_wipe_table/instructions.json").unlink()
        assert not server.scan_tasks(tmp)["g1d_stack_blocks"]["registered"]
        assert request("DELETE", task)[0] == 200 and "g1d_wipe_table" not in server.scan_tasks(tmp)

        def users(method, path="/api/users", body=None, cookie=None):
            headers = {"Cookie": cookie} if cookie else {}
            status, data, _ = request(method, path, json.dumps(body) if body else None, headers)
            return status, json.loads(data)

        carol = {"username": "carol", "name": "李四", "password": "password1"}
        assert users("POST", body={**carol, "password": "short"})[0] == 400
        assert users("POST", body={**carol, "username": "a b"})[0] == 400
        status, listed = users("POST", body=carol)
        assert status == 200 and {"username": "carol", "name": "李四", "admin": False} in listed
        assert users("POST", body=carol)[0] == 409
        carol_cookie = login("carol", "password1")[2]
        for method in ("GET", "POST", "PUT", "DELETE"):  # members cannot manage accounts
            assert users(method, "/api/users?username=alice", {"admin": False}, carol_cookie)[0] == 403
        status, members = users("GET", "/api/members", cookie=carol_cookie)  # but see who can own a task
        assert status == 200 and {"username": "alice", "name": "张三"} in members and "admin" not in members[0]
        assert users("PUT", "/api/users?username=alice", {"admin": False})[0] == 400  # admins cannot demote themselves
        assert users("DELETE", "/api/users?username=alice")[0] == 400
        assert users("PUT", "/api/users?username=nobody", {"name": "x"})[0] == 404
        status, listed = users("PUT", "/api/users?username=carol", {"name": "王五", "password": "password9"})
        assert status == 200 and {"username": "carol", "name": "王五", "admin": False} in listed
        assert request("GET", "/api/me", headers={"Cookie": carol_cookie})[0] == 401  # reset password ends sessions
        carol_cookie = login("carol", "password9")[2]
        assert users("PUT", "/api/users?username=carol", {"admin": True})[1][1]["admin"]
        assert users("GET", cookie=carol_cookie)[0] == 200
        assert users("DELETE", "/api/users?username=carol")[1] == [{"username": "alice", "name": "张三", "admin": True}]
        assert request("GET", "/api/me", headers={"Cookie": carol_cookie})[0] == 401

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
        server.build_asset.SCENES_DIR, server.TASKS_DIR, server.EMBODIMENTS_DIR = scenes_dir, tasks_dir, bodies_dir
        server.EVALS_DIR = evals_dir
        httpd.shutdown()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok {name}")
