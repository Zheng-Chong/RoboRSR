"""Download CC0 HDRIs from Poly Haven (api.polyhaven.com) into an Assets/Background tree, for the RSI background pools
(rsi/AGENTS.md): the benchmark ships one HDR, which its training data also uses. Run where the internet is reachable,
then copy the folder to the GPU server and merge it with scripts/datagen/rsi_assets.py link.

    python scripts/scenes/polyhaven_hdris.py --out extra_hdr/Background --count 40 [--environment indoor] [--res 2k]
Writes <out>/polyhaven_<id>_<res>.hdr and <out>/polyhaven.json (id, name, authors, licence, source URL per file).
"""

import argparse
import json
from pathlib import Path
import time
import urllib.request

API = "https://api.polyhaven.com"
HEADERS = {"User-Agent": "RoboDojo-RSI/0.1 (robot simulation backgrounds)"}


def get(url, tries=5):
    for attempt in range(tries):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=HEADERS), timeout=120) as r:
                return r.read()
        except OSError:
            if attempt == tries - 1:
                raise
            time.sleep(2 * (attempt + 1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--environment", default="indoor", help="Poly Haven attribute, e.g. indoor / outdoor; '' = any")
    ap.add_argument("--res", default="2k")
    args = ap.parse_args(argv)
    assets = json.loads(get(f"{API}/assets?type=hdris"))
    picked = [
        (k, v)
        for k, v in sorted(assets.items(), key=lambda kv: -kv[1].get("download_count", 0))
        if not args.environment or (v.get("attributes") or {}).get("environment") == args.environment
    ][: args.count]
    args.out.mkdir(parents=True, exist_ok=True)
    record = []
    for key, info in picked:
        url = json.loads(get(f"{API}/files/{key}"))["hdri"][args.res]["hdr"]["url"]
        name = f"polyhaven_{key}_{args.res}.hdr"
        (args.out / name).write_bytes(get(url))
        record.append(
            {
                "file": name,
                "id": key,
                "name": info.get("name"),
                "authors": info.get("authors"),
                "license": "CC0-1.0",
                "url": url,
            }
        )
        print(name, flush=True)
    (args.out / "polyhaven.json").write_text(json.dumps(record, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
