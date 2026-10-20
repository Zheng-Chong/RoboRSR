"""Upload an audited dataset folder to a private ModelScope dataset repo, then compare it file by file.

Run where `modelscope login` was done (the saved login is used; this script never reads or prints a token):
    python scripts/datagen/upload.py --dataset <dir> --name <repo name> check    # repo exists? who is logged in?
    python scripts/datagen/upload.py --dataset <dir> --name <repo name> upload   # needs final_validation.json
    python scripts/datagen/upload.py --dataset <dir> --name <repo name> verify   # paths, sizes, sha256 vs local
Record the result in scripts/datagen/registry.json.
"""

import argparse
import json
from pathlib import Path
import sys


def local_files(root):
    return {
        str(p.relative_to(root)): p for p in root.rglob("*") if p.is_file() and p.relative_to(root).parts[0] != "images"
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("mode", choices=("check", "upload", "verify"))
    args = parser.parse_args()
    from modelscope.hub.api import HubApi, ModelScopeConfig

    user, _ = ModelScopeConfig.get_user_info()
    if not user:
        sys.exit("No saved ModelScope login on this machine: run `modelscope login` first")
    api, repo = HubApi(), f"{user}/{args.name}"
    exists = api.repo_exists(repo, repo_type="dataset")
    print(json.dumps(dict(repo=repo, exists=exists)))
    if args.mode == "upload":
        final = json.loads((args.dataset / "final_validation.json").read_text())
        if final["status"] != "validated":
            sys.exit("Run scripts/datagen/audit.py first")
        if not exists:
            api.create_repo(repo, repo_type="dataset", visibility="private")
        api.upload_folder(
            repo_id=repo,
            folder_path=str(args.dataset),
            repo_type="dataset",
            commit_message=f"{args.name}: {final['export']['episodes']} episodes ({final['acceptance_version']})",
            ignore_patterns=["images/**"],
        )
        print(json.dumps(dict(repo=repo, uploaded=True)))
    elif args.mode == "verify":
        remote, page = {}, 1
        while True:
            files = api.get_dataset_files(repo, recursive=True, page_number=page, page_size=100)
            remote |= {f["Path"]: f for f in files if f.get("Type", "blob") != "tree"}
            if len(files) < 100:
                break
            page += 1
        local = local_files(args.dataset)
        sums = json.loads((args.dataset / "sha256.json").read_text())
        report = dict(
            repo=repo,
            local_files=len(local),
            remote_files=len(remote),
            missing=sorted(set(local) - set(remote)),
            extra=sorted(set(remote) - set(local) - {".gitattributes"}),
            size_mismatch=[
                k for k in local if k in remote and int(remote[k].get("Size", -1)) != local[k].stat().st_size
            ],
            sha256_mismatch=[k for k in sums if remote.get(k, {}).get("Sha256") not in (None, sums[k])],
        )
        print(json.dumps(report))
        if report["missing"] or report["size_mismatch"] or report["sha256_mismatch"]:
            sys.exit(1)


if __name__ == "__main__":
    main()
