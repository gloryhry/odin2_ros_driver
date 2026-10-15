#!/usr/bin/env python3
"""Publish after every catalog image passed, without matrix tag races."""
import importlib.util
import json
import os
import pathlib
import re
import subprocess
import sys

spec = importlib.util.spec_from_file_location("support", pathlib.Path(__file__).with_name("jetson-support.py"))
support = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support)


def image_tags(repository, row, revision, ref):
    templates = support.catalog()["image_tags"]
    values = dict(row, revision=revision)
    tags = [repository + ":" + templates[key].format(**values) for key in ("l4t", "revision")]
    release = re.fullmatch(r"v([0-9]+\.[0-9]+\.[0-9]+(?:-[0-9A-Za-z.-]+)?)", ref)
    if release:
        tags.append(repository + ":" + templates["release"].format(**dict(values, release=release[1])))
    # Preserve the existing family alias contract (35.4.1 / 36.4.0).
    if row["legacy_alias"]:
        tags.append(repository + ":" + templates["legacy"])
        if release:
            tags.append(repository + ":" + release[1])
    return tags


def main():
    records = {}
    for path in pathlib.Path(sys.argv[1]).rglob("jetson-build.json"):
        record = json.loads(path.read_text())
        if record["l4t"] in records:
            raise ValueError("Duplicate build artifact: " + record["l4t"])
        records[record["l4t"]] = record
    rows = support.catalog()["versions"]
    if set(records) != {row["l4t"] for row in rows}:
        raise ValueError("Missing or unexpected Jetson build artifacts")
    revision = os.environ["GITHUB_SHA"]
    plans = []
    for row in rows:
        record = records[row["l4t"]]
        if record["build"] != "passed" or record["source_revision"] != revision or record["jetpack"] != row["jetpack"]:
            raise ValueError("Build artifact does not match this release: " + row["l4t"])
        digest = record["digest"]
        if not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
            raise ValueError("Invalid image digest")
        suffix = support.catalog()["image_tags"]["repository_suffix"].format(**row)
        repository = "ghcr.io/" + os.environ["GITHUB_REPOSITORY"].lower() + suffix
        source = repository + "@" + digest
        index = json.loads(subprocess.check_output(["docker", "buildx", "imagetools", "inspect", source, "--raw"], text=True))
        manifests = [m for m in index["manifests"] if m.get("platform", {}).get("os") != "unknown"]
        if len(manifests) != 1 or manifests[0]["platform"]["architecture"] != "arm64" or manifests[0]["platform"]["os"] != "linux":
            raise ValueError("Expected exactly one linux/arm64 platform: " + source)
        config = json.loads(subprocess.check_output(["docker", "buildx", "imagetools", "inspect",
            repository + "@" + manifests[0]["digest"], "--format", "{{json .Image}}"], text=True))
        labels = config["config"]["Labels"]
        if labels["org.opencontainers.image.revision"] != revision or labels["io.odin2.target.l4t"] != row["l4t"]:
            raise ValueError("Image labels do not match the release contract")
        ref = os.environ["GITHUB_REF_NAME"] if os.environ.get("GITHUB_REF_TYPE") == "tag" else ""
        plans.append((source, image_tags(repository, row, revision, ref)))
    for source, tags in plans:
        command = ["docker", "buildx", "imagetools", "create"]
        for tag in tags:
            command += ["--tag", tag]
        subprocess.run(command + [source], check=True)
        print(source + " -> " + ", ".join(tags), flush=True)


if __name__ == "__main__":
    main()
