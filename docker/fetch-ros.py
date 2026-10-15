#!/usr/bin/env python3
"""Fetch the checksum-locked Noetic release sources without rosdistro drift."""
import concurrent.futures
import importlib.util
import json
import pathlib
import subprocess
import sys
import tempfile

spec = importlib.util.spec_from_file_location("jetson_support", pathlib.Path(__file__).with_name("jetson-support.py"))
support = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support)
lock = json.loads(pathlib.Path(sys.argv[1]).read_text())
destination = pathlib.Path(sys.argv[2]).resolve()
destination.mkdir(parents=True, exist_ok=True)


def fetch(package):
    target = (destination / package["name"]).resolve()
    if destination not in target.parents:
        raise ValueError("Invalid source destination")
    target.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        archive = str(pathlib.Path(tmp) / "source.tar.gz")
        support.download(package, archive)
        subprocess.run(["tar", "--extract", "--gzip", "--file", archive, "--directory", str(target),
                        "--strip-components=1", "--no-same-owner"], check=True)


with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    list(pool.map(fetch, lock["packages"]))
print("Verified and extracted %d Noetic source archives" % len(lock["packages"]))
