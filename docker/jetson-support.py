#!/usr/bin/env python3
"""Shared Jetson catalog, image contract and non-sensor environment checks."""
import argparse
import hashlib
import json
import os
import pathlib
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = pathlib.Path(__file__).resolve().parent
CATALOG = HERE / "jetson-versions.json"
METADATA = pathlib.Path("/opt/odin2/jetson-metadata.json")
RELEASE = pathlib.Path("/run/host-nv_tegra_release")
STATUS = pathlib.Path("/run/host-dpkg-status")
PLUGINS = ("appsrc", "appsink", "jpegparse", "h264parse", "h265parse",
           "nvv4l2decoder", "nvvidconv")


def normalize(value):
    if not re.fullmatch(r"[0-9]+\.[0-9]+(?:\.[0-9]+)?", value):
        raise ValueError("Invalid L4T version: " + value)
    return value if value.count(".") == 2 else value + ".0"


def catalog():
    data = json.loads(CATALOG.read_text())
    if data["schema_version"] != 1:
        raise ValueError("Unsupported Jetson catalog schema")
    seen = set()
    for row in data["versions"]:
        version = row["l4t"]
        if normalize(version) != version or version in seen:
            raise ValueError("Duplicate or non-canonical L4T: " + version)
        seen.add(version)
        if not re.fullmatch(r".+@sha256:[0-9a-f]{64}", row["base_image"]):
            raise ValueError("Base image is not digest pinned: " + version)
        package = row["gstreamer"]
        if not package["version"].startswith(version + "-"):
            raise ValueError("Wrong NVIDIA package version: " + version)
        if not re.fullmatch(r"[0-9a-f]{64}", package["sha256"]):
            raise ValueError("Missing NVIDIA package checksum: " + version)
        if not package["url"].startswith("https://repo.download.nvidia.com/jetson/"):
            raise ValueError("NVIDIA package must come from the official repository")
    return data


def select(version):
    version = normalize(version)
    for row in catalog()["versions"]:
        if row["l4t"] == version:
            return row
    raise ValueError("Unsupported Jetson L4T " + version + "; select a listed GA/security release")


def host_version(release=RELEASE, status=STATUS):
    match = re.search(r"^# R(\d+).*?REVISION:\s*([\d.]+),", release.read_text(), re.M)
    if not match:
        raise ValueError("Cannot parse host /etc/nv_tegra_release")
    version = normalize(match[1] + "." + match[2])
    select(version)
    for block in status.read_text().split("\n\n"):
        fields = dict(line.split(": ", 1) for line in block.splitlines()
                      if ": " in line and not line.startswith(" "))
        if fields.get("Package") == "nvidia-l4t-core":
            if fields.get("Status") != "install ok installed":
                raise ValueError("Host nvidia-l4t-core is not fully installed")
            core = fields.get("Version", "").split("-", 1)[0]
            if normalize(core) != version:
                raise ValueError("Host release/core mismatch: release=%s core=%s" % (version, core))
            return version
    raise ValueError("Host dpkg status has no installed nvidia-l4t-core")


def check_host(release=RELEASE, status=STATUS, metadata=METADATA):
    version = host_version(release, status)
    data = json.loads(metadata.read_text())
    row = select(version)
    if data["target_l4t"] != version:
        raise ValueError("Jetson target mismatch: host=%s image=%s; choose :l4t%s" %
                         (version, data["target_l4t"], version))
    if data["base_image"] != row["base_image"] or data["gstreamer"] != row["gstreamer"]:
        raise ValueError("Image metadata differs from its pinned Jetson catalog")
    print("Jetson host=%s JetPack=%s base=%s revision=%s" %
          (version, row["jetpack"], data["base_image"], data["source_revision"]))
    return row


def download(package, destination):
    for attempt in range(4):
        try:
            digest = hashlib.sha256()
            with urllib.request.urlopen(package["url"], timeout=60) as response, open(destination, "wb") as out:
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
                    out.write(chunk)
            if digest.hexdigest() != package["sha256"]:
                raise ValueError("Checksum mismatch: " + package["url"])
            return
        except (OSError, TimeoutError):
            if attempt == 3:
                raise
            time.sleep(attempt + 1)


def install(version, base_image, revision):
    row = select(version)
    if base_image != row["base_image"]:
        raise ValueError("BASE_IMAGE does not match the digest pinned in the catalog")
    if subprocess.check_output(["dpkg", "--print-architecture"], text=True).strip() != "arm64":
        raise ValueError("Jetson images must be built for linux/arm64")
    os_release = pathlib.Path("/etc/os-release").read_text()
    if 'VERSION_CODENAME=' + row["ubuntu_codename"] not in os_release:
        raise ValueError("Wrong Ubuntu release for this JetPack")
    package = row["gstreamer"]
    subprocess.run(["apt-get", "-o", "Acquire::Retries=5", "update"], check=True)
    subprocess.run(["apt-get", "-o", "Acquire::Retries=5", "install", "-y", "--no-install-recommends"] +
                   package["ubuntu_dependencies"], check=True)
    with tempfile.TemporaryDirectory() as tmp:
        path = str(pathlib.Path(tmp) / "gstreamer.deb")
        download(package, path)
        for field, expected in (("Package", "nvidia-l4t-gstreamer"),
                                ("Version", package["version"]), ("Architecture", "arm64")):
            actual = subprocess.check_output(["dpkg-deb", "-f", path, field], text=True).strip()
            if actual != expected:
                raise ValueError("Unexpected NVIDIA Debian " + field + ": " + actual)
        # Follow NVIDIA's container recipe: unpack userspace plugins only. Do not
        # run BSP maintainer scripts, install kernels, or fake the dpkg database.
        subprocess.run(["dpkg-deb", "--extract", path, "/"], check=True)
    pathlib.Path("/etc/ld.so.conf.d/odin2-jetson.conf").write_text(
        "/usr/lib/aarch64-linux-gnu/tegra\n/usr/lib/aarch64-linux-gnu/tegra-egl\n"
        "/usr/lib/aarch64-linux-gnu/nvidia\n")
    subprocess.run(["ldconfig"], check=True)
    METADATA.parent.mkdir(parents=True, exist_ok=True)
    METADATA.write_text(json.dumps({"schema_version": 1, "target_l4t": row["l4t"],
        "jetpack": row["jetpack"], "base_image": base_image, "gstreamer": package,
        "runtime_injection_required": True, "source_revision": revision}, indent=2) + "\n")


def check_devices(row):
    device = "/dev/nvhost-nvdec" if row["jetpack"] == "5" else "/dev/v4l2-nvdec"
    for name in (device, "/dev/nvmap"):
        # R36's V4L2 node may legitimately be c 1,3; use access and real decode
        # acceptance instead of interpreting its major/minor as a fake device.
        if not pathlib.Path(name).is_char_device() or not os.access(name, os.R_OK | os.W_OK):
            raise ValueError("Missing or inaccessible Jetson device: " + name)


def check_plugins():
    if not shutil.which("gst-inspect-1.0"):
        raise ValueError("Missing gst-inspect-1.0; install the Jetson image, including GStreamer tools")
    libraries = set()
    env = dict(os.environ, GST_REGISTRY="/tmp/odin2-preflight-gstreamer.bin")
    for plugin in PLUGINS:
        result = subprocess.run(["gst-inspect-1.0", plugin], text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, env=env, timeout=30)
        if result.returncode:
            raise ValueError("GStreamer plugin unavailable: %s\n%s" % (plugin, result.stdout))
        match = re.search(r"^\s*Filename\s+(/\S+)", result.stdout, re.M)
        if not match:
            raise ValueError("Cannot locate GStreamer plugin library: " + plugin)
        libraries.add(match[1])
        print("plugin %s: %s" % (plugin, match[1]))
    for library in sorted(libraries):
        result = subprocess.run(["ldd", library], text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=30)
        if result.returncode or "not found" in result.stdout:
            raise ValueError("Unresolved GStreamer dependency: %s\n%s" % (library, result.stdout))


def probe():
    row = check_host()
    if platform.machine() not in ("aarch64", "arm64"):
        raise ValueError("Container is not running as ARM64")
    check_devices(row)
    check_plugins()
    print("PASS: Jetson environment ready; real dual-camera hardware decoding is still required")


def preflight(image):
    version = host_version(pathlib.Path("/etc/nv_tegra_release"), pathlib.Path("/var/lib/dpkg/status"))
    if platform.machine() not in ("aarch64", "arm64"):
        raise ValueError("Preflight must run on the target Jetson ARM64 host")
    runtimes = json.loads(subprocess.check_output(["docker", "info", "--format", "{{json .Runtimes}}"], text=True))
    if "nvidia" not in runtimes:
        raise ValueError("NVIDIA Container Runtime is not registered with Docker")
    info = json.loads(subprocess.check_output(["docker", "image", "inspect", image], text=True))[0]
    if info["Architecture"] != "arm64" or info["Os"] != "linux":
        raise ValueError("Image is not linux/arm64")
    print("Host L4T=%s image=%s local_id=%s repo_digests=%s" %
          (version, image, info["Id"], info.get("RepoDigests", [])), flush=True)
    # --pull=never, network=none and an overridden entrypoint keep this check
    # independent of ROS, sensor discovery and the deployed Compose service.
    # Leave the disposable root writable for older NVIDIA ldconfig hooks; the
    # two host files remain read-only binds. No deployed container is modified.
    command = ["docker", "create", "--pull=never", "--runtime=nvidia", "--network=none",
        "--tmpfs", "/tmp:rw,nosuid,nodev", "-e", "NVIDIA_VISIBLE_DEVICES=all",
        "-e", "NVIDIA_DRIVER_CAPABILITIES=all", "-e", "NVIDIA_REQUIRE_JETPACK=csv-mounts=all",
        "--entrypoint", "python3"]
    for source, target in (("/etc/nv_tegra_release", RELEASE), ("/var/lib/dpkg/status", STATUS)):
        command += ["--mount", "type=bind,src=%s,dst=%s,readonly" % (source, target)]
    command += [image, "/opt/odin2/jetson/jetson-support.py", "probe"]
    container = subprocess.check_output(command, text=True).strip()
    try:
        runtime = subprocess.check_output(["docker", "inspect", container, "--format",
                                           "{{.HostConfig.Runtime}}"], text=True).strip()
        if runtime != "nvidia":
            raise ValueError("Temporary container did not select the NVIDIA runtime")
        subprocess.run(["docker", "start", "--attach", container], check=True, timeout=300)
        code = subprocess.check_output(["docker", "inspect", container, "--format",
                                       "{{.State.ExitCode}}"], text=True).strip()
        if code != "0":
            raise ValueError("Container environment probe failed (exit " + code + ")")
    finally:
        subprocess.run(["docker", "rm", "--force", container], stdout=subprocess.DEVNULL, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("matrix")
    query = commands.add_parser("select")
    query.add_argument("--l4t", required=True)
    commands.add_parser("check")
    commands.add_parser("probe")
    validate = commands.add_parser("validate")
    validate.add_argument("--download", action="store_true")
    build = commands.add_parser("install")
    build.add_argument("--l4t", required=True)
    build.add_argument("--base-image", required=True)
    build.add_argument("--revision", default="unknown")
    pre = commands.add_parser("preflight")
    pre.add_argument("--image", required=True)
    args = parser.parse_args()
    if args.command == "matrix":
        data = catalog()
        print(json.dumps({"include": [dict({key: row[key] for key in
              ("l4t", "jetpack", "base_image", "legacy_alias")},
              repository_suffix=data["image_tags"]["repository_suffix"].format(**row))
              for row in data["versions"]]}))
    elif args.command == "select":
        print(json.dumps(select(args.l4t), indent=2))
    elif args.command == "install":
        install(args.l4t, args.base_image, args.revision)
    elif args.command == "check":
        check_host()
    elif args.command == "probe":
        probe()
    elif args.command == "preflight":
        preflight(args.image)
    elif args.command == "validate":
        rows = catalog()["versions"]
        if args.download:
            with tempfile.TemporaryDirectory() as tmp:
                for row in rows:
                    download(row["gstreamer"], str(pathlib.Path(tmp) / "package.deb"))
                    print("NVIDIA SHA256 verified: " + row["l4t"])
        print("Valid Jetson catalog: %d versions" % len(rows))


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        print("Jetson check failed: " + str(error), file=sys.stderr)
        sys.exit(1)
