#!/usr/bin/env python3
"""Regression checks for the Docker contract; no Jetson or Docker daemon needed."""
import importlib.util
import json
import os
import pathlib
import subprocess
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("jetson_support", ROOT / "docker/jetson-support.py")
support = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support)
spec = importlib.util.spec_from_file_location("jetson_publish", ROOT / "docker/publish-jetson.py")
publish = importlib.util.module_from_spec(spec)
spec.loader.exec_module(publish)


class JetsonContract(unittest.TestCase):
    def test_catalog_covers_ga_and_apt_security_updates(self):
        expected = {"35.1.0", "35.2.1", "35.3.1", "35.4.1", "35.5.0", "35.6.0",
                    "35.6.1", "35.6.2", "35.6.3", "35.6.4", "35.6.5", "36.3.0",
                    "36.4.0", "36.4.3", "36.4.4", "36.4.7", "36.5.0", "36.5.2"}
        self.assertEqual({r["l4t"] for r in support.catalog()["versions"]}, expected)

    def test_archive_short_versions_normalize(self):
        for short, full in (("35.1", "35.1.0"), ("36.3", "36.3.0"), ("36.4", "36.4.0")):
            self.assertEqual(support.select(short)["l4t"], full)

    def test_dp_igx_and_unlisted_versions_rejected(self):
        for version in ("34.1", "34.1.1", "36.2", "36.4.5", "36.4.6", "36.4.1", "36.5.1", "37.1"):
            with self.subTest(version=version), self.assertRaises(ValueError):
                support.select(version)

    def test_version_input_is_not_shell_or_repository_text(self):
        for version in ("r36.4.3", "36.4.3\n", "36.4.3-1", "36.4.3;true", ""):
            with self.subTest(version=version), self.assertRaises(ValueError):
                support.normalize(version)

    def fixture(self, folder, version="36.4.3", core=None):
        release = folder / "release"
        status = folder / "status"
        metadata = folder / "metadata.json"
        major, minor, patch = version.split(".")
        release.write_text("# R%s (release), REVISION: %s.%s, BOARD: generic\n" % (major, minor, patch))
        status.write_text("Package: unrelated\nStatus: install ok installed\n\nPackage: nvidia-l4t-core\n"
                          "Status: install ok installed\nVersion: %s-20250101\n" % (core or version))
        row = support.select(version)
        metadata.write_text(json.dumps(dict(target_l4t=version, base_image=row["base_image"],
                                            gstreamer=row["gstreamer"], source_revision="test")))
        return release, status, metadata

    def test_target_host_not_base_image_tag_is_checked(self):
        with tempfile.TemporaryDirectory() as tmp:
            files = self.fixture(pathlib.Path(tmp))
            self.assertEqual(support.check_host(*files)["l4t"], "36.4.3")
            data = json.loads(files[2].read_text())
            data["target_l4t"] = "36.4.0"
            files[2].write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "target mismatch"):
                support.check_host(*files)

    def test_partial_host_upgrade_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            files = self.fixture(pathlib.Path(tmp), core="36.4.4")
            with self.assertRaisesRegex(ValueError, "release/core mismatch"):
                support.check_host(*files)

    def test_unconfigured_host_package_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            files = self.fixture(pathlib.Path(tmp))
            files[1].write_text(files[1].read_text().replace("install ok installed", "install ok unpacked"))
            with self.assertRaisesRegex(ValueError, "not fully installed"):
                support.check_host(*files)

    def test_corrupt_package_is_rejected_before_extraction(self):
        package = {"url": "file:///unused", "sha256": "0" * 64}
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(support.urllib.request, "urlopen") as request:
            response = request.return_value.__enter__.return_value
            response.read.side_effect = [b"corrupt", b""]
            with self.assertRaisesRegex(ValueError, "Checksum mismatch"):
                support.download(package, pathlib.Path(tmp) / "package.deb")

    def test_r36_normal_character_device_is_accepted(self):
        with mock.patch.object(pathlib.Path, "is_char_device", return_value=True), \
                mock.patch.object(support.os, "access", return_value=True):
            support.check_devices(support.select("36.4.3"))

    def test_missing_device_is_actionable(self):
        with mock.patch.object(pathlib.Path, "is_char_device", return_value=False):
            with self.assertRaisesRegex(ValueError, "/dev/nvhost-nvdec"):
                support.check_devices(support.select("35.4.1"))

    def test_missing_gstreamer_tool_is_distinct_from_plugin_failure(self):
        with mock.patch.object(support.shutil, "which", return_value=None):
            with self.assertRaisesRegex(ValueError, "Missing gst-inspect"):
                support.check_plugins()

    def test_unloadable_nvidia_plugin_fails_preflight(self):
        def run(command, **kwargs):
            if command[1] == "nvv4l2decoder":
                return subprocess.CompletedProcess(command, 1, "libnvbufsurface.so: cannot open shared object")
            return subprocess.CompletedProcess(command, 0, "  Filename /tmp/plugin.so\n")
        with mock.patch.object(support.shutil, "which", return_value="gst-inspect-1.0"), \
                mock.patch.object(support.subprocess, "run", side_effect=run):
            with self.assertRaisesRegex(ValueError, "nvv4l2decoder"):
                support.check_plugins()

    def test_plugin_dependency_failure_fails_preflight(self):
        def run(command, **kwargs):
            return subprocess.CompletedProcess(command, 0, "libnvbufsurface.so => not found\n" if
                command[0] == "ldd" else "  Filename /tmp/plugin.so\n")
        with mock.patch.object(support.shutil, "which", return_value="gst-inspect-1.0"), \
                mock.patch.object(support.subprocess, "run", side_effect=run):
            with self.assertRaisesRegex(ValueError, "Unresolved"):
                support.check_plugins()

    def test_preflight_is_local_networkless_and_cleans_up_failure(self):
        calls = []
        def output(command, **kwargs):
            calls.append(command)
            if command[1] == "info":
                return '{"nvidia": {}}'
            if command[1:3] == ["image", "inspect"]:
                return '[{"Architecture":"arm64","Os":"linux","Id":"sha256:local"}]'
            if command[1] == "create":
                return "temporary-container\n"
            return "nvidia\n"
        def run(command, **kwargs):
            calls.append(command)
            if command[1] == "start":
                raise subprocess.CalledProcessError(1, command)
            return subprocess.CompletedProcess(command, 0)
        with mock.patch.object(support, "host_version", return_value="36.4.3"), \
                mock.patch.object(support.platform, "machine", return_value="aarch64"), \
                mock.patch.object(support.subprocess, "check_output", side_effect=output), \
                mock.patch.object(support.subprocess, "run", side_effect=run):
            with self.assertRaises(subprocess.CalledProcessError):
                support.preflight("local:test")
        create = next(c for c in calls if c[1] == "create")
        for flag in ("--pull=never", "--network=none", "--runtime=nvidia"):
            self.assertIn(flag, create)
        self.assertIn("NVIDIA_REQUIRE_JETPACK=csv-mounts=all", create)
        mounts = [create[i + 1] for i, arg in enumerate(create) if arg == "--mount"]
        self.assertEqual(len(mounts), 2)
        self.assertTrue(all(value.endswith(",readonly") for value in mounts))
        self.assertEqual(calls[-1], ["docker", "rm", "--force", "temporary-container"])


class JetsonPublishing(unittest.TestCase):
    revision = "a" * 40

    def test_each_family_has_exactly_one_legacy_alias(self):
        rows = support.catalog()["versions"]
        self.assertEqual([r["l4t"] for r in rows if r["legacy_alias"]], ["35.4.1", "36.4.0"])
        for row in rows:
            tags = publish.image_tags("image", row, self.revision, "v1.2.3-rc.1")
            self.assertIn("image:l4t" + row["l4t"], tags)
            self.assertIn("image:sha-" + self.revision + "-l4t" + row["l4t"], tags)
            self.assertIn("image:1.2.3-rc.1-l4t" + row["l4t"], tags)
            self.assertEqual("image:latest" in tags, row["legacy_alias"])
            self.assertEqual("image:1.2.3-rc.1" in tags, row["legacy_alias"])

    def test_branch_name_does_not_create_a_release_tag(self):
        tags = publish.image_tags("image", support.select("36.4.3"), self.revision, "main")
        self.assertEqual(len(tags), 2)

    def artifacts(self, folder):
        rows = support.catalog()["versions"]
        for index, row in enumerate(rows):
            path = folder / row["l4t"] / "jetson-build.json"
            path.parent.mkdir()
            path.write_text(json.dumps(dict(l4t=row["l4t"], jetpack=row["jetpack"],
                source_revision=self.revision, build="passed", digest="sha256:" + ("%064x" % index))))
        return rows

    def inspect(self, command, **kwargs):
        if command[-1] == "--raw":
            index = int(command[4].split("@sha256:")[1], 16)
            return json.dumps({"manifests": [dict(digest="sha256:" + ("%064x" % index),
                platform=dict(os="linux", architecture="arm64")),
                dict(platform=dict(os="unknown", architecture="unknown"))]})
        index = int(command[4].split("@sha256:")[1], 16)
        row = support.catalog()["versions"][index]
        return json.dumps({"config": {"Labels": {"org.opencontainers.image.revision": self.revision,
            "io.odin2.target.l4t": row["l4t"]}}})

    def invoke(self, folder, inspect=None):
        with mock.patch.object(publish.sys, "argv", ["publish", str(folder)]), \
                mock.patch.dict(os.environ, GITHUB_SHA=self.revision,
                    GITHUB_REPOSITORY="Owner/Repo", GITHUB_REF_NAME="v1.2.3", GITHUB_REF_TYPE="tag"), \
                mock.patch.object(publish.subprocess, "check_output", side_effect=inspect or self.inspect), \
                mock.patch.object(publish.subprocess, "run") as run:
            try:
                publish.main()
            finally:
                self.published = run.call_args_list

    def test_incomplete_matrix_never_publishes(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, "Missing"):
                self.invoke(pathlib.Path(tmp))
            self.assertEqual(self.published, [])

    def test_stale_last_artifact_prevents_all_tags(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            rows = self.artifacts(root)
            path = root / rows[-1]["l4t"] / "jetson-build.json"
            data = json.loads(path.read_text())
            data["source_revision"] = "old"
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "does not match"):
                self.invoke(root)
            self.assertEqual(self.published, [])

    def test_wrong_final_image_label_prevents_all_tags(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            self.artifacts(root)
            def inspect(command, **kwargs):
                result = self.inspect(command, **kwargs)
                return result.replace("36.5.2", "36.4.3") if command[-1] != "--raw" else result
            with self.assertRaisesRegex(ValueError, "labels"):
                self.invoke(root, inspect)
            self.assertEqual(self.published, [])

    def test_success_publishes_each_verified_digest_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            self.artifacts(root)
            self.invoke(root)
            self.assertEqual(len(self.published), 18)
            latest = [call for call in self.published if any(arg.endswith(":latest") for arg in call.args[0])]
            self.assertEqual(len(latest), 2)


if __name__ == "__main__":
    unittest.main()
