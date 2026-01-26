load("@rules_distroless//distroless:defs.bzl", "group", "passwd")
load("@rules_oci//oci:defs.bzl", "oci_image", "oci_load")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files", "strip_prefix", "pkg_attributes")

passwd(
    name = "passwd",
    entries = [
        {
            "uid": 0,
            "gid": 0,
            "home": "/root",
            "shell": "/bin/bash",
            "username": "r00t",
        },
        {
            "uid": 100,
            "gid": 65534,
            "home": "/home/_apt",
            "shell": "/usr/sbin/nologin",
            "username": "_apt",
        },
    ],
)

group(
    name = "group",
    entries = [
        {
            "name": "root",
            "gid": 0,
        },
        {
            "name": "_apt",
            "gid": 65534,
        },
    ],
)

pkg_tar(
    name = "sh",
    symlinks = {
        # needed as dpkg assumes sh is installed in a typical debian installation.
        "/bin/sh": "/bin/bash",
    },
)

oci_image(
    name = "app_image",
    architecture = select({
        "@platforms//cpu:arm64": "arm64",
        "@platforms//cpu:x86_64": "amd64",
    }),
    os = "linux",
    workdir = "/usr/lib/ros_src/launch_nodes.runfiles/_main",
    cmd = "command.txt",
    tars = [
        ":sh",
        ":passwd",
        ":group",
        "@noble//:noble",
        "//docker:python_runtime_tar",
        "//docker:nodes_runfiles_tar",
    ],
)

oci_load(
    name = "load_app",
    image = ":app_image",
    repo_tags = ["app_distroless:latest"],
)
