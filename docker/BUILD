package(default_visibility = ["//visibility:public"])

load("@rules_oci//oci:defs.bzl", "oci_image", "oci_load")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files", "strip_prefix", "pkg_attributes")

PY_RUNTIME_BINARIES = [
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/2to3",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/2to3-3.10",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/idle3",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/idle3.10",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/pip",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/pip3",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/pip3.10",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/pydoc3",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/pydoc3.10",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/python",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/python3",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/python3-config",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/python3.10",
        "@python_3_10_x86_64-unknown-linux-gnu//:bin/python3.10-config",
]

# Separate bin files for executable permissions
filegroup(
    name = "python_bins",
    srcs = PY_RUNTIME_BINARIES,
)

# Executable Python binaries with 0755 permissions
pkg_files(
    name = "python_runtime_bins",
    srcs = [":python_bins"],
    prefix = "/usr/local/python",
    strip_prefix = strip_prefix.from_pkg(),
    attributes = pkg_attributes(
        mode = "0755",
    ),
)

# Non-executable Python runtime files (libraries, headers, etc.) with 0644 permissions
pkg_files(
    name = "python_runtime_libs",
    srcs = ["@python_3_10_x86_64-unknown-linux-gnu//:files"],
    prefix = "/usr/local/python",
    strip_prefix = strip_prefix.from_pkg(),
    excludes = PY_RUNTIME_BINARIES,
)

pkg_tar(
    name = "python_runtime_tar",
    srcs = [
        ":python_runtime_libs",
        ":python_runtime_bins",
    ],
    symlinks = {
        "/usr/bin/python3": "/usr/local/python/bin/python3",
        "/usr/bin/python3.10": "/usr/local/python/bin/python3.10",
    },
)

pkg_tar(
    name = "nodes_runfiles_tar",
    srcs = [
        "//nodes:talker",
        "//nodes:listener",
        "//nodes:launch_nodes",
    ],
    package_dir = "/usr/lib/ros_src",
    include_runfiles = True,
)

oci_image(
    name = "app_image",
    base = "@ros_base",
    workdir = "/usr/lib/ros_src/launch_nodes.runfiles/_main",
    cmd = "command.txt",
    tars = [
        ":python_runtime_tar",
        ":nodes_runfiles_tar",
    ],
)

oci_load(
    name = "load_app",
    image = ":app_image",
    repo_tags = ["app:latest"],
)
