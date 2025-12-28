load("@rules_img//img:layer.bzl", "image_layer", "layer_from_tar")
load("@rules_img//img:image.bzl", "image_manifest")
load("@rules_img//img:load.bzl", "image_load")
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

layer_from_tar(
    name = "python_runtime_layer",
    src = ":python_runtime_tar",
)

pkg_tar(
    name = "nodes_runfiles_tar",
    srcs = [
        "//nodes:talker",
        "//nodes:listener",
    ],
    package_dir = "/usr/lib/ros_src",
    include_runfiles = True,
)

layer_from_tar(
    name = "app_layer",
    src = ":nodes_runfiles_tar",
)

image_manifest(
    name = "app_image",
    base = "@ubuntu",
    layers = [
        ":python_runtime_layer",
        ":app_layer",
    ],
)

image_load(
    name = "load_app",
    image = ":app_image",
    tag = "app:latest",
)
