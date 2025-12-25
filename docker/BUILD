load("@rules_img//img:layer.bzl", "image_layer", "layer_from_tar")
load("@rules_img//img:image.bzl", "image_manifest")
load("@rules_img//img:load.bzl", "image_load")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")

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
        ":app_layer",
    ],
)

image_load(
    name = "load_app",
    image = ":app_image",
    tag = "app:latest",
)
