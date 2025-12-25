load("@rules_img//img:layer.bzl", "image_layer")
load("@rules_img//img:image.bzl", "image_manifest")
load("@rules_img//img:load.bzl", "image_load")

image_layer(
    name = "app_layer",
    srcs = {
        "/usr/bin/talker": "//nodes:talker",
        "/usr/bin/listener": "//nodes:listener",
    },
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
