load("@rules_pkg//pkg:deb.bzl", "pkg_deb")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")


pkg_files(
    name = "ros_nodes_service",
    srcs = ["ros_nodes.service"],
    prefix = "usr/lib/systemd/system",
)

pkg_tar(
    name = "launch_nodes_tar",
    srcs = [":ros_nodes_service"],
    symlinks = {
        "/usr/bin/launch_nodes": "/usr/lib/ros_src/launch_nodes",
    },
    deps = ["//docker:nodes_runfiles_tar"],
)

pkg_deb(
    name = "launch-nodes-deb",
    architecture = select({
        "@platforms//cpu:arm64": "arm64",
        "@platforms//cpu:x86_64": "amd64",
    }),
    data = ":launch_nodes_tar",
    description = "Launch Nodes",
    maintainer = "Ricardo Delfin",
    package = "launch-nodes",
    postinst = "postinst.launch_nodes",
    prerm = "prerm.launch_nodes",
    version = "0.1.0",
)
