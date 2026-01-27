load("@rules_pkg//pkg:tar.bzl", "pkg_tar")

pkg_tar(
    name = "flat_rootfs",
    deps = [
        "//distroless:sh",
        "//distroless:passwd",
        "//distroless:group",
        "@noble//:noble",
        "//docker:python_runtime_tar",
        "//docker:nodes_runfiles_tar",
    ],
)
