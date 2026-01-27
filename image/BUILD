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

genrule(
    name = "disk_image",
    srcs = [
        ":flat_rootfs",
        "genimage.cfg",
    ],
    outs = ["disk.img"],
    cmd = """
        ROOTFS_DIR=$$(mktemp -d)
        TMP_DIR=$$(mktemp -d)
        OUTPUT_DIR=$$(mktemp -d)
        trap "rm -rf $$ROOTFS_DIR $$TMP_DIR $$OUTPUT_DIR" EXIT

        tar --overwrite -xf $(location :flat_rootfs) -C $$ROOTFS_DIR

        genimage \
            --config $(location genimage.cfg) \
            --rootpath $$ROOTFS_DIR \
            --tmppath $$TMP_DIR \
            --outputpath $$OUTPUT_DIR

        cp $$OUTPUT_DIR/disk.img $@
    """,
)
