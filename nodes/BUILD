load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")
load("@rules_python//python:defs.bzl", "py_binary")

ros2_cpp_binary(
    name = "talker",
    srcs = ["talker.cpp"],
    deps = [
        "@fmt",
        "@ros2_rclcpp//:rclcpp",
        "//interface:cpp_rdelfin_msgs",
    ],
)

py_binary(
    name = "listener",
    srcs = ["listener.py"],
    deps = [
        "@ros2_rclpy//:rclpy",
        "//interface:py_rdelfin_msgs",
    ],
)
