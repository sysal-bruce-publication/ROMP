load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "point",
    srcs = ["point.cpp"],
    hdrs = ["point.h"],
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cpp"],
    hdrs = ["obstacle.h"],
    deps = [":point"],
)

cc_library(
    name = "wind",
    srcs = ["wind.cpp"],
    hdrs = ["wind.h"],
    deps = [":point"],
)

cc_library(
    name = "sensor",
    srcs = ["sensor.cpp"],
    hdrs = ["sensor.h"],
    deps = [":point"],
)

cc_library(
    name = "drone",
    srcs = ["drone.cpp"],
    hdrs = ["drone.h"],
    deps = [
	":sensor",
	":obstacle",
	":wind"
    ],
)

cc_binary(
    name = "cpp_ortools",
    srcs = ["main.cpp"],
    deps = [
	":drone",
	"@com_google_ortools//ortools/base",
	"@com_google_ortools//ortools/constraint_solver:routing",
	"@com_google_ortools//ortools/constraint_solver:routing_flags",
    ],
)
