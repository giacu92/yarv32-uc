# fence.i oracle (M7, core side): fence.i is a real pipeline flush + refetch
# from pc+4 (execute EX_FENCEI, redirect on retire). Checks that execution
# continues exactly after it in every fetch context that matters, and that it
# retires exactly once. 0x600D / 0xBAD @0x2000, failing check @0x2004.
S_SRCS := fencei_ops_test.S
include ../../common/sw_build.mk
