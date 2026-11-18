# Portable controller core: no ESP-IDF calls, so the same sources build on the host
# (test/host/Makefile) against the simulator's behaviour spec.
idf_component_register(
    SRCS "src/core.c" "src/acl.c" "src/link_mode.c" "src/pixels.c"
    INCLUDE_DIRS "include"
    REQUIRES scorbot_proto)
