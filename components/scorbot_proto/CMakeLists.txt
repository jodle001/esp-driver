# The Pi-to-ESP32 contract: nanopb runtime and generated messages copied from
# scorbot_ros2/scorbot_protocol (scripts/sync_protocol.sh --check keeps them identical),
# plus C ports of the COBS/CRC-16 framing.
idf_component_register(
    SRCS "src/cobs.c" "src/framing.c" "src/envelope.c"
         "third_party/nanopb/pb_common.c" "third_party/nanopb/pb_decode.c" "third_party/nanopb/pb_encode.c"
         "generated/scorbot/v1/scorbot.pb.c"
    INCLUDE_DIRS "include" "third_party/nanopb" "generated")
target_compile_definitions(${COMPONENT_LIB} PUBLIC PB_ENABLE_MALLOC=0 PB_FIELD_32BIT=0)
