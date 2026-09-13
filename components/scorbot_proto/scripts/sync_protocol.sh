#!/usr/bin/env bash
# Copies the protocol contract from scorbot_ros2/scorbot_protocol into this component.
# The nanopb runtime and the generated code must be byte-identical on both sides:
#   scripts/sync_protocol.sh          copy
#   scripts/sync_protocol.sh --check  diff only (exit 1 on any difference), as CI runs it
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
src="${SCORBOT_PROTOCOL_DIR:-$here/../../../scorbot_ros2/scorbot_protocol}"
files=(
  "third_party/nanopb/LICENSE.txt" "third_party/nanopb/pb.h" "third_party/nanopb/pb_common.c"
  "third_party/nanopb/pb_common.h" "third_party/nanopb/pb_decode.c" "third_party/nanopb/pb_decode.h"
  "third_party/nanopb/pb_encode.c" "third_party/nanopb/pb_encode.h"
  "generated/scorbot/v1/scorbot.pb.h" "generated/scorbot/v1/scorbot.pb.c"
)
status=0
for f in "${files[@]}"; do
  if [[ "${1:-}" == "--check" ]]; then
    if ! diff -q "$src/$f" "$here/$f" >/dev/null; then echo "differs: $f"; status=1; fi
  else
    mkdir -p "$here/$(dirname "$f")"; cp "$src/$f" "$here/$f"; echo "copied $f"
  fi
done
exit $status
