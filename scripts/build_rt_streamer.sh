#!/bin/bash
# Build rt_streamer (direct DRFL servoj_rt streaming, ROS-free).
set -eu

cd "$(dirname "$0")/.."

DRFL_INC=/ros2_ws/install/dsr_common2/include
DRFL_LIB=/ros2_ws/install/dsr_common2/lib/libDRFL.a

docker exec doosan_kos bash -lc "
  cd /kos_workspace &&
  g++ -std=c++17 -O2 -I${DRFL_INC} \
      scripts/rt_streamer.cpp \
      ${DRFL_LIB} \
      -lPocoFoundation -lPocoNet -lpthread -ldl \
      -o /tmp/rt_streamer
"
echo '✓ built: /tmp/rt_streamer (in container)'
