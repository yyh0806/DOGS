#!/usr/bin/env bash
set -euo pipefail

# Idempotently commission the external FAST_LIO workspace for latest-frame
# navigation.  This helper only patches/builds software; it never touches a
# service or robot command.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_FILE="$SCRIPT_DIR/patches/fast_lio_latest_frame.patch"
LIVOX_RELIABLE_PATCH_FILE="$SCRIPT_DIR/patches/fast_lio_livox_reliable_qos.patch"
BODY_PATCH_FILE="$SCRIPT_DIR/patches/fast_lio_body_cloud.patch"
BODY_QOS_PATCH_FILE="$SCRIPT_DIR/patches/fast_lio_body_cloud_qos.patch"
BOUNDED_BODY_PATCH_FILE="$SCRIPT_DIR/patches/fast_lio_bounded_body_cloud.patch"
ROTATING_BODY_PATCH_FILE="$SCRIPT_DIR/patches/fast_lio_rotating_body_sample.patch"
ANGULAR_BODY_PATCH_FILE="$SCRIPT_DIR/patches/fast_lio_angular_body_cloud.patch"
FASTLIO_WS="${FASTLIO_WS:-/home/nx/ws_livox}"
SOURCE="$FASTLIO_WS/src/FAST_LIO_ROS2/src/laserMapping.cpp"
EXECUTABLE="$FASTLIO_WS/install/fast_lio/lib/fast_lio/fastlio_mapping"
SOURCE_ONLY="${FASTLIO_SOURCE_ONLY:-0}"

die() {
  echo "prepare_fastlio_low_latency: $*" >&2
  exit 1
}

[[ -r "$PATCH_FILE" ]] || die "patch payload is missing: $PATCH_FILE"
[[ -r "$LIVOX_RELIABLE_PATCH_FILE" ]] || die "patch payload is missing: $LIVOX_RELIABLE_PATCH_FILE"
[[ -r "$BODY_PATCH_FILE" ]] || die "patch payload is missing: $BODY_PATCH_FILE"
[[ -r "$BODY_QOS_PATCH_FILE" ]] || die "patch payload is missing: $BODY_QOS_PATCH_FILE"
[[ -r "$BOUNDED_BODY_PATCH_FILE" ]] || die "patch payload is missing: $BOUNDED_BODY_PATCH_FILE"
[[ -r "$ROTATING_BODY_PATCH_FILE" ]] || die "patch payload is missing: $ROTATING_BODY_PATCH_FILE"
[[ -r "$ANGULAR_BODY_PATCH_FILE" ]] || die "patch payload is missing: $ANGULAR_BODY_PATCH_FILE"
[[ -f "$SOURCE" ]] || die "unknown FAST_LIO workspace: missing $SOURCE"

patched=0
if grep -Fq "auto latest_livox_qos = rclcpp::SensorDataQoS();" "$SOURCE" \
    && grep -Fq "if (!lidar_pushed)" "$SOURCE" \
    && grep -Fq "time_buffer.clear();" "$SOURCE"; then
  echo "FAST_LIO latest-frame patch already present"
elif grep -Fq "create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk)" "$SOURCE" \
    && grep -Fq "lidar_buffer.push_back(ptr);" "$SOURCE" \
    && grep -Fq "time_buffer.push_back(last_timestamp_lidar);" "$SOURCE"; then
  patch --batch --forward -d "$FASTLIO_WS" -p1 < "$PATCH_FILE" \
    || die "known stock source did not accept the pinned patch"
  patched=1
else
  die "unknown FAST_LIO source revision; refusing an unverified patch"
fi

# The commissioned Livox driver publishes RELIABLE.  Keep a latest-only
# reader, but match reliability; the NX FastDDS build can otherwise deliver a
# single sample and then stall after repeated participant restarts.
if grep -Fq "latest_livox_qos.reliable();" "$SOURCE"; then
  echo "FAST_LIO reliable latest-frame Livox QoS patch already present"
elif grep -Fq "latest_livox_qos.best_effort();" "$SOURCE"; then
  patch --batch --forward -d "$FASTLIO_WS" -p1 < "$LIVOX_RELIABLE_PATCH_FILE" \
    || die "known FAST_LIO latest-frame QoS did not accept reliable patch"
  patched=1
else
  die "unknown FAST_LIO latest-frame QoS revision; refusing an unverified patch"
fi

# Publish the body cloud without also enabling the expensive world cloud.
# This gives obstacle processing one efficient PointCloud2 source and avoids a
# second subscriber on the driver's large raw CustomMsg stream.
if grep -Fq "if (scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);" "$SOURCE"; then
  echo "FAST_LIO independent body-cloud patch already present"
elif grep -Fq "if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);" "$SOURCE"; then
  patch --batch --forward -d "$FASTLIO_WS" -p1 < "$BODY_PATCH_FILE" \
    || die "known FAST_LIO body-cloud source did not accept the pinned patch"
  patched=1
else
  die "unknown FAST_LIO body-cloud source revision; refusing an unverified patch"
fi

if grep -Fq "latest_body_cloud_qos.best_effort();" "$SOURCE"; then
  echo "FAST_LIO latest-frame body-cloud QoS patch already present"
elif grep -Fq 'create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20)' "$SOURCE"; then
  patch --batch --forward -d "$FASTLIO_WS" -p1 < "$BODY_QOS_PATCH_FILE" \
    || die "known FAST_LIO body-cloud publisher did not accept the pinned QoS patch"
  patched=1
else
  die "unknown FAST_LIO body-cloud publisher revision; refusing an unverified patch"
fi

if grep -Fq "angular_stratified_body_cloud" "$SOURCE" \
    && grep -Fq "azimuth_bins * elevation_bins" "$SOURCE"; then
  patch --batch --reverse -d "$FASTLIO_WS" -p1 < "$ANGULAR_BODY_PATCH_FILE" \
    || die "known angular body-cloud source did not accept safe rollback patch"
  echo "FAST_LIO angular sampler rolled back to compact fixed-phase 1000"
  patched=1
elif grep -Fq "constexpr int max_body_cloud_points = 1000;" "$SOURCE" \
    && grep -Fq "source_index = 0; source_index < source_size" "$SOURCE"; then
  echo "FAST_LIO bounded body-cloud patch already present"
elif grep -Fq "constexpr int max_body_cloud_points = 1000;" "$SOURCE" \
    && grep -Fq "body_cloud_phase++ % stride" "$SOURCE"; then
  patch --batch --reverse -d "$FASTLIO_WS" -p1 < "$ROTATING_BODY_PATCH_FILE" \
    || die "known rotating body-cloud source did not accept safe rollback patch"
  echo "FAST_LIO rotating sampling rolled back to compact fixed-phase 1000"
  patched=1
elif { grep -Fq "constexpr int max_body_cloud_points = 3000;" "$SOURCE" \
      || grep -Fq "constexpr int max_body_cloud_points = 6000;" "$SOURCE" \
      || grep -Fq "constexpr int max_body_cloud_points = 9000;" "$SOURCE"; } \
    && grep -Fq "source_index += stride" "$SOURCE"; then
  # Larger trials saturated conversion/DDS and made Nav2 lose TF. Return to
  # a compact fixed-phase budget. This keeps the Livox raw stream single-reader
  # while preventing optional obstacle serialization from stalling pose.
  sed -i -E 's/constexpr int max_body_cloud_points = (3000|6000|9000);/constexpr int max_body_cloud_points = 1000;/' "$SOURCE"
  echo "FAST_LIO body-cloud limit migrated to compact fixed-phase 1000"
  patched=1
elif grep -Fq "int size = feats_undistort->points.size();" "$SOURCE" \
    && grep -Fq "for (int i = 0; i < size; i++)" "$SOURCE"; then
  patch --batch --forward -d "$FASTLIO_WS" -p1 < "$BOUNDED_BODY_PATCH_FILE" \
    || die "known FAST_LIO body-cloud function did not accept the bounded patch"
  patched=1
else
  die "unknown FAST_LIO body-cloud function revision; refusing an unverified patch"
fi

grep -Fq "latest_livox_qos.keep_last(1);" "$SOURCE" \
  || die "patched source is missing keep-last 1"
grep -Fq "latest_livox_qos.reliable();" "$SOURCE" \
  || die "patched source is missing reliable latest-frame QoS"
grep -Fq "if (scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);" "$SOURCE" \
  || die "patched source is missing independent body-cloud output"
grep -Fq "latest_body_cloud_qos.best_effort();" "$SOURCE" \
  || die "patched source is missing best-effort body-cloud QoS"
grep -Fq "constexpr int max_body_cloud_points = 1000;" "$SOURCE" \
  || die "patched source is missing bounded body-cloud output"
grep -Fq "source_index = 0; source_index < source_size" "$SOURCE" \
  || die "patched source is missing stable fixed-phase body-cloud sampling"

if [[ "$SOURCE_ONLY" == "1" ]]; then
  echo "FASTLIO_SOURCE_ONLY=1: source prepared; build intentionally skipped"
  exit 0
fi
[[ "$SOURCE_ONLY" == "0" ]] \
  || die "FASTLIO_SOURCE_ONLY must be 0 or 1"

if [[ "$patched" == "1" || ! -x "$EXECUTABLE" || "$SOURCE" -nt "$EXECUTABLE" ]]; then
  command -v colcon >/dev/null 2>&1 || die "colcon is unavailable"
  [[ -r /opt/ros/humble/setup.bash ]] \
    || die "ROS Humble setup is unavailable"
  # ROS Humble's generated setup scripts legitimately probe variables that
  # may be unset.  Keep strict mode for this helper, but suspend nounset only
  # for the setup script and restore it before invoking the build.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  (
    cd "$FASTLIO_WS"
    colcon build --packages-select fast_lio --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  )
else
  echo "FAST_LIO executable is newer than the commissioned source; build skipped"
fi

[[ -x "$EXECUTABLE" ]] || die "FAST_LIO executable missing after preparation"
echo "FAST_LIO low-latency preparation complete"
