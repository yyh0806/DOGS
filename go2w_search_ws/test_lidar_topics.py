#!/usr/bin/env python3
"""
Test LiDAR data acquisition from Go2W using correct official topic names.
Based on: https://support.unitree.com/home/zh/developer/LiDAR_service
"""

import sys
import time
import struct
import threading

from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_
from unitree_sdk2py.idl.unitree_go.msg.dds_._HeightMap_ import HeightMap_

INTERFACE = 'enp65s0'
TIMEOUT = 15  # seconds

results = {}
lock = threading.Lock()


def make_handler(topic_name, data_type):
    """Create a handler for a specific topic."""
    def handler(msg):
        with lock:
            if topic_name not in results:
                results[topic_name] = []
            results[topic_name].append(msg)

        count = len(results.get(topic_name, []))
        if count == 1:
            print(f"\n{'='*50}")
            print(f"GOT DATA on '{topic_name}' ({data_type})")
            print(f"{'='*50}")
            # Print key attributes
            for attr in dir(msg):
                if attr.startswith('_') or callable(getattr(msg, attr, None)):
                    continue
                try:
                    val = getattr(msg, attr)
                    val_str = str(val)
                    if len(val_str) > 300:
                        val_str = val_str[:300] + '...'
                    print(f"  {attr} = {val_str}")
                except:
                    pass

            # For PointCloud2, try to parse first few points
            if data_type == 'PointCloud2_':
                try:
                    parse_pointcloud(msg)
                except Exception as e:
                    print(f"  [PointCloud parse error: {e}]")

            # For HeightMap, show grid info
            if data_type == 'HeightMap_':
                try:
                    parse_heightmap(msg)
                except Exception as e:
                    print(f"  [HeightMap parse error: {e}]")

        elif count % 10 == 0:
            print(f"  [{topic_name}] received {count} messages so far...")

    return handler


def parse_pointcloud(msg):
    """Parse PointCloud2 data and show some points."""
    print(f"\n  --- PointCloud2 Details ---")
    print(f"  height={msg.height}, width={msg.width}")
    print(f"  point_step={msg.point_step}, row_step={msg.row_step}")
    print(f"  is_dense={msg.is_dense}, is_bigendian={msg.is_bigendian}")
    print(f"  data length={len(msg.data)} bytes")

    # Show fields
    print(f"  fields count={len(msg.fields)}")
    for f in msg.fields:
        print(f"    field: name={f.name}, offset={f.offset}, datatype={f.datatype}, count={f.count}")

    # Parse first few points (assuming xyz float32)
    if msg.point_step >= 12 and len(msg.data) >= msg.point_step:
        n_points = min(5, msg.width)
        fmt = '>' if msg.is_bigendian else '<'
        for i in range(n_points):
            offset = i * msg.point_step
            xyz = struct.unpack_from(fmt + 'fff', bytes(msg.data), offset)
            print(f"  point[{i}] = ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")


def parse_heightmap(msg):
    """Parse HeightMap and show info."""
    print(f"\n  --- HeightMap Details ---")
    print(f"  stamp={msg.stamp}, frame_id={msg.frame_id}")
    print(f"  resolution={msg.resolution} m/cell")
    print(f"  width={msg.width}, height={msg.height} cells")
    print(f"  origin=({msg.origin[0]:.2f}, {msg.origin[1]:.2f}) m")
    print(f"  data length={len(msg.data)}")

    # Count valid cells and show stats
    valid = [v for v in msg.data if abs(v) < 1e8]
    if valid:
        print(f"  valid cells={len(valid)}/{len(msg.data)}")
        print(f"  min={min(valid):.3f}, max={max(valid):.3f}, mean={sum(valid)/len(valid):.3f}")
    else:
        print(f"  no valid cells (all empty/1e9)")


def main():
    print(f"Initializing ChannelFactory on {INTERFACE}...")
    factory = ChannelFactory()
    factory.Init(0, INTERFACE)

    # Official topics from Unitree docs
    topics = [
        ('rt/utlidar/cloud', PointCloud2_, 'PointCloud2_'),
        ('rt/utlidar/cloud_deskewed', PointCloud2_, 'PointCloud2_'),
        ('rt/utlidar/height_map_array', HeightMap_, 'HeightMap_'),
    ]

    # Try to import PointStamped for range info
    try:
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_._PointStamped_ import PointStamped_
        topics.append(('rt/utlidar/range_info', PointStamped_, 'PointStamped_'))
    except Exception as e:
        print(f"Could not import PointStamped_: {e}")

    print(f"\nSubscribing to {len(topics)} official LiDAR topics...")
    for topic, msg_type, type_name in topics:
        try:
            ch = factory.CreateRecvChannel(topic, msg_type)
            ch.SetReader(handler=make_handler(topic, type_name))
            print(f"  [OK] {topic}")
        except Exception as e:
            print(f"  [FAIL] {topic}: {e}")

    print(f"\nWaiting {TIMEOUT}s for data...")
    for i in range(TIMEOUT):
        time.sleep(1)
        if results:
            sys.stdout.write(f"\r  {i+1}s - received data on {len(results)} topic(s)")
        else:
            sys.stdout.write(f"\r  {i+1}s - waiting...")
        sys.stdout.flush()

    # Final summary
    print(f"\n\n{'='*60}")
    print("FINAL RESULTS")
    print(f"{'='*60}")

    if not results:
        print("NO DATA received on ANY topic!")
        print("\nTroubleshooting:")
        print("  1. Make sure Go2W is powered on and LiDAR is spinning (you should hear it)")
        print("  2. Check ethernet connection: ping 192.168.123.161")
        print("  3. LiDAR auto-starts on boot - if not, may need to enable it")
        print("  4. HeightMap requires sport mode running (运控程序正常)")
    else:
        for topic, msgs in results.items():
            print(f"\n  {topic}:")
            print(f"    messages received: {len(msgs)}")
            print(f"    rate: ~{len(msgs)/TIMEOUT:.1f} Hz")
            msg = msgs[0]
            if isinstance(msg, PointCloud2_):
                print(f"    points per frame: {msg.width}")
            elif hasattr(msg, 'width') and hasattr(msg, 'resolution'):
                print(f"    grid: {msg.width}x{msg.height}, resolution={msg.resolution}")


if __name__ == '__main__':
    main()
