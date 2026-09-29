import pyrealsense2 as rs


def main():
    context = rs.context()
    devices = context.query_devices()

    print(f"RealSense camera count: {len(devices)}")
    for index, device in enumerate(devices):
        print(f"\n[{index}]")
        print(f"  Model:  {device.get_info(rs.camera_info.name)}")
        print(f"  Serial: {device.get_info(rs.camera_info.serial_number)}")


if __name__ == "__main__":
    main()
