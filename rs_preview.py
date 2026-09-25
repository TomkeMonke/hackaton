import time
import numpy as np
import cv2
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 640, 480, 30

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

print(f"Stream started: {WIDTH}x{HEIGHT} @ {FPS} FPS")

frame_count = 0
t0 = time.time()

try:
    while True:
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        combined = np.hstack((color_image, depth_colormap))
        cv2.imshow("RealSense D415 - color | depth", combined)

        frame_count += 1
        if frame_count % 30 == 0:
            elapsed = time.time() - t0
            fps = frame_count / elapsed
            print(f"FPS: {fps:.1f}, color: {color_image.shape}, depth: {depth_image.shape}")

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Stopped.")
