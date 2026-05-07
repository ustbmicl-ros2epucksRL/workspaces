import cv2
img = cv2.imread('/root/catkin_ws2/src/robosar/robosar_sim_bringup/worlds/stage_config/maps/tepper_sim_map.png', cv2.IMREAD_UNCHANGED)
print(f"{img.shape[1]}x{img.shape[0]}")
