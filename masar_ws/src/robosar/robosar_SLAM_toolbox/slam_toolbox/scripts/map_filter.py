import rospy
import numpy as np
from scipy import ndimage

from nav_msgs.msg import OccupancyGrid


pub = None

def filter_map(map_msg):
  # 地图分辨率与原点（当前函数中未直接使用，保留便于后续调参/扩展）
  scale = map_msg.info.resolution
  origin = [map_msg.info.origin.position.x,
            map_msg.info.origin.position.y]
  # 1) 把一维栅格数组恢复成二维矩阵，便于图像处理
  raw = np.reshape(
      map_msg.data, (map_msg.info.height, map_msg.info.width))
  # 2) 二值化：<50 视作空闲(0)，>=50 视作占据(100)
  #    注意：这里会把 unknown(-1) 也归到 0
  map_filt = np.copy(raw)
  map_filt = np.where(map_filt<50, 0, 100)
  # 3) 高斯平滑 + 再阈值，抑制小噪点、连通近邻占据区域
  map_filt = ndimage.gaussian_filter(map_filt, sigma=3)
  map_filt = np.where(map_filt > 6, 100, 0)
  # 4) 仅更新“原图已占据”的区域，避免大范围改写自由空间
  map_final = np.where(raw > 50, map_filt, raw)
  # 5) 回写并发布过滤后的地图
  map_msg.data = tuple(map_final.flatten())
  pub.publish(map_msg)



if __name__ == '__main__':
  # 订阅 slam_toolbox 输出地图，过滤后转发到 /map
  rospy.init_node('map_filter', anonymous=False)
  pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
  rospy.Subscriber("/slam_toolbox/map", OccupancyGrid, filter_map)
  rospy.spin()

