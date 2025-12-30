import cv2
import numpy as np

# ========== 颜色阈值配置区（这里是你需要微调的地方） ==========
# 蓝色HSV阈值（根据你的需求修改这4个数值）
BLUE_LOWER = np.array([90, 120, 80])   # 下限：H, S, V
BLUE_UPPER = np.array([130, 255, 255]) # 上限：H, S, V

# 红色HSV阈值（红色分两个区间，同理修改）
RED_LOWER1 = np.array([0, 190, 140])
RED_UPPER1 = np.array([3, 255, 255])
RED_LOWER2 = np.array([177, 190, 140])
RED_UPPER2 = np.array([180, 255, 255])
# ============================================================

# 打开摄像头（0为默认设备）
cap = cv2.VideoCapture(0)
# 设置摄像头分辨率（适配多数设备）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 定义噪点过滤的核（固定参数，不用改）
kernel = np.ones((5, 5), np.uint8)

while True:
    # 读取摄像头画面
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败！")
        break

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ========== 蓝色识别逻辑 ==========
    blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    # 过滤蓝色掩码的噪点
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_result = cv2.bitwise_and(frame, frame, mask=blue_mask)

    # ========== 红色识别逻辑 ==========
    red_mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    red_mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    # 过滤红色掩码的噪点
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    # 绘制红色区域轮廓
    red_result = frame.copy()
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > 200:  # 过滤小区域
            cv2.drawContours(red_result, [cnt], -1, (0, 255, 0), 2)  # 绿色轮廓

    # ========== 显示窗口（自适应大小） ==========
    cv2.namedWindow('Original Frame', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Blue Detection', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Red Detection', cv2.WINDOW_NORMAL)
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Blue Detection', blue_result)
    cv2.imshow('Red Detection', red_result)

    # 按q键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
