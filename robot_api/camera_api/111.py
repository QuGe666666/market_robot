# -*- coding: gbk -*-
# -*- coding: utf-8 -*-
import cv2  # 导入 OpenCV 库

# 1. 初始化摄像头，0 表示默认摄像头
cap = cv2.VideoCapture(7)

# 2. 检查摄像头是否成功打开
if not cap.isOpened():
    print("错误：无法打开摄像头！")
    exit()

# 3. 循环读取并显示视频帧
while True:
    # 逐帧读取摄像头画面
    # ret 是一个布尔值，表示读取是否成功
    # frame 是读取到的图像帧
    ret, frame = cap.read()
    
    # 如果读取失败，则退出循环
    if not ret:
        print("无法接收帧，退出...")
        break
    
    # 在名为 'Camera Feed' 的窗口中显示当前帧
    cv2.imshow('Camera Feed', frame)
    
    # 等待 1 毫秒，并检测是否有按键按下
    # 如果按下 'q' 键，则退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 4. 释放资源
cap.release()  # 释放摄像头
cv2.destroyAllWindows()  # 关闭所有 OpenCV 创建的窗口