# Triple Wheel
目标追踪的三全向轮底盘设计

![image](https://github.com/user-attachments/assets/1e674473-47e3-4e2e-9d7c-0119644a414c)

## 1.ROI识别
机器人从出发区发车后，OpenMV在LAB色彩空间中使用目标颜色阈值，过滤得到二值化图像，在二值化图像中匹配面积最大的ROI区域。

![image](https://github.com/user-attachments/assets/fb2a0bae-666f-4174-9638-d238ad767c55)


## 2.控制解算
求出ROI中心与图像中心在图像坐标系下的笛卡尔坐标偏差。偏差经过位置式单极PID控制器得到控制量。

## 3.MCU通讯
控制量经过串口通信向STM32发送数据包。STM32校验帧格式后得到原始控制量。

## 4.底盘解算
经过线性空间映射，将控制量解算为机器人在本体坐标系下的速度与角速度。通过运动学逆解，得到每个电机的转速。
![image](https://github.com/user-attachments/assets/3bbff67d-d7be-4d66-9e6e-268341ece2d9)

## 5.底盘移动
目标转速经过速度环PID控制器输出PWM波脉冲宽度，产生PWM脉冲信号，使电机转动。

![image](https://github.com/user-attachments/assets/b920f453-b576-47e3-b305-9246719fd4b8)
