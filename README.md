# 激光追踪系统

## 展示

  [点我观看演示视频](https://github.com/lianga6/Laser-tracking-system/issues/1#issue-2771810727)
<video controls>
  <source src="https://github.com/lianga6/Laser-tracking-system/issues/1#issue-2771810727" type="video/mp4">
  Your browser does not support the video tag.
</video>
  
## 背景
该项目是2024年全国大学生电子设计大赛的比赛项目，这一部分主要是我来完成的。

## 要求
该系统发出的绿光必须快速紧紧跟随由使用者发出的红色激光

## 思路
该系统用的主控是openmv（由stm32单片机改造而来），在整个视野中openmv的视野可以说是由一个个像素点所构成，我将绿色激光笔固定在摄像头下方的位置，跟随摄像头一起运动，这样的话绿色激光点在openmv的视野中所处的位置就是固定的，当使用者发出红外激光时，就可立马锁定红色激光点在openmv中的视野位置，然后就非常简单，上下位置偏差多少就补多少，左右偏差多少就左右补多少
，但如何让整个跟随系统更加丝滑呢，这就用到pid算法，积分是加入了“过去”的作用，而微分起到预测未来的作用，对过去，现在，未来进行合理的加权，便能让整个跟随系统像视频中所展示的那样丝滑。还有一些细节处理可看源码。

