# Robomaster-2023赛季
***武汉理工大学Robowarrior战队电控部分代码***

- 目前三个兵种代码相同，后续在修改各自的代码后分别上传

> 项目框图由电控队员共同维护 （修改权限已开放）
> [RM 控制框图](https://boardmix.cn/app/share/CAE.CPfXyAwgASoQqDkYRqcqzDTPrO8fK9xiNzAFQAE/O2cjVd)

**共同要求**：

* [x] ~~*通过遥控器控制底盘运动*~~
* [x] ~~*通过遥控器控制云台运动*~~
* [x] ~~*子弹连发*~~
* [x] ~~*子弹单发*~~
* [ ] 接收上位机发送的云台数据并控制云台移动。
* [ ] 自瞄
* [ ] 小陀螺
* [ ] 电脑键鼠控制
* [ ] 功率限制分配
* [ ] 裁判系统通信

---

* 步兵
  * [x] ~~*爬坡功率测量*~~
* 英雄
  * [x] ~~*程序控制继电器，进而控制云台电源*~~

  * [x] ~~*英雄的发射机构，波弹电机调试*~~
    - 发射逻辑
      首先进行拨档（中->上，更换开关状态），打开摩擦轮，等待摩擦轮达到最大速度；
      在摩擦轮达到最大速度后，子弹会自动上膛，通过微动开关进行检测；
      检测到上膛后，停止拨弹轮电机，进入到等待射击状态，等待发射指令；
      拨档下拨，或者鼠标点击，切换到射击状态，控制电机转动36°角，发射子弹；
      子弹发射后，切换到发射完成状态，停止拨弹轮；
      此时拨弹轮停止，一段时间内识别子弹未上膛，切换到准备子弹状态，自动开始上膛子弹；
      如果拨片一直在下挡位，一段时间后，判定为连发模式。

* 哨兵
  * [x] ~~*接收上位机的底盘运动数据并控制移动*~~
  * [x] ~~*发送底盘里程计信息*~~
  * [ ] 判断遥控器是否开启，选择控制模式
  * [ ] 完成自动控制方式，提供api给算法组调用

---
### 任务协调

> 比赛时间 4.21-4.23 上海

- 算法组
  * [ ] 确定辅助瞄准方案
  * [ ] 确定哨兵控制API接口
<br>

- 机械组
  * [ ] 英雄pitch轴电机更换
  * [ ] 哨兵的安装调试
  * [ ] 导电滑环的安装


#### 任务1 小陀螺

> 3.23 至 ......

* [ ] 陀螺仪零飘校准
* [ ] 遥控器通道分配
* [x] ~~*加入小陀螺状态机*~~
* [x] ~~*小陀螺控制云台从陀螺仪获取反馈信息*~~
* [x] ~~*状态机切换保存控制量*~~
* [ ] 设置小陀螺旋转缓启动

#### 任务2 自瞄

> 3.23 至 ......

* [ ] 提出完整的自瞄控制方案，绘制流程图
* [ ] 在两周内完成自瞄模式的调试与切换

#### 任务3 哨兵调试

* [ ] 提出自动控制方案，绘制流程图

#### 任务4 线路连接

> 等待裁判系统验收......

* [ ] 导电滑环
* [ ] 裁判系统
