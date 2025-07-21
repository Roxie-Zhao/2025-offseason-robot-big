# 6941 Sanya Offseason 程序问题汇总

# Auto

1. 自动阶段卡在 GetCoral 与 DriveToBackoffPoint 中间

_原因_: 看到 Coral 的时间不够长卡状态，或者 DriveToPose 不能正确结束

_目前解决_: DriveToBackoffPoint 增加 2s Timeout, Tolerance 加大， CoralInSight 增加 0.15s TimeDelay

2. 自动阶段 L4 打不准

_原因_: 需要讨论

_目前解决_: 暂无

3. 自动阶段 Intake 撞墙

_原因_: 加速度过大，刹车距离不够

_目前解决_: 回退加速度设置

4. 自动阶段 Chase 不够有效，

_原因_: Coral 识别问题，同一个 Coral 识别成多个 Coral, 导致转角偏差或丢 Tag
    
_目前解决_: TimeDelay, 考虑

5. 自动蓝方转角不对

_原因_: 转角没有反转

_目前解决_: 完成反转，模拟 Debug 完成，还没有上机测试


# Teleop - Intaker & Superstructure

1. 自动阶段撞墙后，手动阶段 Intaker 不转，反转后恢复正常

_原因_: Roller 堵转（已确认）

_目前解决_: 自动阶段限制加速度，手动阶段与 Driver 沟通反转解决


# Teleop - ReefAim

1. L4 打飞

# Teleop - NetAim

1. NetAim 会导致 EE 打到 Net

_原因_: NetAim 过程缺少视觉更新导致位姿不准，NetAim X Setpoint 太过靠前， NetAim 球充气量连带射球电压改变

_目前解决_: 设置 NetAim X Setpoint 到 0.90, NetAim Voltage 到 -11.0V，限制加速度，限制 IMU


# Miscellaneous

1. Elastic

_原因_：屏幕适配

_解决_: 在 DriverStation 上重做，需要添加方式确认连接

2. Remove NT4 Logging



1. NetAimCommand 减少距离 OK
2. 自动 NearFlat OK
3. Disamiguate
4. L2, L3 不准
5. 爬升加延时 OK


