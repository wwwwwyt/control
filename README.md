# wolf_control
hjj jy hzj对此项目亦有贡献

## 目前效果
ros1下正常控制整车
## todo

## 食用
```bash
mon launch rm_hw rm_hw.launch#整个硬件层
mon launch robot_state_controller load_controllers.launch#全关节控制
mon launch rm_chassis_controllers load_controllers.launch #底盘控制器
rosrun rm_dbus rm_dbus #遥控器
``` 
