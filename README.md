# wolf_control
hjj jy hzj对此项目亦有贡献

## 目前效果
ros1下正常控制整车（应该）
## todolist
- [ ] 轮子极性
- [ ] 实时性test
- [ ] 裁判系统test
- [ ] ros2 && ubuntu22实时内核
## 食用
```bash
mon launch rm_hw rm_hw.launch #整个硬件层
mon launch robot_state_controller load_controllers.launch #全关节控制
mon launch rm_chassis_controllers load_controllers.launch #底盘控制器
rosrun rm_dbus rm_dbus #遥控器
``` 
## 流程
![](https://github.com/guodongxiaren/ImageCache/raw/master/Logo/foryou.gif) 
![](https://github.com/wwwwwyt/control/blob/main/picture/process.jpg) 