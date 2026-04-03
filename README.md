# 🤖 MechanicalArm_Code_V4

基于 STM32F407 的六自由度机械臂控制系统（V4 版本）。

🎉 **
核心升级
**：
相比 V2 版本，本版本新增了 **
Xbox 手柄遥控模式
**，并将整个工程底层架构全面移植至 **
FreeRTOS
**，大幅提升了系统的实时性与多任务并发处理能力！

---

## 1. ⚙️ 硬件与驱动器配置

### 1.1 硬件来源
- **
电机类型
**：步进电机
- **
驱动器
**：淘宝 ZDT 步进电机驱动器
- **
BOM 表
**：请前往 Bilibili 搜索 UP主 [**
零一造物
**] 获取完整物料清单。

### 1.2 驱动器参数修改
请务必按照下表修改各关节驱动器的方向（`Dir`）设置，并将**
所有驱动器的 
`Serial`
 模式修改为 
`CAN`
**，其余参数保持默认。

| 关节编号 | 电机地址 | 运动方向 (Dir) | 通信方式 |
| :---: | :---: | :---: | :---: |
| 关节一 | 1 | `CCW` | CAN |
| 关节二 | 2 | `CCW` | CAN |
| 关节三 | 3 | `CW`  | CAN |
| 关节四 | 4 | `CW`  | CAN |
| 关节五 | 5 | `CCW` | CAN |
| 关节六 | 6 | `CCW` | CAN |


---

## 2. 🚀 使用教程

> ⚠️ **
危险警告 / WARNING
**
> **
上电前，务必手动将机械臂摆放到“零点位置”！
** 否则通电后可能会发生不可控的碰撞。零点姿态请参考下图：

<img width="955" height="1066" alt="机械臂零点位置" src="https://github.com/user-attachments/assets/fbc55990-da79-4110-940f-3ca5419e9fa2" />

### 2.1 PTP 控制模式 (点对点控制)
通过串口发送特定格式的字符串即可控制机械臂运动。
- **
指令格式
**：`PTP:x,y,z,alpha,beta,gamma\n`
  *
(注意：必须使用英文冒号和逗号，发送文本格式而非十六进制，以换行符结尾)
*
- **
归零示例
**：
  
text
PTP:177.63,0,176,0,1.57,3.14


### 2.2 手柄控制模式

#### 准备工作
1. 准备一个支持 **Xbox 模式** 的游戏手柄。
2. 将单片机连接至电脑，打开设备管理器查看端口号（例如 `COM5`）。
3. 修改配套 Python 脚本中的配置参数：

python
======== 脚本配置参数 ========
POR
T = 'COM5' 替换为你电脑上的实际串口号
BAUD = 115200
波特率
F
PS = 100 目标发送频率 100Hz
DEADZONE
= 0.15 摇杆死区设置
TCPPORT = 8888 # VOFA+ 连接的本地网络端口


> 💡 **进阶玩法 (VOFA+ 抓波形)**：
> 如果你想在运行 Python 脚本的同时，实时查看串口数据曲线，可以使用 **VOFA+**。
> 在 VOFA+ 中将数据接口改为 **TCP 客户端**，并按下图配置网络端口（与脚本中的 `TCP_PORT` 一致）：
> <img width="345" height="318" alt="VOFA配置" src="https://github.com/user-attachments/assets/db05b628-356d-4162-a51a-fd0669691093" />

#### 实际操作步骤
1. 打开手柄，切换至 Xbox 模式并连接电脑。
2. 运行 Python 控制脚本，观察控制台输出，确认连接成功：
   <img width="1116" height="390" alt="控制台输出" src="https://github.com/user-attachments/assets/5e3d2463-17f9-46e4-9836-e78a7ae5d21e" />
3. 拨动摇杆，观察控制台数据是否有变化。
4. **解锁机械臂**：单片机上电后默认为 `Standby` (待机) 模式，此时机械臂对摇杆无反应。**按下手柄的 `Y` 键**切换模式后，即可开始遥控！
   
   <img width="727" height="567" alt="手柄按键说明" src="https://github.com/user-attachments/assets/99ce7c80-ad5e-4e7f-8628-880b5e40992f" />
6. **模式切换**：如果想从手柄模式切换回 PTP 模式，只需停止 Python 脚本运行，然后打开串口助手正常发送 PTP 指令即可。

---

## 3. 🗺️ 后续方向 (To-Do)

- [ ] **3.1 硬件升级**：添加转接板，实现驱动器的级联通信（俗称“串串香”），简化走线。
  ![转接板设计](https://github.com/user-attachments/assets/9d710ba0-5541-4508-a0e5-de11092a2af6)

- [ ] **3.2 结构优化**：探索低成本的机械结构优化方案，提升机械臂刚性与精度。
  <img width="691" height="637" alt="机械结构优化" src="https://github.com/user-attachments/assets/47f8dd17-c257-4bca-a217-08e831ade15a" />

- [ ] **3.3 算法进阶**：系统学习并深入机器人控制算法（动力学规划等）。
  <img width="1906" height="1007" alt="机器人控制算法" src="https://github.com/user-attachments/assets/da750c29-7581-42cc-bc5f-f212747b85ea" />
