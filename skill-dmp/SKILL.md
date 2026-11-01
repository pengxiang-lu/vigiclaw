---
name: skill-dmp
description: 通过现有 HTTP 脚本感知 ManiSkill 场景，使用已有 DMP 权重生成并执行机械臂抓取和放置轨迹。不使用 DMP 的单步目标位姿控制使用 skill-direct。
---

# 机械臂 DMP 控制

控制 ManiSkill-3 机械臂仿真环境，通过 HTTP API 与仿真服务端交互。

> ⚠️ **重要规则**:
> - 只能运行下方列出的脚本完成任务
> - **禁止生成或修改任何代码**
> - 仿真服务端必须运行在 `http://127.0.0.1:8000`

---

## 🚀 快速启动（新对话必读）

### 前置检查
```bash
# 检查仿真服务端是否运行
cd skill-dmp/scripts
uv run python health.py
```
如果仿真服务端未运行，**提醒用户手动启动**，不要私自启动。

---

## 📋 动作指令规范

### 7 维动作向量 `[x, y, z, roll, pitch, yaw, gripper]`

| 参数 | 范围 | 说明 |
|------|------|------|
| `x` | -0.4 ~ 0.2 | 前后（米，世界坐标系） |
| `y` | -0.3 ~ 0.3 | 左右（米，世界坐标系） |
| `z` | 0 ~ 0.6 | 上下（米，世界坐标系） |
| `roll` | **-3.14** | 固定值 |
| `pitch` | **0** | 固定值 |
| `yaw` | 物体 yaw 值 | 可变值 |
| `gripper` | 1.0/-1.0 | 1.0=打开，-1.0=闭合 |

> ⚠️ **关键约束**: `roll` 全程保持 `-3.14`，`pitch` 全程保持 `0`

---

## 🧭 方向坐标系

| 方向 | 轴   | 正方向 |
| ---- | ---- | ------ |
| 前   | x 轴 | +x     |
| 后   | x 轴 | -x     |
| 左   | y 轴 | +y     |
| 右   | y 轴 | -y     |
| 上   | z 轴 | +z     |
| 下   | z 轴 | -z     |

## 🎯 标准工作流

​	获取任务说明：uv run python get_task_instruction.py，按照获取的任务的说明完成任务。

### DMP 动作

动作统一通过 `dmp_action.py`，使用本 skill 的 `weights/` 中的 DMP 权重生成并逐点发送轨迹。
`--weights pick`、`--weights place`、`--weights push` 分别读取 `pick_weights.npy`、`place_weights.npy`、`push_weights.npy`；
使用前确认示范动作、分段边界和夹爪顺序适用于当前任务。视觉权重位于本 skill 的 `assets/yolo_seg.pt`。
动作脚本依赖项目的 `openclaw_control/dmp_core.py`，需保留项目目录关系。

默认起点为服务端实际 TCP 位姿，服务端须提供 `/arrival_status` 的 `tcp_action`。
`--midpoint` 设置分段终点，索引从 0 开始；N 段轨迹的中间点索引为 `0..N-2`，
最终终点使用 `--goal`。两者均支持 XYZ、XYZ+RPY 或完整七维动作，中间点需先写索引。
只传 XYZ 时保留相应示范端点的姿态和夹爪状态。夹爪按权重中的分段顺序切换，
切换时默认等待 0.5 秒并等待到达；发送失败或等待超时后停止发送。
`--time-scale 2` 将运动时长加倍；`--dry-run` 只生成不发送，显式提供 `--start` 时无需服务端。
`--save-trajectory` 只传文件名，结果保存到项目的 `openclaw_control/trajectory/`。

下方动作命令中的 `<...>` 为占位符，执行前应代入实际数值并计算高度。

### 技能1：看看桌上有什么东西

```python
~/anaconda3/bin/conda run -n yolo26 python object_detect.py
```

### 技能2：获取物体世界坐标

```python
# 获取物体世界坐标
~/anaconda3/bin/conda run -n yolo26 python get_object_pose.py
```

### 技能3：重置环境

```bash
uv run python reset.py
```

### 技能4：抓取物体 A

获取物体 A 的世界坐标 `[Ax, Ay, Az, roll, pitch, A_yaw]`：

```bash
~/anaconda3/bin/conda run -n yolo26 python get_object_pose.py
```

`--midpoint 0` 的 XYZ 使用物体坐标 `[Ax, Ay, Az]`，夹爪张开；
`--goal` 使用物体上方的坐标 `[Ax, Ay, Az+0.2]`，夹爪闭合。
姿态统一使用 `roll=-3.14`、`pitch=0`、`yaw=A_yaw`。

```bash
uv run python dmp_action.py --weights pick \
  --midpoint 0 <Ax> <Ay> <Az> -3.14 0 <A_yaw> 1.0 \
  --goal <Ax> <Ay> <Az+0.2> -3.14 0 <A_yaw> -1.0
```

### 技能5：把手上的物体A放到B处

获取目标 B 的世界坐标，并根据任务确定放置高度 `Zplace`：

```bash
~/anaconda3/bin/conda run -n yolo26 python get_object_pose.py
```

`--midpoint 0` 的 XYZ 使用放置坐标 `[Bx, By, Zplace]`，夹爪闭合；
`--goal` 使用放置点上方的坐标 `[Bx, By, Zplace+0.2]`，夹爪张开。
`Zplace` 为 TCP 放置高度，需根据支撑面高度和当前抓持位置确定，不能直接使用 B 的中心高度。
姿态统一使用 `roll=-3.14`、`pitch=0`，yaw 沿用手中物体 A 的 `A_yaw`。

```bash
uv run python dmp_action.py --weights place \
  --midpoint 0 <Bx> <By> <Zplace> -3.14 0 <A_yaw> -1.0 \
  --goal <Bx> <By> <Zplace+0.2> -3.14 0 <A_yaw> 1.0
```

### 技能6：推动方块

获取方块 A 的世界坐标 `[Ax, Ay, Az]` 和尺寸：

```bash
~/anaconda3/bin/conda run -n yolo26 python get_object_pose.py
```

`--midpoint 0` 使用方块背向推动方向一侧的 TCP 接触坐标；
`--midpoint 1` 使用接触点沿推动方向移动 `L` 后的坐标；
`--goal` 使用推动终点上方 0.2 米的坐标。推动时夹爪保持闭合。

下方模板中，`L > 0` 为推动距离，`d > 0` 为方块中心到 TCP 接触点的水平距离，
需根据方块沿推动方向的半宽及夹爪几何确定；`H` 为 TCP 接触高度。
执行前计算 `<...>` 中的表达式并替换为数值。

#### 向前推动（+X）

```bash
uv run python dmp_action.py --weights push \
  --midpoint 0 <Ax-d> <Ay> <H> -3.14 0 0 -1.0 \
  --midpoint 1 <Ax-d+L> <Ay> <H> -3.14 0 0 -1.0 \
  --goal <Ax-d+L> <Ay> <H+0.2> -3.14 0 0 -1.0
```

#### 向后推动（-X）

```bash
uv run python dmp_action.py --weights push \
  --midpoint 0 <Ax+d> <Ay> <H> -3.14 0 3.14159 -1.0 \
  --midpoint 1 <Ax+d-L> <Ay> <H> -3.14 0 3.14159 -1.0 \
  --goal <Ax+d-L> <Ay> <H+0.2> -3.14 0 3.14159 -1.0
```

#### 向左推动（+Y）

```bash
uv run python dmp_action.py --weights push \
  --midpoint 0 <Ax> <Ay-d> <H> -3.14 0 1.57080 -1.0 \
  --midpoint 1 <Ax> <Ay-d+L> <H> -3.14 0 1.57080 -1.0 \
  --goal <Ax> <Ay-d+L> <H+0.2> -3.14 0 1.57080 -1.0
```

#### 向右推动（-Y）

```bash
uv run python dmp_action.py --weights push \
  --midpoint 0 <Ax> <Ay+d> <H> -3.14 0 -1.57080 -1.0 \
  --midpoint 1 <Ax> <Ay+d-L> <H> -3.14 0 -1.57080 -1.0 \
  --goal <Ax> <Ay+d-L> <H+0.2> -3.14 0 -1.57080 -1.0
```

yaw 以当前 +X 推动示范的 `yaw=0` 为基准设置。当前 `push` 权重来自 +X 示范，
以上模板设置的是各段终点，不会把整条轨迹刚性旋转；其他方向执行前应预览生成路径，
确认接近及推动段适用。可用 `--dry-run` 生成而不发送，或通过 `dmp_replay.py` 查看轨迹对比图。
推动后重新获取方块坐标，检查实际位置是否达到目标。
