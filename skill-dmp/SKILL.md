---
name: skill-dmp
description: 使用已有 DMP 权重生成轨迹，并通过 HTTP API 在 ManiSkill 中执行机械臂操作；支持依据物体位置修改分段终点和目标点。直接发送单步 TCP 目标、不使用 DMP 的任务使用 skill-direct。
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

## DMP 操作流程

动作脚本为本 skill 的 `scripts/dmp_action.py`，DMP 权重存放在 `weights/`。
视觉分割权重位于 `assets/yolo_seg.pt`。本 skill 在 VigiClaw 项目中使用，
动作脚本依赖项目的 `openclaw_control/dmp_core.py`，保留两者的相对目录关系。
下面的命令在 `skill-dmp/scripts/` 下运行。

1. 运行 `uv run python health.py` 检查服务。未运行时提醒用户手动启动服务。
2. 用 `uv run python get_task_instruction.py` 获取任务说明；按需使用视觉脚本识别目标及世界坐标。
3. 从本 skill 的 `weights/` 中选择对应操作的已有权重，确认其示范任务、轨迹段和夹爪顺序匹配当前任务。权重文件名本身不保证适配任意物体或任务。
4. 依据目标位置设置 `--midpoint` 和 `--goal`，用 `dmp_action.py` 生成并发送轨迹。没有适用权重时说明缺少的示范，不用直接动作替代 DMP 操作。
5. 执行后重新观察场景，检查抓取或放置结果，再继续下一项操作。

### 感知与环境脚本

```bash
uv run python get_task_instruction.py
~/anaconda3/bin/conda run -n yolo26 python object_detect.py
~/anaconda3/bin/conda run -n yolo26 python get_object_pose.py
uv run python get_rgb_image.py --help
uv run python reset.py
```

`reset.py` 会重置场景，仅在任务需要重新开始时运行。

### 生成并执行 DMP 轨迹

```bash
# pick 自动定位 skill-dmp/weights/pick_weights.npy
uv run python dmp_action.py --weights pick

# 修改第一段终点和最终目标；XYZ 单位为米
uv run python dmp_action.py --weights pick \
  --midpoint 0 -0.20 0.05 0.30 --goal -0.10 0.05 0.45

# 只生成、不发送；显式指定起点时无需连接服务端
uv run python dmp_action.py --weights pick \
  --start -0.30 0 0.30 --dry-run --save-trajectory dmp_preview.npy
```

权重参数可传操作名称、完整文件名或权重文件路径。默认从服务端实际 TCP 位姿开始，
不会先移动到示范起点；指定起点后会先到达该起点再执行。
`--start`、`--goal` 支持 XYZ、XYZ+RPY 或完整七维动作；只指定 XYZ 时保留相应姿态和夹爪状态。
`--midpoint` 格式为 `索引 X Y Z [ROLL PITCH YAW [GRIPPER]]`，可重复指定。
若有 N 段轨迹，可修改的中间点索引为 `0..N-2`，最后一段终点用 `--goal`；
不指定的中间点沿用示范值。不能仅根据物体坐标猜测某个分段的语义。

`--time-scale 2` 将运动时长加倍。夹爪在段内保持不变，分段边界按权重中的顺序切换；
夹爪切换默认保持 0.5 秒，并等待到达后继续。发送失败或到达超时后停止发送。
服务端须提供 `/arrival_status` 的 `tcp_action` 字段，表示实际 TCP 位姿和夹爪指令。
`--save-trajectory` 只传文件名，输出统一保存到项目的 `openclaw_control/trajectory/`。
采集和权重学习使用项目的 `dmp_collect_trajectory.py`、`dmp_generate_weight.py`，
不在执行任务时改写动作脚本或临时生成代码。
