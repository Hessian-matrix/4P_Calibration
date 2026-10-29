# 用户指南

## 模式

4P Calibration 支持：

- 单相机内参与畸变标定；
- 多相机联合内外参标定；
- 多相机—IMU 预积分联合标定；
- 多相机—IMU 连续时间 B 样条联合标定。

## 启动

无参数运行进入 TUI：

```sh
four-p-calibration-<target>
```

指定 YAML 预填配置：

```sh
four-p-calibration-<target> --config calibration.yaml
```

无 TUI 批处理：

```sh
four-p-calibration-<target> --no-tui --config calibration.yaml
```

## 基本流程

1. 选择 ROS bag 与容器格式。
2. 扫描并选择相机、IMU topics。
3. 配置 AprilGrid/标定板几何。
4. 选择相机模型、内参策略和联合标定方法。
5. 通过启动检查后运行。
6. 检查重投影误差、求解诊断、内外参与轨迹结果。

## 输出

每次运行创建独立目录，包含输入快照、检测统计、求解诊断、标定结果、轨迹和可视化。联合求解结果默认标记为待人工验收；达到迭代或阻尼边界不会被静默伪装为收敛。

## 交互快捷键

- `Tab` / `Shift-Tab`：切换字段或只读视图；
- `Ctrl-P` / `Ctrl-N`：切换分区；
- `Enter`：编辑或确认；
- `Space`：勾选或展开；
- `Ctrl-S`：应用 YAML；
- `Ctrl+Y`：切换鼠标操作与终端文本复制；
- `F8` / `Ctrl-C`：确认取消当前运行；
- `Ctrl-Q`：确认退出。

## 完整性检查

下载 Release 后必须先验证 `SHA256SUMS`。若 Release 缺少任一目标平台文件或校验和文件，该 Release 不满足项目发布合同。
