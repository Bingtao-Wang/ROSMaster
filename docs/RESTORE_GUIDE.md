# 源码恢复指南 (归还前必读)

> 本文档记录了所有对源码的修改，归还小车前必须按此恢复

---

## 修改记录

### 1. yahboomcar_nav/params/teb_nav_params.yaml

**文件路径:** `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/params/teb_nav_params.yaml`

**修改位置:** 第 324-333 行 (behavior_server 的 behavior_plugins 配置)

**原始内容:**
```yaml
    behavior_plugins: ["backup"]
    backup:
      plugin: "nav2_behaviors/BackUp"
      backup_speed: -1.0
      backup_duration: 1.5
      simulate_ahead_time: 2.0
```

**修改后内容:**
```yaml
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
      backup_speed: -1.0
      backup_duration: 1.5
      simulate_ahead_time: 2.0
    wait:
      plugin: "nav2_behaviors/Wait"
```

**修改原因:** 修复 bt_navigator 激活失败问题 (默认配置缺少 Spin 和 Wait 插件)

**修改时间:** 2026-03-24

---

## 恢复步骤

### 方法 1: 手动恢复 (推荐)

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/params
nano teb_nav_params.yaml
```

找到第 324 行，将：
```yaml
behavior_plugins: ["spin", "backup", "wait"]
spin:
  plugin: "nav2_behaviors/Spin"
backup:
  plugin: "nav2_behaviors/BackUp"
  backup_speed: -1.0
  backup_duration: 1.5
  simulate_ahead_time: 2.0
wait:
  plugin: "nav2_behaviors/Wait"
```

改回：
```yaml
behavior_plugins: ["backup"]
backup:
  plugin: "nav2_behaviors/BackUp"
  backup_speed: -1.0
  backup_duration: 1.5
  simulate_ahead_time: 2.0
```

保存后重新编译：
```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select yahboomcar_nav --symlink-install
```

### 方法 2: Git 恢复 (如果是 Git 仓库)

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav
git checkout params/teb_nav_params.yaml
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select yahboomcar_nav --symlink-install
```

---

## 验证恢复

恢复后检查文件内容：
```bash
grep -A 10 "behavior_plugins" ~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/params/teb_nav_params.yaml | head -7
```

应该只看到：
```
    behavior_plugins: ["backup"]
    backup:
      plugin: "nav2_behaviors/BackUp"
```

如果看到 `spin` 或 `wait`，说明还没恢复成功。

---

## 其他修改 (仅文档，无需恢复)

以下文件是新增的文档，不影响源码，可保留或删除：

- `/home/jetson/yahboomcar_ros2_ws/WBT_ROS2_WS/yahboomcar_examples_summary.md`
- `/home/jetson/yahboomcar_ros2_ws/WBT_ROS2_WS/1.gmapping_guide.md`
- `/home/jetson/yahboomcar_ros2_ws/WBT_ROS2_WS/2.nav2_teb_navigation_guide.md`
- `/home/jetson/yahboomcar_ros2_ws/WBT_ROS2_WS/RESTORE_GUIDE.md` (本文件)

地图文件 (可保留或删除):
- `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/maps/my_map.*`
- `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/maps/2.*`
