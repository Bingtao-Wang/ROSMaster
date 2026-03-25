# WBT ROS2 Workspace - Development Guidelines

## Workspace Structure

```
/home/jetson/wbt_ws/WBT_ROS2_WS/
├── docs/                    # Documentation files
├── yahboomcar_packages/     # Modified ROS2 packages (editable)
├── yahboomcar_ws/          # Source code (READ-ONLY, symlink)
└── .gitignore
```

## Important Rules

### 1. Source Code Protection
- **`yahboomcar_ws/`** is a symlink to `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws`
- **DO NOT modify files in `yahboomcar_ws/` directly**
- This is the original source code and must remain unchanged

### 2. Making Changes
- To modify any package from `yahboomcar_ws/`:
  1. Copy the package to `yahboomcar_packages/`
  2. Make your changes in `yahboomcar_packages/`
  3. Commit and push changes from `yahboomcar_packages/`

### 3. Git Operations
- Git repository is located at `/home/jetson/wbt_ws/WBT_ROS2_WS/.git`
- You can run git commands directly in `/home/jetson/wbt_ws/WBT_ROS2_WS/`
- Only files in `yahboomcar_packages/` and `docs/` are tracked by git

## Current Packages

### In yahboomcar_packages/ (tracked by git):
- `yahboomcar_nav` - Navigation package (ROS2)
- `yahboomcar_ctrl` - Control package (ROS2)
- `yahboomcar_nav_rrt` - RRT exploration (converted to ROS2)

### In yahboomcar_ws/ (not tracked, read-only):
- Original source packages from yahboomcar

## Workflow Example

```bash
# If you need to modify a package from yahboomcar_ws:
cp -r yahboomcar_ws/src/package_name yahboomcar_packages/

# Make your changes in yahboomcar_packages/package_name

# Commit and push
git add yahboomcar_packages/package_name
git commit -m "Modify package_name"
git push origin main
```
