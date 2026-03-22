# AGENTS
# Repository guidance for autonomous coding agents.
# Scope: /home/yangy/Academics/Waveshare_dev

## Quick orientation
- This repo is a ROS 2 workspace under `ros2_ws/`.
- Python packages live in `ros2_ws/src/*_py`.
- C++ packages live in `ros2_ws/src/*_cpp` and `ros2_ws/src/depthai-ros/*`.
- There are no Cursor rules or Copilot rules in this repo.

## Environment setup (ROS 2)
- Always source the ROS 2 environment before build/test.
- Example (replace `<distro>` with your ROS 2 distro):
  - `source /opt/ros/<distro>/setup.bash`
- For local package overlays after build:
  - `source ros2_ws/install/setup.bash`

## Build commands
- Workspace build (recommended):
  - `colcon build --symlink-install`
- Build a single package:
  - `colcon build --symlink-install --packages-select ugv_bringup_py`
- Build with more output:
  - `colcon build --symlink-install --event-handlers console_direct+`

## Test commands
- Run all tests:
  - `colcon test`
- Run tests for a single package:
  - `colcon test --packages-select ugv_bringup_py`
- Show test output:
  - `colcon test --event-handlers console_direct+ --packages-select ugv_bringup_py`
- View test results summary:
  - `colcon test-result --verbose`

## Single-test commands
- Run a single test case via ctest (recommended):
  - `colcon test --packages-select ugv_bringup_py --ctest-args -R test_flake8`
- Run a single test file directly with pytest (when deps installed):
  - `pytest -q ros2_ws/src/ugv_bringup_py/test/test_flake8.py`
- Run all lint-related tests only:
  - `colcon test --packages-select ugv_bringup_py --ctest-args -R flake8|pep257|copyright`

## Lint commands
- Python linting is driven by ament lint tests:
  - `colcon test --packages-select ugv_bringup_py --ctest-args -R flake8`
  - `colcon test --packages-select ugv_bringup_py --ctest-args -R pep257`
- C++ linting is enabled in `ugv_bringup_cpp` via ament_lint_auto:
  - `colcon test --packages-select ugv_bringup_cpp`

## Formatting tools
- C++ formatting uses `.clang-format` in `ros2_ws/src/depthai-ros/`:
  - BasedOnStyle: Google
  - IndentWidth: 4
  - ColumnLimit: 160
  - PointerAlignment: Left
- Python formatting is not explicitly configured; follow existing style.

## Code style guidelines (Python)
- Indentation: 4 spaces, no tabs.
- Imports:
  - Group as: standard library, third-party, ROS 2 libs, local package imports.
  - Keep groups separated by a single blank line.
  - When editing existing files, avoid large reorders unless needed.
- Naming:
  - Classes: `CamelCase` (e.g., `SerialNode`).
  - Functions/methods/variables: `snake_case`.
  - Constants: `UPPER_SNAKE_CASE` when truly constant.
- Types:
  - Use type hints for public functions and message callbacks when clear.
  - Avoid over-annotating ROS 2 message fields (they are dynamic).
- Logging:
  - Use `self.get_logger().info/warn/error` in nodes.
  - Include actionable context in error messages.
- Error handling:
  - Prefer explicit `try/except` with narrow exception types.
  - For ROS nodes, fail gracefully and log errors rather than raising.
  - Avoid `assert` for runtime validation in production paths.
- ROS 2 patterns:
  - Declare parameters early in `__init__`.
  - Use `create_publisher`/`create_subscription` with explicit QoS depth.
  - Use `rclpy.init()` and `rclpy.shutdown()` in `main()`.
- File structure:
  - Keep node logic in `ugv_*_py/<package>/`.
  - Launch files live in `launch/` and use `.launch.py`.

## Code style guidelines (C++)
- Formatting follows the `.clang-format` in `ros2_ws/src/depthai-ros/`.
- Compiler warnings enabled: `-Wall -Wextra -Wpedantic` in `ugv_bringup_cpp`.
- Prefer `std::` types and RAII for resources.
- Use ROS 2 naming conventions for nodes and topics.

## Error handling and robustness
- Serial/UART code should:
  - Catch JSON parse errors and log the raw line.
  - Clear buffers on parse errors to resynchronize.
  - Use background threads carefully; avoid shared mutable state without locks.
- Avoid blocking calls (e.g., `input()`) in nodes unless explicitly required.

## Dependencies and packaging
- Python packages are `ament_python` (see `package.xml`).
- Tests rely on `ament_flake8`, `ament_pep257`, and `pytest`.
- C++ packages use `ament_cmake` and `ament_lint_auto`.

## Git hygiene for agents
- Do not modify generated files in `ros2_ws/build/`, `ros2_ws/install/`, or `ros2_ws/log/`.
- Keep changes confined to source packages under `ros2_ws/src/`.

## Common package names
- `ugv_bringup_py`: UART control, sensor publishing, teleop.
- `ugv_localization_py`: encoder/odometry utilities.
- `ugv_description`: URDF assets.
- `ugv_bringup_cpp`: minimal C++ package shell.
- `depthai-ros`: third-party camera stack with its own C++ style.

## When adding new code
- Mirror existing patterns and keep diffs minimal.
- Add tests under `test/` when behavior is non-trivial.
- Update `package.xml` and `setup.py` for new dependencies.

## If commands fail
- Ensure ROS 2 environment is sourced.
- Rebuild the workspace before running tests again.
- Check `ros2_ws/log/` for detailed failures.
