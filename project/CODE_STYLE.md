# Team 20 code style

This document defines the style for Team 20 maintained code under `project/src`.
Generated files, bundled Unity assets, `simulation/src/json_nlohmann`, and
`perception/third_party` are third-party code and are not reformatted.

## Architecture

- Keep ROS nodes as adapters: declare parameters, connect topics and TF, schedule work,
  and translate between ROS messages and domain values.
- Put parsing, validation, geometry, planning, decision, and control rules in modules
  that can be tested without starting a ROS graph.
- Give each module one small interface and keep implementation details private.
- Preserve package ownership: perception observes, route management supplies route
  context, trajectory generation plans, decision making selects safe intent, and the
  controller produces vehicle commands.
- A topic has one clear semantic meaning and one authoritative publisher. In
  particular, `/car_command` must have exactly one publisher.

## Python

- Follow the ROS 2 Jazzy `ament_flake8` and `ament_pep257` profiles.
- Use four spaces, a maximum line length of 99, single quotes for strings, and
  triple double quotes for docstrings.
- Order imports as standard library, third-party/ROS, then package-local imports.
- Use `snake_case` for modules, functions, variables, parameters, and topics;
  `PascalCase` for classes; and `UPPER_CASE` for constants.
- Add docstrings where they explain a public interface, invariant, or non-obvious
  safety rule. Do not add comments that merely repeat the code.
- Keep executable entry points thin and put observable behaviour behind testable
  functions or classes.

## C++

- Follow the ROS 2 `ament_uncrustify` and `ament_cpplint` profiles.
- Use C++17, two-space indentation, and the brace placement produced by
  `ament_uncrustify`. Use `snake_case` for functions and variables, `PascalCase`
  for types, and trailing underscores for private data members.
- Prefer RAII, standard-library containers and smart pointers, `const` correctness,
  and explicit units in names such as `_m`, `_mps`, `_deg`, and `_sec`.
- Keep ROS callbacks short; move reusable calculations behind a small module
  interface and test that interface directly.

## ROS interfaces and traceability

- Topic, parameter, frame, and node names are public interfaces. Rename them only
  with a documented migration and an integration test.
- Put tunable values in YAML or declared parameters; document units and safe ranges.
- Log state transitions and safety fallbacks, but throttle repetitive diagnostics.
- Every behaviour or safety fix must include a regression test named after the
  observable behaviour it protects.

## Verification

From `project/`, run:

```bash
bash tools/check_code_quality.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```
