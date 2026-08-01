#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"

source /opt/ros/jazzy/setup.bash
set -u

python_paths=(
  "${project_root}/src/controller/launch"
  "${project_root}/src/decision_making"
  "${project_root}/src/perception/launch"
  "${project_root}/src/route_manager"
  "${project_root}/src/simulation/launch"
  "${project_root}/src/simulation/test_scripts"
  "${project_root}/src/system_bringup"
  "${project_root}/src/trajectory_generator"
  "${project_root}/tools"
)

mapfile -t cpp_paths < <(
  find \
    "${project_root}/src/controller/include" \
    "${project_root}/src/controller/src" \
    "${project_root}/src/controller/test" \
    "${project_root}/src/perception/include" \
    "${project_root}/src/perception/src" \
    "${project_root}/src/perception/test" \
    -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o \
    -name '*.h' -o -name '*.hh' -o -name '*.hpp' \) \
    -print
)

echo 'Checking Python style and docstrings...'
python3 -m ament_flake8.main "${python_paths[@]}"
python3 -m ament_pep257.main "${python_paths[@]}"

echo 'Checking C++ style...'
python3 -m ament_cpplint.main "${cpp_paths[@]}"
python3 -m ament_uncrustify.main "${cpp_paths[@]}"

echo 'Code-quality checks passed.'
