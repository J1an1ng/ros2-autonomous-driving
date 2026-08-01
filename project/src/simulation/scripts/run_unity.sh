#!/usr/bin/env bash
set -euo pipefail

configure_unity_graphics() {
  local kernel_release="${1:-$(uname -r)}"

  if [[ "${kernel_release,,}" == *microsoft* ]]; then
    # WSLg exposes the Windows GPU through Mesa's D3D12 Gallium driver.  Mesa
    # can otherwise silently fall back to llvmpipe, which makes Unity render
    # entirely on the CPU.  Keep explicit user overrides intact.
    export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  fi
}

main() {
  local package_prefix unity_bin

  configure_unity_graphics
  package_prefix="$(ros2 pkg prefix simulation)"
  unity_bin="${package_prefix}/share/simulation/unity_sim/Build_Ubuntu/AD_Sim.x86_64"

  if [[ ! -x "${unity_bin}" ]]; then
    echo "Unity executable is missing or not executable: ${unity_bin}" >&2
    return 1
  fi

  if [[ "${GALLIUM_DRIVER:-}" == "d3d12" ]]; then
    echo "WSL Unity renderer: Mesa D3D12 (${MESA_D3D12_DEFAULT_ADAPTER_NAME:-system default})" >&2
  fi

  exec "${unity_bin}" "$@"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
