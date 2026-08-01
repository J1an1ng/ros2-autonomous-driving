"""Regression tests for Unity's WSL graphics environment selection."""

import os
import subprocess
from pathlib import Path


LAUNCHER = Path(__file__).parents[1] / 'scripts' / 'run_unity.sh'


def configured_environment(kernel_release, extra_environment=None):
    """Return graphics variables produced for a synthetic kernel release."""
    environment = os.environ.copy()
    environment.pop('GALLIUM_DRIVER', None)
    environment.pop('MESA_D3D12_DEFAULT_ADAPTER_NAME', None)
    environment.update(extra_environment or {})

    command = (
        f'source "{LAUNCHER}"; '
        f'configure_unity_graphics "{kernel_release}"; '
        "printf '%s\\n%s\\n' \"${GALLIUM_DRIVER-}\" "
        '"${MESA_D3D12_DEFAULT_ADAPTER_NAME-}"'
    )
    result = subprocess.run(
        ['bash', '-c', command],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )
    return result.stdout.splitlines()


def test_wsl_defaults_to_nvidia_d3d12_renderer():
    """WSL defaults to the accelerated D3D12 NVIDIA path."""
    assert configured_environment('6.6.87.2-microsoft-standard-WSL2') == [
        'd3d12',
        'NVIDIA',
    ]


def test_explicit_renderer_settings_are_preserved():
    """Explicit advanced-user graphics settings take precedence."""
    assert configured_environment(
        '6.6.87.2-microsoft-standard-WSL2',
        {
            'GALLIUM_DRIVER': 'custom-driver',
            'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'custom-adapter',
        },
    ) == ['custom-driver', 'custom-adapter']


def test_native_linux_does_not_force_wsl_renderer():
    """Native Linux retains its system-selected OpenGL driver."""
    assert configured_environment('6.8.0-generic') == ['', '']
