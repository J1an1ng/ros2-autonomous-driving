import importlib.util
import json
from pathlib import Path


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "simulation.launch.py"
SPEC = importlib.util.spec_from_file_location("simulation_launch", LAUNCH_FILE)
SIMULATION_LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIMULATION_LAUNCH)


def write_config(path, spawn_index):
    path.write_text(json.dumps({"spawnIndexData": {"spawnIndex": spawn_index}}))


def test_stage_unity_runtime_uses_selected_config_without_mutating_source(tmp_path):
    source_build = tmp_path / "source"
    source_data = source_build / "AD_Sim_Data"
    source_streaming = source_data / "StreamingAssets"
    source_streaming.mkdir(parents=True)
    (source_build / "AD_Sim.x86_64").write_bytes(b"executable")
    (source_build / "UnityPlayer.so").write_bytes(b"player")
    (source_data / "globalgamemanagers").write_bytes(b"game-data")

    source_config = source_streaming / "simulation_config.json"
    selected_config = tmp_path / "selected.json"
    write_config(source_config, 0)
    write_config(selected_config, 5)
    source_before = source_config.read_bytes()

    runtime_dir, runtime_executable = SIMULATION_LAUNCH.stage_unity_runtime(
        source_build,
        selected_config,
        runtime_parent=tmp_path / "runtimes",
    )

    runtime_config = (
        runtime_dir
        / "AD_Sim_Data"
        / "StreamingAssets"
        / "simulation_config.json"
    )
    assert runtime_executable.read_bytes() == b"executable"
    assert json.loads(runtime_config.read_text())["spawnIndexData"]["spawnIndex"] == 5
    assert source_config.read_bytes() == source_before
    assert (runtime_dir / "UnityPlayer.so").is_symlink()
    assert (runtime_dir / "AD_Sim_Data" / "globalgamemanagers").is_symlink()

    SIMULATION_LAUNCH.remove_unity_runtime(runtime_dir)
    assert not runtime_dir.exists()
