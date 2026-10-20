from pathlib import Path

PACKAGE_PATH = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_PATH.parents[1]
ASSET_PATH = REPO_ROOT / "assets"
PIPER_ASSET_DIR = ASSET_PATH / "agx_arm_urdf" / "piper"
PIPER_URDF_PATH = PIPER_ASSET_DIR / "urdf" / "piper_description.urdf"
