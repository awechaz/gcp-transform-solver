from pathlib import Path

PROJECT_ROOT_DIR = Path(__file__).parent
BINARIES_DIR = PROJECT_ROOT_DIR / "build/bin"

# make tmp directory
TMP_DIR = PROJECT_ROOT_DIR / "tmp"
TMP_DIR.mkdir(exist_ok=True)