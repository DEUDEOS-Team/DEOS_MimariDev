import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_e2e_script_import_and_run_smoke():
    """
    E2E koşucunun en azından import edilip çalıştırılabildiğini doğrular.
    (Detaylı davranış zaten modül testlerinde var; burada kırılma yakalanır.)
    """

    from tools.e2e_race_scenarios import main

    main()

