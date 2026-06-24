"""Import-path bridge for running uam_ocp tests from the project root."""

from pathlib import Path

_INNER = Path(__file__).resolve().parent / "uam_ocp"
if str(_INNER) not in __path__:
    __path__.append(str(_INNER))
