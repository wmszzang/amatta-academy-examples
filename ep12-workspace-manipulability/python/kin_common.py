"""앞 편의 정본 IK·FK·자코비안을 재사용하는 공통 진입점."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

def _load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

_ik = _load('ep10_kin', 'ep10-ik-analytic/python/ik2link.py')
_jac = _load('ep11_kin', 'ep11-jacobian-singularity/python/jacobian.py')
ik2 = _ik.ik2
fk = _ik.fk2
wrap = _ik.wrap
jacobian = _jac.jacobian
