"""Concordance MATLAB-Python du cœur Pure Pursuit, cas par cas (section 3.2).

Rejoue chaque cas de data/concordance/pp_cas.csv (généré par exporter_concordance.m)
avec gen6323_core et compare les sorties.

TOLÉRANCE ANNONCÉE : 1e-9 (rad, m, m/s) en valeur absolue ; indices de segment égaux.
Les deux implantations exécutent les mêmes opérations en double précision ; les seuls
écarts attendus proviennent de l'arrondi des fonctions élémentaires (atan, hypot, sin),
de l'ordre de 1e-15. La marge de 1e-9 détecte toute erreur de formule ou de convention.

Lancer depuis python/ :  python3 -m pytest -v tests/test_concordance.py
"""
import csv
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gen6323_core.controllers import pure_pursuit   # noqa: E402
from gen6323_core.path import Progress, make_path, source_reference   # noqa: E402
from gen6323_core.vehicle import Limits, saturate   # noqa: E402

TOL = 1e-9
RACINE = os.path.join(os.path.dirname(__file__), '..', '..')
CAS = os.path.join(RACINE, 'data', 'concordance', 'pp_cas.csv')
SPIELBERG = os.path.join(RACINE, 'data', 'chemins_prepares', 'Spielberg_chemin.csv')


def charger_cas():
    if not os.path.isfile(CAS):
        pytest.skip(f'{CAS} absent : lancer exporter_concordance.m dans MATLAB')
    with open(CAS, newline='') as f:
        return list(csv.DictReader(f))


CHEMINS = {}


def chemin(nom):
    if nom not in CHEMINS:
        if nom == 'Spielberg':
            T = np.loadtxt(SPIELBERG, delimiter=',', skiprows=1)
            CHEMINS[nom] = make_path(T[:, 1:3], closed=True)   # même chemin que MATLAB
        else:
            CHEMINS[nom] = source_reference(nom)
    return CHEMINS[nom]


CAS_LISTE = charger_cas() if os.path.isfile(CAS) else []


@pytest.mark.parametrize('cas', CAS_LISTE, ids=[f"{c['chemin']}-{i}" for i, c in enumerate(CAS_LISTE)])
def test_cas(cas):
    f = {k: float(v) for k, v in cas.items() if k != 'chemin'}
    lim = Limits()
    C = chemin(cas['chemin'])
    prog = Progress(seg=int(f['seg_in']) - 1, s=f['s_in'], s_tot=f['s_tot_in'])   # MATLAB -> Python
    d_geom, prog_out, cible = pure_pursuit((f['x'], f['y'], f['psi']), f['v'], C, prog, lim.L,
                                           l0=f['l0'], tau=f['tau'])
    vc, dc, _ = saturate(f['v_des'], d_geom, f['vc_prev'], f['dc_prev'], lim)

    assert prog_out.seg + 1 == int(f['seg_out']), 'segment projeté différent'
    for nom, py, ml in [('s', prog_out.s, f['s_out']), ('s_tot', prog_out.s_tot, f['s_tot_out']),
                        ('delta_geom', d_geom, f['delta_geom']),
                        ('cible_x', cible[0], f['cible_x']), ('cible_y', cible[1], f['cible_y']),
                        ('vc', vc, f['vc']), ('dc', dc, f['dc'])]:
        assert abs(py - ml) <= TOL, f'{nom} : Python {py!r} contre MATLAB {ml!r} (écart {abs(py - ml):.2e})'


def test_jeu_complet():
    """Le jeu doit contenir les deux chemins et tous les cas attendus."""
    cas = charger_cas()
    assert len(cas) == 50
    assert {c['chemin'] for c in cas} == {'huit', 'Spielberg'}
