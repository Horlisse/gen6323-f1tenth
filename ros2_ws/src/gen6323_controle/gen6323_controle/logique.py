"""Logique de suivi Pure Pursuit, INDÉPENDANTE de ROS (section 3.2 de l'énoncé).

Aucun abonnement, aucune publication, aucune lecture de fichier : la classe reçoit une pose
et une vitesse, et rend une commande. Elle s'appuie sur le cœur gen6323_core, dont la
concordance avec MATLAB est vérifiée (tests/test_concordance.py, tolérance 1e-9).
"""
import time

from gen6323_core.controllers import pure_pursuit
from gen6323_core.path import Progress, project
from gen6323_core.sim import speed_profile
from gen6323_core.vehicle import Limits, saturate


class SuiviPP:
    """Un appel de commande() par nouvel état simulé."""

    def __init__(self, chemin, ref, l0, tau, v_cible, lim=Limits(), tours=1):
        self.chemin, self.ref = chemin, ref          # chemin suivi / référence source (mesures)
        self.l0, self.tau, self.v_cible, self.lim, self.tours = l0, tau, v_cible, lim, tours
        self.prog = Progress()                        # progression sur le chemin suivi (s = 0 au départ)
        self.prog_ref = Progress()                    # progression sur la référence source
        self.vc = 0.0                                 # consignes précédentes (repos au démarrage)
        self.dc = 0.0
        self.termine = False
        self.arret = False
        self.motif = ''

    def arret_securite(self, motif):
        """Arrêt prioritaire sur les limites de variation (section 2.1) : vitesse nulle immédiate."""
        if not self.arret:
            self.arret, self.motif = True, motif
        self.vc = 0.0

    def commande(self, x, y, psi, v):
        """Pose (x, y, psi) du milieu de l'essieu arrière et vitesse mesurée v.
        Rend un dictionnaire : consignes vc, dc, erreur et progression sur la référence,
        temps de calcul en secondes (temps réel)."""
        t0 = time.perf_counter()
        self.prog_ref, _, _, e_ref = project(self.ref, (x, y), self.prog_ref)
        if not self.termine and self.ref.closed and self.prog_ref.s_tot >= self.tours * self.ref.length:
            self.termine, self.motif = True, f'{self.tours} tour(s) complet(s)'
        if self.arret or self.termine:
            self.vc = 0.0                             # arrêt : vitesse nulle, braquage maintenu
            d_geom = self.dc
        else:
            d_geom, self.prog, _ = pure_pursuit((x, y, psi), max(v, 0.0), self.chemin, self.prog,
                                                self.lim.L, l0=self.l0, tau=self.tau)
            v_des = speed_profile(self.v_cible, self.chemin.length - self.prog.s_tot, self.chemin.closed)
            self.vc, self.dc, _ = saturate(v_des, d_geom, self.vc, self.dc, self.lim)
        return {'vc': self.vc, 'dc': self.dc, 'd_geom': d_geom, 'e_ref': abs(e_ref),
                's_tot_ref': self.prog_ref.s_tot, 'termine': self.termine, 'arret': self.arret,
                't_calcul': time.perf_counter() - t0}
