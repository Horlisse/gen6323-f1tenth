"""Nœud ROS 2 de suivi Pure Pursuit (phase 2).

Entrées  : /ego_racecar/odom (pose, vitesse), /ego_racecar/collision, /ego_racecar/lap_count,
           /gen6323/arret (std_msgs/Bool : arrêt à la demande).
Sortie   : /drive (ackermann_msgs/AckermannDriveStamped).
Mode synchrone (async_mode: false, annexe C.3) : une commande par NOUVEL état simulé, publications
répétées du même horodatage ignorées ; l'échange démarre à la première odométrie.
Surveillance : odométrie périmée détectée sur une horloge RÉELLE (STEADY), qui avance même si
la simulation est bloquée. Arrêt explicite à la fermeture du nœud (Ctrl + C).
Protections de validité : réinitialisation externe (/initialpose), autre émetteur sur /drive et
collision déjà active au départ déclarent l'essai INVALIDE (docs/protocole_essais_ros2.md).
Journal CSV : horodatage simulé, pose, vitesse mesurée, consignes envoyées (vc, delta_c),
collision (mémorisée), progression, tours, temps de calcul.
"""
import csv
import math
import os
from datetime import datetime

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Int32

from gen6323_core.path import make_path
from gen6323_core.vehicle import Limits

from .logique import SuiviPP


def lire_xy(fichier):
    """Colonnes x, y d'un CSV. Lignes de commentaire (#) et en-tête texte ignorées.
    Chemin préparé (en-tête s_m,x_m,y_m,psi_rad) : colonnes 2-3 ; ligne centrale source : 1-2."""
    lignes, prepare = [], False
    with open(os.path.expanduser(fichier)) as f:
        for ligne in f:
            ligne = ligne.strip()
            if not ligne or ligne.startswith('#'):
                continue
            if ligne.startswith('s_m'):
                prepare = True
                continue
            lignes.append([float(v) for v in ligne.split(',')])
    T = np.array(lignes)
    return T[:, 1:3] if prepare else T[:, 0:2]


def lacet(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NoeudSuiviPP(Node):
    def __init__(self):
        super().__init__('suivi_pp')
        p = self.declare_parameter
        p('chemin_csv', '~/gen6323-f1tenth/data/chemins_prepares/Spielberg_chemin.csv')
        p('ref_csv', '~/gen6323_data/f1tenth_racetracks/Spielberg/Spielberg_centerline.csv')
        p('l0', 0.3)
        p('tau', 0.2)
        p('v_cible', 2.0)
        p('tours', 1)
        p('delai_odom_s', 0.5)
        p('journal_dir', '~/gen6323-f1tenth/resultats')
        p('essai', 'essai')
        g = lambda n: self.get_parameter(n).value   # noqa: E731

        chemin = make_path(lire_xy(g('chemin_csv')), closed=True)
        ref = make_path(lire_xy(g('ref_csv')), closed=True)
        self.logique = SuiviPP(chemin, ref, g('l0'), g('tau'), g('v_cible'), Limits(), g('tours'))
        self.delai_odom = g('delai_odom_s')
        self.get_logger().info(
            f'Chemin suivi : {chemin.nseg} segments, {chemin.length:.3f} m ; référence S0 = {ref.length:.3f} m ; '
            f'l0 = {g("l0")} m, tau = {g("tau")} s, v_cible = {g("v_cible")} m/s')

        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_odom, 10)
        self.create_subscription(Bool, '/ego_racecar/collision', self.cb_collision, 10)
        self.create_subscription(Int32, '/ego_racecar/lap_count', self.cb_tours, 10)
        self.create_subscription(Bool, '/gen6323/arret', self.cb_arret, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.cb_reinitialisation, 10)

        # Horloge réelle pour la surveillance : elle avance même si la simulation est bloquée.
        self.horloge_reelle = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.cb_surveillance, clock=self.horloge_reelle)

        self.dernier_stamp = None
        self.derniere_odom_reelle = None
        self.nb_etats = 0                 # nombre d'états simulés distincts reçus
        self.fige = False                 # fin d'essai : véhicule arrêté, plus aucune publication
        self.invalide = ''                # motif d'invalidité ('' = essai valide)
        self.tic_surveillance = 0
        self.debut_reel = self.horloge_reelle.now()
        self.dernier_amorcage = None
        self.collision = False
        self.nb_tours = 0
        self.fin_annoncee = False
        self.dernier_header = None
        self.mesures = {'t': [], 'e': [], 'vc': [], 'dc': [], 'tc': []}

        dossier = os.path.expanduser(g('journal_dir'))
        os.makedirs(dossier, exist_ok=True)
        nom = f'ros2_{g("essai")}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        self.chemin_journal = os.path.join(dossier, nom)
        self.f_journal = open(self.chemin_journal, 'w', newline='')
        self.journal = csv.writer(self.f_journal)
        self.journal.writerow(['t_sim_s', 'x_m', 'y_m', 'psi_rad', 'v_mesuree_mps', 'vc_mps', 'dc_rad',
                               'e_ref_m', 's_tot_ref_m', 'collision', 'nb_tours', 'etat', 't_calcul_ms'])
        self.get_logger().info(f'Journal : {self.chemin_journal}')

    # ------------------------------------------------------------------ rappels
    def cb_odom(self, msg):
        if self.fige:
            return                                        # essai terminé : la simulation reste figée
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.dernier_stamp:
            return                                        # même état simulé : pas de nouvelle commande
        self.dernier_stamp = stamp
        maintenant = self.horloge_reelle.now()
        if self.nb_etats == 1:
            # Premier pas réellement simulé : la surveillance s'arme à partir d'ici. Le premier
            # pas peut durer plusieurs secondes (compilation Numba du simulateur au premier appel).
            delai = (maintenant - self.derniere_odom_reelle).nanoseconds * 1e-9
            self.get_logger().info(f'Premier pas simulé reçu après {delai:.2f} s ; surveillance armée.')
        self.nb_etats += 1
        self.derniere_odom_reelle = maintenant
        self.dernier_header = msg.header
        pose = msg.pose.pose
        x, y, psi = pose.position.x, pose.position.y, lacet(pose.orientation)
        v = msg.twist.twist.linear.x
        r = self.logique.commande(x, y, psi, v)
        self.publier(r['vc'], r['dc'], msg.header.stamp)

        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        etat = 'arret: ' + self.logique.motif if r['arret'] else ('termine' if r['termine'] else 'suivi')
        self.journal.writerow([f'{t:.9f}', f'{x:.9f}', f'{y:.9f}', f'{psi:.9f}', f'{v:.6f}', f'{r["vc"]:.9f}',
                               f'{r["dc"]:.9f}', f'{r["e_ref"]:.6f}', f'{r["s_tot_ref"]:.6f}', int(self.collision),
                               self.nb_tours, etat, f'{1e3 * r["t_calcul"]:.4f}'])
        if not (r['termine'] or r['arret']):
            for k, val in zip(('t', 'e', 'vc', 'dc', 'tc'), (t, r['e_ref'], r['vc'], r['dc'], r['t_calcul'])):
                self.mesures[k].append(val)
        if (r['termine'] or r['arret']) and not self.fin_annoncee:
            self.fin_annoncee = True
            self.resume(etat)
        if (r['termine'] or r['arret']) and abs(v) < 0.05:
            # Véhicule arrêté : on cesse de publier. En mode synchrone, la simulation n'avance plus.
            self.fige = True
            self.f_journal.flush()
            self.get_logger().info(f'Véhicule arrêté (v = {v:.3f} m/s) : fin des publications. '
                                   'Ctrl + C pour fermer le nœud.')

    def cb_collision(self, msg):
        if msg.data and not self.collision:
            self.collision = True                         # mémorisée jusqu'à la fin de l'essai (annexe C.3)
            if self.nb_etats == 0:
                self.invalider('collision deja active au depart',
                               'Collision DÉJÀ ACTIVE au départ : relancer le simulateur avant l’essai.')
            else:
                self.invalider('collision', 'COLLISION détectée : essai invalide, arrêt de sécurité.')

    def cb_reinitialisation(self, msg):
        if not self.fige:
            p = msg.pose.pose.position
            self.invalider('reinitialisation externe',
                           f'Réinitialisation de la pose reçue sur /initialpose ({p.x:.2f} ; {p.y:.2f}) : '
                           'essai invalide (outil de pose de Foxglove ?).')

    def invalider(self, motif, message):
        if not self.invalide:
            self.invalide = motif
            self.get_logger().error(message)
        self.logique.arret_securite(motif)
        self.publier(0.0, self.logique.dc)

    def cb_tours(self, msg):
        self.nb_tours = msg.data

    def cb_arret(self, msg):
        if msg.data:
            self.get_logger().warn('Arrêt demandé sur /gen6323/arret.')
            self.logique.arret_securite('arret demande')
            self.publier(0.0, self.logique.dc)

    def cb_surveillance(self):
        maintenant = self.horloge_reelle.now()
        if self.derniere_odom_reelle is None:
            # Amorçage (annexe C.3) : en mode synchrone, le simulateur attend une commande pour
            # avancer. Sans odométrie après 1 s, une commande NULLE fait avancer d'un pas.
            depuis = (maintenant - self.debut_reel).nanoseconds * 1e-9
            if depuis > 1.0 and (self.dernier_amorcage is None or
                                 (maintenant - self.dernier_amorcage).nanoseconds * 1e-9 > 1.0):
                self.dernier_amorcage = maintenant
                self.get_logger().info('Aucune odométrie reçue : commande nulle d’amorçage publiée.')
                self.publier(0.0, 0.0)
            return
        if self.logique.arret or self.logique.termine or self.fige or self.nb_etats < 2:
            return                                        # surveillance armée après le premier pas
        self.tic_surveillance += 1
        if self.tic_surveillance % 20 == 0:                # toutes les secondes (minuterie de 0,05 s)
            n = self.count_publishers('/drive')
            if n > 1:
                self.invalider('autre emetteur sur /drive',
                               f'{n - 1} autre(s) émetteur(s) détecté(s) sur /drive : essai invalide.')
                return
        age = (maintenant - self.derniere_odom_reelle).nanoseconds * 1e-9
        if age > self.delai_odom:
            self.get_logger().error(f'Odométrie périmée depuis {age:.2f} s : arrêt de sécurité.')
            self.logique.arret_securite('odometrie perimee')
            self.publier(0.0, self.logique.dc)

    # ------------------------------------------------------------------ outils
    def publier(self, vc, dc, stamp=None):
        m = AckermannDriveStamped()
        if stamp is not None:
            m.header.stamp = stamp
        m.header.frame_id = 'ego_racecar/base_link'
        m.drive.speed = float(vc)
        m.drive.steering_angle = float(dc)
        self.pub.publish(m)

    def resume(self, etat):
        M = {k: np.array(v) for k, v in self.mesures.items()}
        if len(M['t']) < 2:
            self.get_logger().warn(f'Essai terminé ({etat}) sans mesure exploitable'
                                   + (f' ; ESSAI INVALIDE ({self.invalide}).' if self.invalide else '.'))
            return
        dt = np.diff(M['t']); T = M['t'][-1] - M['t'][0]
        e_rms = math.sqrt(np.sum(M['e'][1:] ** 2 * dt) / T)
        J_d = np.sum(np.diff(M['dc']) ** 2 / dt) / T
        J_v = np.sum(np.diff(M['vc']) ** 2 / dt) / T
        tc = 1e3 * M['tc']
        verdict = 'ESSAI INVALIDE (' + self.invalide + ')' if self.invalide else 'ESSAI VALIDE'
        self.get_logger().info(
            f'{verdict} | FIN ({etat}) : T = {T:.2f} s, e_RMS = {100 * e_rms:.2f} cm, e_max = {100 * M["e"].max():.2f} cm, '
            f'J_delta = {J_d:.5f}, J_v = {J_v:.5f}, collision = {self.collision}, '
            f'calcul moyen {tc.mean():.3f} ms, 95e centile {np.percentile(tc, 95):.3f} ms, max {tc.max():.3f} ms')
        self.f_journal.flush()

    def fermer(self):
        self.publier(0.0, self.logique.dc)               # arrêt explicite à la fermeture
        self.f_journal.close()
        self.get_logger().info(f'Commande nulle publiée ; journal fermé : {self.chemin_journal}')


def main(args=None):
    # Gestion manuelle de Ctrl + C : on publie une commande nulle AVANT d'arrêter ROS.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    noeud = NoeudSuiviPP()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.fermer()
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
