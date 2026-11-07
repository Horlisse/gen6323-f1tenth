"""Contrôle AVANT DÉPART d'un essai ROS 2 (docs/protocole_essais_ros2.md).

À lancer après le simulateur et AVANT le nœud de suivi. Vérifie : odométrie reçue, pose et cap
de départ attendus, vitesse nulle, absence de collision, aucun émetteur sur /drive, /cmd_vel
ou /initialpose. Code de sortie 0 si le départ est conforme, 1 sinon.

Exemple (Spielberg, départ nominal) :
  ros2 run gen6323_controle verifier_depart --ros-args -p stheta:=-2.8789845418
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool


class VerifierDepart(Node):
    def __init__(self):
        super().__init__('verifier_depart')
        souple = ParameterDescriptor(dynamic_typing=True)     # accepte 0 comme 0.0
        for nom, defaut in (('sx', 0.0), ('sy', 0.0), ('stheta', -2.8789845418),
                            ('tol_pos', 0.05), ('tol_cap', 0.02), ('attente_s', 5.0)):
            self.declare_parameter(nom, defaut, souple)
        self.odom = None
        self.collision = None
        self.create_subscription(Odometry, '/ego_racecar/odom', self.cb_odom, 10)
        self.create_subscription(Bool, '/ego_racecar/collision', self.cb_collision, 10)

    def cb_odom(self, msg):
        self.odom = msg

    def cb_collision(self, msg):
        self.collision = msg.data

    def p(self, nom):
        return float(self.get_parameter(nom).value)


def main(args=None):
    rclpy.init(args=args)
    n = VerifierDepart()
    t0 = time.time()
    while time.time() - t0 < n.p('attente_s') and (n.odom is None or n.collision is None):
        rclpy.spin_once(n, timeout_sec=0.1)

    resultats = []                                 # (conforme, libellé, détail)
    if n.odom is None:
        resultats.append((False, 'Odométrie reçue', 'aucune : le simulateur tourne-t-il ?'))
    else:
        pos, q = n.odom.pose.pose.position, n.odom.pose.pose.orientation
        cap = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        d_pos = math.hypot(pos.x - n.p('sx'), pos.y - n.p('sy'))
        d_cap = abs(math.atan2(math.sin(cap - n.p('stheta')), math.cos(cap - n.p('stheta'))))
        v = n.odom.twist.twist.linear.x
        resultats += [
            (True, 'Odométrie reçue', ''),
            (d_pos <= n.p('tol_pos'), 'Position de départ',
             f'({pos.x:.4f} ; {pos.y:.4f}), écart {100 * d_pos:.1f} cm'),
            (d_cap <= n.p('tol_cap'), 'Cap de départ', f'{cap:.6f} rad, écart {d_cap:.4f} rad'),
            (abs(v) < 0.05, 'Véhicule immobile', f'v = {v:.3f} m/s'),
        ]
    if n.collision is None:
        resultats.append((False, 'Pas de collision', 'état de collision non reçu'))
    else:
        resultats.append((not n.collision, 'Pas de collision',
                          'collision ACTIVE : relancer le simulateur' if n.collision else ''))
    for topic in ('/drive', '/cmd_vel', '/initialpose'):
        k = n.count_publishers(topic)
        resultats.append((k == 0, f'Aucun émetteur sur {topic}', f'{k} émetteur(s)' if k else ''))

    print('\nCONTRÔLE AVANT DÉPART')
    for ok, libelle, detail in resultats:
        print(f"  [{'OK' if ok else 'ÉCHEC'}] {libelle}" + (f' : {detail}' if detail else ''))
    conforme = all(ok for ok, _, _ in resultats)
    print('\n  => DÉPART CONFORME : lancer le nœud de suivi.\n' if conforme else
          '\n  => DÉPART NON CONFORME : corriger avant de lancer l’essai.\n')
    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if conforme else 1)


if __name__ == '__main__':
    main()
