# Protocole des essais ROS 2 (phase 2)

Ce protocole garantit qu'un essai mesuré est **valide** : départ à la pose attendue, aucune
collision héritée d'un essai précédent, un seul émetteur de commandes (annexe C.2), aucune
intervention extérieure pendant l'essai. Il découle d'incidents réels, consignés dans le journal
de bord (véhicule non réinitialisé, collision héritée, téléportation par l'outil de pose de Foxglove).

## 1. Avant chaque essai

1. **Relancer le simulateur**, même s'il tourne déjà : c'est la seule façon de revenir à la pose
   de départ et d'effacer une collision mémorisée.
   ```bash
   source ~/sim_env.sh
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py config:=<configuration>.yaml open_foxglove:=false
   ```
2. **Fermer Foxglove** pour les essais mesurés. Il consomme du temps processeur (il fausse le coût
   de calcul) et ses outils de publication peuvent déplacer le véhicule (`/initialpose`).
   Foxglove reste réservé aux démonstrations et aux vidéos, sans utiliser ses outils de pose.
3. **Contrôle avant départ** (le nœud de suivi n'est pas encore lancé) :
   ```bash
   ros2 run gen6323_controle verifier_depart --ros-args -p sx:=<sx> -p sy:=<sy> -p stheta:=<stheta>
   ```
   Ne lancer l'essai que si le contrôle affiche **DÉPART CONFORME**.

   | Essai | sx | sy | stheta |
   |---|---|---|---|
   | Spielberg nominal | 0.0 | 0.0 | -2.8789845418 |
   | Spielberg récupération | 0.1298000639 | -0.4828580986 | -2.7044516166 |
   | Silverstone nominal | 0.0 | 0.0 | 0.9443958881 |

## 2. Pendant l'essai

- Ne lancer **aucune autre commande** sur `/drive`, `/cmd_vel` ou `/initialpose`.
- Le nœud surveille lui-même la validité : une collision, une réinitialisation reçue sur
  `/initialpose` ou un autre émetteur sur `/drive` déclenchent l'arrêt de sécurité et déclarent
  l'essai **INVALIDE**, motif à l'appui.

## 3. Après l'essai

1. Relever la ligne `ESSAI VALIDE | FIN (...)` ou `ESSAI INVALIDE (motif) | FIN (...)`.
2. Attendre `Véhicule arrêté ... fin des publications`, puis **Ctrl + C** (commande nulle publiée).
3. Archiver le journal, **valide ou non** (section 4.1 de l'énoncé) :
   ```bash
   cp ~/gen6323_resultats/ros2_*.csv /mnt/c/Users/horli/gen6323-f1tenth/resultats/
   ```
   puis commiter depuis PowerShell. Un essai invalide est conservé et signalé comme tel dans le rapport.
