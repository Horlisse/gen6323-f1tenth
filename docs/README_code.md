# GEN6323 – Projet intégrateur F1TENTH

## Arborescence (état : étapes 1 et 2)
- `matlab/vehicule/` : paramètres, modèle cinématique (arc exact / Euler), saturations
- `matlab/chemins/`  : références sources (annexe A.1), poses initiales (A.3),
  projection avec progression, point à une abscisse
- `matlab/tests/`    : `verif_modele.m`, `test_chemins.m`
- `python/gen6323_core/` : jumeau Python du cœur (réutilisé par le nœud ROS 2)
- `python/tests/`    : mêmes tests en Python

## Lancement MATLAB
```matlab
addpath(genpath('matlab'));
verif_modele      % vérification du modèle (1)
test_chemins      % S0, poses, croisement du huit
```
## Lancement Python
```bash
cd python && python3 tests/test_vehicle.py && python3 tests/test_path.py
```

## Sorties attendues (obtenues avec le jumeau Python)
| Test | Valeur |
|---|---|
| Arc exact, droite / cercles, 10 s à 2 m/s | erreur ≤ 5e-13 m |
| Euler, cercles | 2.000e-02 m (= v·Δt) |
| R théorique δ = 0,2 rad | 1.628928 m |
| S0 droite / cercle / huit | 20.000000 / 31.415927 / 48.777786 m |
| Récupération huit | (-0.35355, 0.35355, 0.95993) |
| Huit, progression | min ds > 0, écart signé = +0.3000 |
| Huit, recherche globale | 5 sauts de branche |

## Choix de conception documentés
1. Discrétisation : intégration exacte à commande bloquée (ZOH) sur Δt = 0,01 s.
2. Saturation : amplitude puis variation, à partir de (0, 0) au démarrage.
3. Projection : fenêtre [s − 1 m, s + 3 m] autour de la projection précédente,
   initialisée à s = 0 (pas de recherche globale : le départ du huit est au croisement).
