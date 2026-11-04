# Plan de réglage des deux lois (étape 6)

Ce plan est fixé et commité AVANT toute simulation de réglage (section 3.1 de l'énoncé :
« Régler les deux lois avec un effort de recherche comparable. Expliquer les paramètres
explorés et le critère de choix, puis geler les réglages avant la campagne comparative »).

## 1. Paramètres explorés : même budget pour les deux lois (6 x 6 = 36 combinaisons)

| Loi | Paramètre 1 | Valeurs | Paramètre 2 | Valeurs |
|---|---|---|---|---|
| Pure Pursuit | l0 [m] | 0,3 ; 0,45 ; 0,6 ; 0,8 ; 1,0 ; 1,3 | tau [s] | 0 ; 0,1 ; 0,2 ; 0,3 ; 0,45 ; 0,6 |
| Stanley | ks [1/s] | 0,5 ; 1 ; 1,5 ; 2 ; 3 ; 5 | v0 [m/s] | 0,25 ; 0,5 ; 1 ; 1,5 ; 2 ; 3 |

Règle de sélection de la cible de Pure Pursuit : avance le long du chemin l(v) = l0 + tau*v.
Les grilles couvrent une plage de part et d'autre des réglages de démonstration
(l0 = 0,6 ; tau = 0,3 ; ks = 2 ; v0 = 1), avec un pas croissant.

## 2. Essais de réglage (identiques pour les deux lois) : 6 essais par combinaison

| # | Chemin | Vitesse cible | Départ | Facteur | Rôle |
|---|---|---|---|---|---|
| E1 | huit | 1 m/s | nominal | — | précision, basse vitesse |
| E2 | huit | 2 m/s | nominal | — | précision, vitesse de référence |
| E3 | huit | 3 m/s | nominal | — | précision, vitesse la plus élevée de la campagne |
| E4 | huit | 2 m/s | récupération | — | convergence depuis une erreur initiale |
| E5 | Spielberg (chemin préparé, gauss 0,3 m) | 2 m/s | nominal | — | circuit réel, épingles |
| E6 | huit | 2 m/s | nominal | retard 100 ms | tenue au retard (étape 5) |

Total : 36 combinaisons x 6 essais x 2 lois = 432 simulations.
Le huit couvre les virages des deux sens et le croisement ; Spielberg est le circuit des
phases 2 et 3. La droite et le cercle ne sont pas utilisés pour le réglage : ils servent
de contrôle dans la campagne. Réglages de vitesse identiques pour les deux lois.

## 3. Critère de choix (fixé avant les résultats)

**Admissibilité** (une combinaison est éliminée si une condition échoue) :
1. E1 à E5 se terminent normalement (tour complet), sans interruption ;
2. sur Spielberg (E5), e_max <= 0,30 m (marge sous la demi-largeur utile de la piste,
   1,10 - 0,155 = 0,945 m, en prévision de la dynamique de F1TENTH) ;
3. avec 100 ms de retard (E6), tour complet et J_delta(E6) <= 3 x J_delta(E2).

**Score** parmi les combinaisons admissibles, calculé séparément pour chaque loi sur E1 à E5 :

    S = 0,5 x moyenne_i [ e_RMS,i / médiane(e_RMS,i) ] + 0,5 x moyenne_i [ J_delta,i / médiane(J_delta,i) ]

où la médiane est prise sur les combinaisons admissibles de la même loi. Précision et
régularité du braquage ont le même poids. J_v n'entre pas dans le score : le profil de
vitesse est identique pour les deux lois.

**Choix** : la combinaison de score minimal. En cas d'égalité à 1 % près, on retient celle
dont le J_delta avec retard (E6) est le plus faible.

**Gel** : un seul jeu de paramètres par loi, enregistré dans `matlab/reglages_figes.m`,
utilisé sans modification pour toute la campagne (tableau 2) et pour le transfert ROS 2.
