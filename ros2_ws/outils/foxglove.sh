#!/usr/bin/env bash
# Ouvre Foxglove (navigateur Windows) connecté à la simulation F1TENTH
# et prépare la mise en page fournie par la passerelle.
# Usage : ~/foxglove.sh            (le simulateur doit déjà tourner)
set -uo pipefail

PORT=8765
ORGANISATION="uqo-2"
MISE_EN_PAGE=~/sim_ws/src/f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json
DESTINATION=/mnt/c/Users/horli/Downloads/gym_bridge_foxglove.json

if [ ! -f "$MISE_EN_PAGE" ]; then
    echo "ERREUR : mise en page introuvable : $MISE_EN_PAGE"; exit 1
fi
if ! (exec 3<>/dev/tcp/127.0.0.1/$PORT) 2>/dev/null; then
    echo "ERREUR : rien n'écoute sur le port $PORT. Lance d'abord le simulateur."; exit 1
fi
echo "OK : la passerelle Foxglove écoute sur ws://localhost:$PORT"

cp "$MISE_EN_PAGE" "$DESTINATION" || { echo "ERREUR : copie impossible vers $DESTINATION"; exit 1; }
echo "OK : mise en page copiée dans C:\\Users\\horli\\Downloads\\gym_bridge_foxglove.json"

URL="https://app.foxglove.dev/${ORGANISATION}/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A${PORT}"
if command -v wslview >/dev/null 2>&1 && wslview "$URL"; then
    echo "OK : Foxglove ouvert (wslview)"
elif powershell.exe -NoProfile -Command "Start-Process '$URL'" >/dev/null 2>&1; then
    echo "OK : Foxglove ouvert (PowerShell)"
else
    echo "ATTENTION : ouverture automatique impossible. Ouvre cette adresse dans ton navigateur :"
    echo "  $URL"
fi

cat << 'TEXTE'

PREMIÈRE FOIS SEULEMENT, dans Foxglove :
  1. En haut à droite, ouvre le menu « Default for ws://localhost:8765 ».
  2. Choisis « Import from file… ».
  3. Sélectionne : Téléchargements\gym_bridge_foxglove.json
  -> La carte, le véhicule et le LiDAR s'affichent.

LES FOIS SUIVANTES : Foxglove recharge automatiquement cette mise en page
pour ws://localhost:8765. Il suffit de relancer ce script.
TEXTE
