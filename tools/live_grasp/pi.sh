#!/bin/bash
# Uruchamia skrypt z tego katalogu na Pi przez ssh stdin (nic nie kopiuje na Pi). percept.py idzie zawsze pierwszy.
# Uzycie (git-bash na laptopie, z katalogu repo):
#   bash tools/live_grasp/pi.sh snap_only.py                                   # zdjecie + pozycje stawow -> out/snap_only.jpg
#   bash tools/live_grasp/pi.sh pick.py MODEL=data/grasp_model.json            # jedna proba z modelu
#   bash tools/live_grasp/pi.sh teach_clean.py --bg teach                      # w tle na Pi: /tmp/teach.jsonl, /tmp/teach.err
# Zmienne VAR=plik wstawiaja do skryptu:  VAR = r'''<zawartosc pliku>'''   (MODEL, STATE, CYCLE, ARGS).
# --bg NAZWA: nohup na Pi, stdout -> /tmp/NAZWA.out, stderr -> /tmp/NAZWA.err (przezyje zerwanie wifi).
D="$(cd "$(dirname "$0")" && pwd)"
PI="${PI_HOST:-robot@robot.local}"
SCRIPT="$1"; shift
VARS=(); BG=""
while [ $# -gt 0 ]; do
  case "$1" in
    --bg) BG="$2"; shift 2 ;;
    *=*) VARS+=("$1"); shift ;;
    *) echo "nieznany argument: $1"; exit 1 ;;
  esac
done
build() {
  for v in "${VARS[@]}"; do
    name="${v%%=*}"; file="${v#*=}"
    [ -f "$file" ] || file="$D/$file"
    echo "$name = r'''$(cat "$file")'''"
  done
  cat "$D/percept.py" "$D/$SCRIPT"
}
mkdir -p "$D/out"
base="$(basename "$SCRIPT" .py)"
if [ -n "$BG" ]; then
  build | ssh -o BatchMode=yes "$PI" "cd ~/hackaton && nohup .venv/bin/python - > /tmp/$BG.out 2> /tmp/$BG.err" &
  echo "w tle; log: ssh $PI tail -f /tmp/$BG.err"
  exit 0
fi
build | ssh -o BatchMode=yes "$PI" 'cd ~/hackaton && nohup .venv/bin/python -' > "$D/out/$base.b64" 2> "$D/out/$base.err"
grep -vE '^\s*$' "$D/out/$base.err" | grep -vE '^(STATE|POS|IMG) ' | tail -40
python - "$D/out/$base.b64" "$D/out/$base.jpg" <<'PY'
import base64, sys
lines = [l for l in open(sys.argv[1]).read().splitlines() if l and not l.startswith(("POS ", "IMG "))]
if lines:
    open(sys.argv[2], "wb").write(base64.b64decode(lines[-1])); print("zdjecie:", sys.argv[2])
PY
