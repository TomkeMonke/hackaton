#!/bin/bash
# Jedna proba chwytu na zywo, z laptopa (git-bash), kod idzie na Pi przez stdin (nic nie kopiuje na Pi).
# Uzycie: bash tools/live_grasp/run.sh N   -> log out/aN.err, zdjecie out/aN.jpg, stan w state.json
D="$(cd "$(dirname "$0")" && pwd)"
PI="${PI_HOST:-robot@robot.local}"
mkdir -p "$D/out"
{ echo "STATE = r'''$(cat "$D/state.json")'''"; cat "$D/percept.py" "$D/attempt.py"; }   | timeout 180 ssh -o BatchMode=yes "$PI" 'cd ~/hackaton && .venv/bin/python -' > "$D/out/a$1.b64" 2> "$D/out/a$1.err"
grep -E "^\[|Error|RESULT" "$D/out/a$1.err" | tail -30
NEW=$(grep "^STATE " "$D/out/a$1.err" | tail -1 | cut -c7-)
[ -n "$NEW" ] && echo "$NEW" > "$D/state.json"
python -c "import base64,sys;l=open(sys.argv[1]).read().strip().splitlines();open(sys.argv[2],'wb').write(base64.b64decode(l[-1])) if l else None" "$D/out/a$1.b64" "$D/out/a$1.jpg"
