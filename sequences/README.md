# sequences/ - zhardkodowane sekwencje jazda + ramie

Pliki `<nazwa>.json` zapisuje panel jazdy (`web_control.py`, :8000, sekcja "SEKWENCJA")
i odtwarza je w trybie "sequence" (`pinecone_bot/sequence.py`). Mozna je tez pisac recznie:

```json
{
  "name": "szyszka1",
  "steps": [
    {"type": "drive", "speed": 0.3, "steer": 0.0, "seconds": 2.0},
    {"type": "arm", "name": "grasp_cam"},
    {"type": "wait", "seconds": 0.5},
    {"type": "drive", "speed": -0.3, "steer": 0.0, "seconds": 2.0}
  ]
}
```

- `drive`: speed/steer to ulamki -1..1 jak WASD (+speed = przod, +steer = prawo jak D),
  bez limitu predkosci z suwaka; po `seconds` stop i 0.3 s przerwy.
- `arm`: nazwa ruchu z `motions/` (nagraj go z panelu ramienia, sekcja "NAGRYWANIE RUCHU");
  krok konczy sie, gdy panel ramienia (`tools/arm_web.py`, :8010) zglosi "gotowe".
- `wait`: pauza.

Pliki `_*.json` (szkic z przegladarki, test kroku) sa tymczasowe i ignorowane przez git.
STOP w panelu, zmiana trybu albo utrata heartbeatu przerywa sekwencje, zeruje jazde
i wysyla STOP do ramienia.
