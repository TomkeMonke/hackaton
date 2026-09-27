# models/ - wytrenowane wagi

Wagi sa w Git LFS (`*.safetensors`): po klonie `git lfs install && git lfs pull`. Kopie leza tez na Pi
w `~/models/act_so101_grasp2/<krok>/pretrained_model` (1000 i 2000 kompletne, 3000/4000 przerwane w polowie).
Opis checkpointow (jeden trening ACT, batch 8, AMP, na `datasets/so101_grasp2`, 50 epizodow, kamera wrist):

- `act_so101_grasp2/001000/pretrained_model/` - po 1000 krokach, 2026-09-27 14:32, loss 2.28 przy kroku 800.
  Pierwszy uzywalny checkpoint do testu toru inferencji.
- `act_so101_grasp2/002000/pretrained_model/` - po 2000 krokach, 14:39, loss 1.24 (l1 0.31).
- `act_so101_grasp2/003000/pretrained_model/` - po 3000 krokach, 14:46, loss 0.79 (l1 0.27).
- `act_so101_grasp2/004000/pretrained_model/` - po 4000 krokach, 15:24, loss 0.575 (l1 0.255).
  Wznowione z 3000 (`--config_path .../003000/pretrained_model/train_config.json --resume=true`).
- `act_so101_grasp2/007000/pretrained_model/` - trening dokonczony do 7000 krokow (wznowiony z 4000), 16:05,
  loss 0.301 (l1 0.198). UZYWAC TEGO do rolloutu. Posrednie 5000/6000 tylko na laptopie frane
  (`C:/Users/frane/outputs/act_so101_grasp2_run2/checkpoints/`), nie w LFS.

UWAGA: obiekty LFS dla 2000/3000/4000 moga jeszcze dojezdzac (uplink hotspotu ~50 KB/s). Jesli `git lfs pull`
zglasza brak obiektu, wagi sa na Pi: `scp -r robot@172.20.10.4:~/models/act_so101_grasp2/004000 .`
Obiekt 7000 jest wgrany w calosci (207 MB, 4 MB/s).

Na Pi:

    tar -cf - -C models/act_so101_grasp2/007000 pretrained_model | ssh robot@172.20.10.4       'mkdir -p ~/models/act_so101_grasp2/007000 && tar -xf - -C ~/models/act_so101_grasp2/007000'

Rollout: docs/SETUP.md, sekcja "Rollout polityki ACT na Pi".
