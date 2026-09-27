# models/ - wytrenowane wagi

WAGI NIE SA NA MASTERZE (uplink hotspotu 50 KB/s, GitHub odrzuca commity bez obiektow LFS).
Leza na Pi w `~/models/act_so101_grasp2/<krok>/pretrained_model` i na galezi `frane/models-lfs` (LFS), gdy
zostanie wypchnieta. Opis checkpointow:

- `act_so101_grasp2/001000/pretrained_model/` - ACT po 1000 krokach (batch 8, AMP) na
  `datasets/so101_grasp2` (50 epizodow, kamera wrist), 2026-09-27 14:32, loss 2.28 przy kroku 800.
  Pierwszy uzywalny checkpoint do testu toru inferencji.
- `act_so101_grasp2/002000/pretrained_model/` - ACT po 2000 krokach, 14:39, loss 1.24 (l1 0.31).
- `act_so101_grasp2/003000/pretrained_model/` - ACT po 3000 krokach, 14:46, loss 0.79 (l1 0.27).
- `act_so101_grasp2/004000/pretrained_model/` - ACT po 4000 krokach, 15:24, loss 0.575. UZYWAC TEGO (koniec treningu).
  Wznowione z 3000 po twardym restarcie laptopa (`--config_path .../003000/pretrained_model/train_config.json --resume=true`).

UWAGA: obiekty LFS dla 2000/3000/4000 moga jeszcze dojezdzac (uplink hotspotu ~50 KB/s). Jesli `git lfs pull`
zglasza brak obiektu, wagi sa na Pi: `scp -r robot@172.20.10.4:~/models/act_so101_grasp2/004000 .`

`*.safetensors` sa w Git LFS: po klonie `git lfs install` i `git lfs pull`. Na Pi:

    tar -cf - -C models/act_so101_grasp2/001000 pretrained_model | ssh robot@172.20.10.4 \
      'mkdir -p ~/models/act_so101_grasp2/001000 && tar -xf - -C ~/models/act_so101_grasp2/001000'

Rollout: docs/SETUP.md, sekcja "Rollout polityki ACT na Pi".
