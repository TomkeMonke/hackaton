# models/ - wytrenowane wagi (Git LFS)

- `act_so101_grasp2/001000/pretrained_model/` - ACT po 1000 krokach (batch 8, AMP) na
  `datasets/so101_grasp2` (50 epizodow, kamera wrist), 2026-09-27 14:32, loss 2.28 przy kroku 800.
  Pierwszy uzywalny checkpoint do testu toru inferencji; kolejne (2000+) dochodza w miare treningu.
- `act_so101_grasp2/007000/pretrained_model/` - ten sam trening dokonczony do 7000 krokow (wznawiany z 3000 i 4000),
  2026-09-27 16:05, loss 0.301 (l1 0.198). TEN checkpoint do rolloutu. Posrednie 2000-6000 tylko na laptopie frane
  (`C:/Users/frane/outputs/act_so101_grasp2_run2/checkpoints/`) - nie w LFS, limit 1 GB.

`*.safetensors` sa w Git LFS: po klonie `git lfs install` i `git lfs pull`. Na Pi:

    tar -cf - -C models/act_so101_grasp2/007000 pretrained_model | ssh robot@172.20.10.4 \
      'mkdir -p ~/models/act_so101_grasp2/007000 && tar -xf - -C ~/models/act_so101_grasp2/007000'

Rollout: docs/SETUP.md, sekcja "Rollout polityki ACT na Pi".
