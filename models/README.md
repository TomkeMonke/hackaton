# models/ - wytrenowane wagi (Git LFS)

- `act_so101_grasp2/001000/pretrained_model/` - ACT po 1000 krokach (batch 8, AMP) na
  `datasets/so101_grasp2` (50 epizodow, kamera wrist), 2026-09-27 14:32, loss 2.28 przy kroku 800.
  Pierwszy uzywalny checkpoint do testu toru inferencji.
- `act_so101_grasp2/002000/pretrained_model/` - ACT po 2000 krokach, 14:39, loss 1.24 (l1 0.31).
- `act_so101_grasp2/003000/pretrained_model/` - ACT po 3000 krokach, 14:46, loss 0.79 (l1 0.27). UZYWAC TEGO.
  Trening przerwany na kroku 3528 (restart sesji Claude); to ostatni checkpoint z budzetu 1 h.

`*.safetensors` sa w Git LFS: po klonie `git lfs install` i `git lfs pull`. Na Pi:

    tar -cf - -C models/act_so101_grasp2/001000 pretrained_model | ssh robot@172.20.10.4 \
      'mkdir -p ~/models/act_so101_grasp2/001000 && tar -xf - -C ~/models/act_so101_grasp2/001000'

Rollout: docs/SETUP.md, sekcja "Rollout polityki ACT na Pi".
