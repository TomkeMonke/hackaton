# datasets/ - dane treningowe (LeRobotDataset v3)

- `so101_grasp2/` - 50 epizodow chwytu szyszki nagranych leaderem SO-101 na Pi, 2026-09-27,
  30 fps, 22451 klatek, jedna kamera `observation.images.wrist` (D415 na ramieniu, 640x480, AV1).
  Skopiowane z Pi (`~/datasets/so101_grasp2`) bez katalogow `tmp*`.

Wideo (`*.mp4`) jest w Git LFS: po klonie potrzebny `git lfs install` i `git lfs pull`,
inaczej zamiast filmow sa pliki-wskazniki. Trening na laptopie:

    lerobot-train --dataset.repo_id=local/so101_grasp2 --dataset.root=datasets/so101_grasp2 \
      --dataset.video_backend=pyav --policy.type=act --policy.device=cuda --policy.use_amp=true \
      --policy.push_to_hub=false --batch_size=16 --num_workers=4 --wandb.enable=false \
      --output_dir=outputs/train/act_so101_grasp2 --job_name=act_so101_grasp2 --steps=6000 --save_freq=500
