"""lerobot-train na Windows bez uprawnien do symlinkow: pomija tworzenie checkpoints/last."""
import sys
import lerobot.scripts.lerobot_train as t

def _no_symlink(checkpoint_dir):
    print("[train_win] pomijam symlink checkpoints/last (Windows bez trybu deweloperskiego)", flush=True)

t.update_last_checkpoint = _no_symlink
sys.argv = ["lerobot-train"] + sys.argv[1:]
t.main()
