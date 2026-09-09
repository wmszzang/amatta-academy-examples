import argparse
from pathlib import Path
import matplotlib.pyplot as plt


def draw(path):
    fig, ax = plt.subplots(figsize=(5, 5))
    for x, y, name in [(0,0,'A'),(1,0,'B'),(-1,1,'C')]:
        ax.quiver(x,y,1,0,color='#d95f59',angles='xy',scale_units='xy',scale=1)
        ax.quiver(x,y,0,1,color='#16866b',angles='xy',scale_units='xy',scale=1)
        ax.text(x+.1,y+.1,name)
    ax.set(xlim=(-2,3),ylim=(-2,3)); ax.set_aspect('equal'); ax.grid(alpha=.3)
    fig.savefig(path, dpi=150); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--frames', action='store_true'); args = parser.parse_args()
    if args.frames:
        out = Path('frames'); out.mkdir(exist_ok=True)
        for i in range(12): draw(out / f'frame_{i:02d}.png')
    else: draw('frames.png')
