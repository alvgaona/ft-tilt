"""Generates the world's ground textures (tileable grass and asphalt) and the quads that repeat
them: 400 x 400 m of grass every 5 m, and the 20 x 17 m parking lot every 2 m (an SDF <plane> or
<box> would stretch one texture over the whole surface).

    pixi run python ft_tilt_bringup/tools/make_ground.py
"""
import os

import numpy as np
from PIL import Image

OUT = os.path.join(os.path.dirname(__file__), "..", "models", "ground")
PX = 1024  # texture pixels


def noise(rng, sigma_px):
    """Periodic (tileable) smooth noise in [-1, 1]: white noise low-passed in Fourier space."""
    f = np.fft.fftfreq(PX)
    k2 = f[:, None] ** 2 + f[None, :] ** 2
    n = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal((PX, PX))) * np.exp(-2 * (np.pi * sigma_px) ** 2 * k2)))
    return n / np.abs(n).max()


def save(img, name):
    Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(
        os.path.join(OUT, "materials", name), quality=85)


def quad(name, size_x, size_y, tile):
    """Quad centred on the origin in the xy plane, facing up, texture repeated every `tile` m."""
    hx, hy, u, v = size_x / 2, size_y / 2, size_x / tile, size_y / tile
    with open(os.path.join(OUT, "meshes", name), "w") as f:
        f.write(f"v {-hx} {-hy} 0\nv {hx} {-hy} 0\nv {hx} {hy} 0\nv {-hx} {hy} 0\n")
        f.write(f"vt 0 0\nvt {u} 0\nvt {u} {v}\nvt 0 {v}\nvn 0 0 1\n")
        f.write("f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")  # counter-clockwise from above: faces up


def main():
    os.makedirs(os.path.join(OUT, "materials"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)

    rng = np.random.default_rng(7)
    patches, clumps, blades = noise(rng, 60), noise(rng, 8), noise(rng, 0.7)
    base = np.array([62, 102, 38], float)    # grass green
    dry = np.array([118, 122, 58], float)     # yellowish patches
    t = np.clip(0.5 + 0.6 * patches, 0, 1)[..., None]
    img = base * (1 - 0.35 * t) + dry * 0.35 * t
    img *= (1 + 0.12 * clumps + 0.18 * blades)[..., None]
    save(img, "grass.jpg")
    quad("ground.obj", 400.0, 400.0, 5.0)  # wide enough to hide the edge

    rng = np.random.default_rng(11)
    stains, grain = noise(rng, 40), noise(rng, 0.6)
    gray = 92 + 9 * stains + 16 * grain    # weathered asphalt, not fresh black
    stones = rng.random((PX, PX)) < 0.012       # light aggregate showing through
    gray[stones] += rng.uniform(25, 55, stones.sum())
    save(np.repeat(gray[..., None], 3, axis=2) * np.array([1.0, 1.0, 1.03]), "asphalt.jpg")
    quad("asphalt.obj", 20.0, 17.0, 2.0)


if __name__ == "__main__":
    main()
