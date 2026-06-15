"""
CyberArena — Cosmic Vortex
===========================

A high-level Payton example that combines geometry, particles, multi-light
scenes, collision detection, clock-driven animation, GUI controls, and
interactive click-to-explode mechanics into one visually dense experience.

Usage
-----
    cd examples/high-level/cyberarena
    python main.py

Hit SPACE to start the animation.  Press H for help.
"""

from arena import CyberArena

if __name__ == "__main__":
    arena = CyberArena(width=1600, height=900)
    arena.run(start_clocks=True)
