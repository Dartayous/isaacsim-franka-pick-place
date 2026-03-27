# Isaac Sim Franka Pick-and-Place

> A physics-stable Franka pick-and-place prototype built in NVIDIA Isaac Sim using a Z-up runtime stage, a referenced Y-up OpenUSD environment, and smooth joint-space interpolation.

## Demo

![Franka Pick and Place](./media/franka_pick_place_demo.gif)

**Full video:** [franka_pick_place_demo.mp4](./media/franka_pick_place_demo.mp4)

## Overview

This project implements a scripted Franka pick-and-place sequence in Isaac Sim using a referenced OpenUSD tabletop scene.

Key focus areas:
- OpenUSD scene composition
- Y-up to Z-up runtime reconciliation
- articulated robot placement
- grasp sequencing
- lift stability
- smooth, non-yanking joint-space motion

## Portfolio Description

Implemented a scripted Franka pick prototype in Isaac Sim using a Z-up runtime stage and a referenced Y-up OpenUSD environment, debugging articulation placement, frame conventions, grasp sequencing, and lift stability through interpolated joint-space motion.

Goal:
Build a small robotics simulation in Isaac Sim using a built-in robot, a simple environment, a basic scripted task, and synthetic data capture.

Initial target:
- Robot: Franka
- Environment: simple tabletop scene
- Task: basic reach / pick-place style motion
- Output: RGB synthetic data

Project folders:
- scripts/  -> Python scripts
- usd/      -> saved USD stages
- output/   -> captured images / video
- docs/     -> notes and debugging logs
