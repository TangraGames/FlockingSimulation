# Flocking

A simulation of flocking behavior using [raylib](https://www.raylib.com/).

## Overview

This project implements a Boids flocking simulation where autonomous agents follow simple local rules to create emergent, coordinated movement patterns — inspired by schools of fish, flocks of birds, or swarms of insects.

Based on the paper by Craig Reynolds: *Boids, Background and Update*
https://www.red3d.com/cwr/boids/

YouTube video of an example in Processing (Java) by Programming Chaos:
https://youtu.be/L5l_G5A3T1k?si=pzPyfOfOHGDN9UTN

## Features

- Autonomous boids with local neighbor awareness
- Three Reynolds flocking behaviors: Separation, Alignment, Cohesion
- Interactive mouse target that attracts and repels boids
- Color feedback: boids turn **blue** when attracted to target, **red** when repelling
- Toroidal (wrapping) world — boids exiting one edge re-enter from the opposite side
- Neighbor distance calculations account for screen wrapping
- Real-time visualization using raylib

## Controls

| Input | Action |
|---|---|
| **Left Click** | Toggle mouse target on/off |
| **Right Click** | Toggle target radius display (only while target is active) |
| **ESC** | Exit the simulation |

## Implementation

Each frame, four steering forces are computed per boid and summed into acceleration:

- `ComputeSeparation` — steer away from nearby boids, weighted by inverse distance so closer boids push harder
- `ComputeAlignment` — steer toward the average heading of nearby boids
- `ComputeCohesion` — steer toward the average position of nearby boids
- `ComputeTargetSteering` — steer toward or away from the mouse cursor depending on distance

All neighbor detection uses wrapped distance to correctly handle boids near screen edges.

## Configurable Parameters

All tuning values are defined as named constants at the top of `src/main.c`:

| Constant | Default | Description |
|---|---|---|
| `MAX_BOIDS` | `100` | Number of boids |
| `FPS` | `60` | Target frame rate |
| `BOID_SIZE` | `16.0` px | Size of the boid sprite |
| `SEPARATION_RADIUS` | `2 × BOID_SIZE` | Neighbor radius for separation |
| `ALIGNMENT_RADIUS` | `3 × BOID_SIZE` | Neighbor radius for alignment |
| `COHESION_RADIUS` | `4 × BOID_SIZE` | Neighbor radius for cohesion |
| `TARGET_AVOIDANCE_RADIUS` | `5 × BOID_SIZE` | Distance at which boids flee the target |
| `TARGET_DETECTION_RADIUS` | `12 × BOID_SIZE` | Distance at which boids are attracted to the target |
| `SEPARATION_K` | `2.5` | Separation steering weight |
| `ALIGNMENT_K` | `1.0` | Alignment steering weight |
| `COHESION_K` | `1.0` | Cohesion steering weight |
| `TARGET_ATTRACT_K` | `1.25` | Target attraction steering weight |
| `TARGET_REPEL_K` | `5.25` | Target repulsion steering weight |
| `MAX_BOID_SPEED` | `2.5 × FPS` px/s | Maximum boid speed |
| `MIN_BOID_SPEED` | `1.0 × FPS` px/s | Minimum boid speed |
| `MAX_FORCE` | `0.04 × FPS` | Maximum steering force per frame |

## Dependencies

- [raylib](https://github.com/raysan5/raylib) - Graphics and input library
- [raymath](https://github.com/raysan5/raylib/blob/master/src/raymath.h) - Vector math (included with raylib)
