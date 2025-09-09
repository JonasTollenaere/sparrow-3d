# Packing arbitrary three-dimensional objects

`sparrow-3d` is a nesting algorithm designed to tackle 3D irregular strip packing problems. 

In such problems, the goal is to place a set of 3D irregular items inside a rectangular container of fixed width and depth,
while minimising the height of the container.
It is based on and intended to closely match the strategy of [`sparrow`](https://github.com/JeroenGar/sparrow) algorithm as described in
["_Gardeyn et al. (2025) An open-source heuristic to reboot 2D nesting research_"](https://doi.org/10.5281/zenodo.17053914).

![Packing Animation](images/example.gif)

[//]: # (Animation of the `sparrow-3d` algorithm packing [engine parts]&#40;https://www.thingiverse.com/thing:644933&#41;. )

## General information

Our C++ implementation relies on the [MeshCore library](https://github.com/JonasTollenaere/MeshCore) for loading files, rendering and providing the necessary collision detection methods.
The project use uses CMake as its build system.
We recommend using vcpkg as a package manager to install the dependencies that are defined in the `vcpkg.json` file.
To set up a development environment, we refer to the [vcpkg documentation](https://vcpkg.io/en/getting-started).

## CMake targets

- `StripPacking_Fixed_Rotation`:
  Run `sparrow-3d` on a single instance, no rotation of the items considered.
* `StripPacking_Discrete_Rotation`:
  Run `sparrow-3d` on a single instance, a discrete number of rotations allowed.
- `Benchmark_Fixed_Rotation`:
  Benchmark `sparrow-3d` on a set of instances, without item rotation.
* `Benchmark_Discrete_Rotation`:
  Benchmark `sparrow-3d` on a set of instances, with discrete item rotations.
- `Render_Item_Poles`:
  Render the poles of inaccessibility of a given item.
* `Render_Solution_Poles`:
  Render the poles of inaccessibility for an entire solution.