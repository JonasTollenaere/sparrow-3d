# sparrow-3d

`sparrow-3d` tackles 3D irregular strip packing problems,
where a set of 3D irregular items must be placed inside a rectangular container of fixed width and depth,
while minimising the height of the container.
It is based on and intended to closely match the strategy of [`sparrow`](https://github.com/JeroenGar/sparrow) algorithm as described in
["_Gardeyn et al. (2025) An open-source heuristic to reboot 2D nesting research_"](https://doi.org/10.5281/zenodo.17053914).

![Packing Animation](images/example.gif)

## General information

Our C++ implementation uses the [MeshCore library](https://github.com/JonasTollenaere/MeshCore) for loading files, rendering and providing the necessary geometric operations.
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