# PMR
**PMR** (point model registration) aims to register a pointcloud to a CAD model (usually described in .stl format).

This work is published in the article *"Estimation of Point Cloud Object Pose Using Particle Swarm Optimization (ICMVA 2018)"* [https://www.researchgate.net/publication/326165587]()
<div align=center>
<img src="https://github.com/windz0629/pmr/blob/master/register_results.png" width="300"/>
</div>

## Pipeline

 `load stl model` --> `downsample stl triangles` --> `load pcd data` --> `filter cloud` --> `compute normal` --> `icf estimate`

 the `icf estimate` is accelerated by PSO algorithm:

<div align=center>
 <img src="https://github.com/windz0629/pmr/blob/master/icf_pso_flowchart.png" width="300" />
</div>

## Architecture
* **common**  -- defines data structure of stl model
* **conversion**  -- convert stl model to pointCloud (.pcd)
* **filter** -- randomly downsample triangles of the stl model
* **io**  -- load stl model from file
* **math**  -- a set of algorithms, including pso(partical swarm optimization), and transform algorithm.
* **registration** -- defines registration pipelines
  * icf -- iterative closet face method
  * mcn -- mean cluster normals estimation method (todo)
* **visualization** -- defines stl visualizer
* **examples** -- demos of icf_pso method

* **tools** -- defines some utility tools
  * cloud_transform -- transform a point cloud by 6D pose, including 3 euler angles(theta, phi, psi) and translation offsets(x,y,z)
  ```
  ./cloud_transform <stl_file_name> <0 0.5 0 10 0 0>
  ```
  * pcd_gen -- convert a stl model to a point cloud model, a leafsize should be given to define the point size
  ```
  ./pcd_gen <stl_file_name> <pcd_file_name> <leafsize>
  ```
  * pcd_viz -- visualize a pcd file
  ```
  ./pcv_viz <pcd_file_name>
  ```

  * stl_viz -- visualize a stl file
  ```
  ./stl_viz <stl_file_name>
  ```

## Compile & run
### Dependencies
* PCL 1.8
* VTK (v5.8 is suggested)

### Compile
#### examples compile
```
git clone
cd pmr
mkdir build && cd build
cmake ..
make
```
These commands will generate 2 demos: `icf_demo` and `icp_demo`.

 `icf_demo` uses directly face information from stl model and points from point cloud, to determine the pose of the point cloud. PSO algorithm is applied to accelerate the search speed.

However, `icp_demo` firstly converts the stl model to a point cloud model, and calculates the distances of points pairs to estimate the pose.

#### tools compile
```
cd tools
mkdir build && cd build
cmake ..
make
```
There is a `testdata` folder in `tools`, which is used for testing these tools.

### Run
#### icf_demo

`icf_demo` need 2 arguments: `stl_model_path` and `pcd_file_path`
```
cd build
./icf_demo ../models/cuboid.stl ../models/cuboid_30.pcd
```
The register precision and the computation time is a trade-off. If we increase the maximum iteration steps, we will ger better results, meanwhile, maybe we have to wait for the results over a cup of coffee.

After 40 steps of iteration, the cuboid point cloud was aligned with an error.

<div align=center>
<img src="https://github.com/windz0629/pmr/blob/master/icf_reg_39step.png" width="300"/>
</div>

After 200 steps, the cuboid point cloud was aligned well.

<div align=center>
<img src="https://github.com/windz0629/pmr/blob/master/icf_reg_200step.png" width="300"/>
</div>


If you need to adjust the maximum step of iteration, here is an instruction:

in `icf_pso.cpp`

```C++
void IterativeClosestFace_PSO::estimate(Eigen::Matrix4f & estimated_transf) {
  math::Swarm sw;
  //Config paras
  math::Pose minBound(-1, -1, -1, -M_PI/2, -M_PI/2, -M_PI/2);
  math::Pose maxBound(1, 1, 1, M_PI/2, M_PI/2, M_PI/2);
  math::Pose minVel(-0.04, -0.04, -0.04, -M_PI/60, -M_PI/60, -M_PI/60);
  math::Pose maxVel(0.04, 0.04, 0.04, M_PI/60, M_PI/60, M_PI/60);
  sw.setSearchSpaceBound(minBound,maxBound);
  sw.setSearchSpeedBound(minVel,maxVel);
  sw.setMaxIteration(200); // increase the max step will improve the result precision
  sw.setPopulationSize(20);

  ...
}
```
