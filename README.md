# An Exploration-Enhanced Search Algorithm for Robot Indoor Source Searching

[View Paper](https://ieeexplore.ieee.org/document/10665938)

## Source code

**Coming soon!**

Robot simulations were conducted using the [Gazebo](https://gazebosim.org/home) simulation environment on the [ROS](https://www.ros.org/) platform. The source search is performed by the [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) Burger robot, equipped with a single-line lidar, MOX concentration sensor, and anemometer. The ROS version is [Noetic](https://wiki.ros.org/noetic).

1. Download the source code and put it in `~/catkin_ws/src`.
2. Run `catkin_make` in `~/catkin_ws`.
3. Run the launch file.

## Dataset

The case names are formatted as HouseX1-X2-X3-X4-X5X6, where X1 represents the house number, X2 represents the airflow inlets, X3 represents the airflow outlets, X4 represents the airflow speed, X5 represents the space where the source is located (B for Bathroom or K for Kitchen), and X6 indicates whether the source is in the airflow (I for In or O for Out). For example, House16-13-2-04-KI signifies the scenario in House16 with airflow inlets 1 and 3, airflow outlet 2, airflow speed of 0.4 m/s, and a source located in the kitchen within the airflow.

- [CAD files.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetCAD) The 3D models were created by the [RobotAtVirtualHome](https://github.com/DavidFernandezChaves/RobotAtVirtualHome) project and converted into CAD models.
- [CFD data.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetCFD) CFD simulations were conducted using the [OpenFOAM](https://openfoam.org/) 11 software. The solver used is the simpleFOAM solver, and the model used is the $k-\epsilon$ model.
- [Source diffusion simulation data.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetGADEN) The diffusion of substances throughout the environment is simulated using [GADEN](https://github.com/MAPIRlab/gaden). You need to replace the absolute paths in the launch files with the paths of the corresponding files on your computer.

## Videos

- [House01-1-2-08-KI](https://youtu.be/IIh54fCSpMQ)
- [House01-1-2-08-BO](https://youtu.be/_lX9yu22AiU)
- [House07-1-3-08-KI](https://youtu.be/vkH--xWRIqw)
- [House14-1-3-04-KI](https://youtu.be/fFt_5qDy7Kg)
- [Scenario 1](https://youtu.be/WWvrSr8OD-Q)
- [Scenario 4](https://youtu.be/82lw_Nb8ELA)

## Cite this paper

```
@ARTICLE{10665938,
  author={Wang, Miao and Xin, Bin and Jing, Mengjie and Qu, Yun},
  journal={IEEE Transactions on Robotics}, 
  title={An Exploration-Enhanced Search Algorithm for Robot Indoor Source Searching}, 
  year={2024},
  volume={40},
  number={},
  pages={4160-4178},
  keywords={Robots;Robot sensing systems;Indoor environment;Sensors;Search problems;Dispersion;Computational modeling;Frontier-based exploration;indoor;rapidly-exploring random trees (RRTs);robot;source searching},
  doi={10.1109/TRO.2024.3454572}}
```
