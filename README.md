# An Exploration-Enhanced Search Algorithm for Robot Indoor Source Searching

[View Paper](https://ieeexplore.ieee.org/document/10665938)

## Source code

Robot simulations were conducted using the [Gazebo](https://gazebosim.org/home) simulation environment on the [ROS](https://www.ros.org/) platform. The source search is performed by the [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) Burger robot, equipped with a single-line lidar, MOX concentration sensor, and anemometer. The ROS version is [Noetic](https://wiki.ros.org/noetic).

1. Download the source code and put it in `~/catkin_ws/src`.
2. Run `catkin_make` in `~/catkin_ws`.
3. Run the launch file.

## Dataset

- [CAD files.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetCAD) The 3D models were created by the [RobotAtVirtualHome](https://github.com/DavidFernandezChaves/RobotAtVirtualHome) project and converted into CAD models.
- [CFD data.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetCFD) CFD simulations were conducted using the [OpenFOAM](https://openfoam.org/) 11 software.
- [Dispersion data.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetGADEN) The diffusion of substances throughout the environment is simulated using [GADEN](https://github.com/MAPIRlab/gaden).
## Videos

- [House01-1-2-08-KI](https://youtu.be/zXVaouayGMA)
- [House01-1-2-08-BO](https://youtu.be/b7bXw0T3Oj8)
- [House07-1-3-08-KI](https://youtu.be/l2h5XbsGqTg)
- [House14-1-3-04-KI](https://youtu.be/87Sldf0MPOE)
- [Scenario 1](https://youtu.be/WWvrSr8OD-Q)
- [Scenario 4](https://youtu.be/82lw_Nb8ELA)

## Cite this paper

```
@article{wang2024exploration,
  title={An Exploration-Enhanced Search Algorithm for Robot Indoor Source Searching},
  author={Wang, Miao and Xin, Bin and Jing, Mengjie and Qu, Yun},
  journal={IEEE Transactions on Robotics},
  year={2024},
  publisher={IEEE}
}
```
