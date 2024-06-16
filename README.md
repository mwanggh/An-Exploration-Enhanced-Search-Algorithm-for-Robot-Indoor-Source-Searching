# An Exploration-Enhanced Search Algorithm for Robot Indoor Source Searching

## Source code

The source code will be uploaded after the paper is accepted.

Robot simulations were conducted using the [Gazebo](https://gazebosim.org/home) simulation environment on the [ROS](https://www.ros.org/) platform. The source search is performed by the [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) Burger robot, equipped with a single-line lidar, MOX concentration sensor, and anemometer.

## Dataset

The case names are formatted as House$X_1$-$X_2$-$X_3$-$X_4$-$X_5 X_6$, where $ X_1 $ represents the house number, $ X_2 $ represents the airflow inlets, $ X_3 $ represents the airflow outlets, $ X_4 $ represents the airflow speed, $ X_5 $ represents the space where the source is located (B for Bathroom or K for Kitchen), and $ X_6 $ indicates whether the source is in the airflow (I for In or O for Out). For example, House16-13-2-04-KI signifies the scenario in House16 with airflow inlets 1 and 3, airflow outlet 2, airflow speed of 0.4 m/s, and a source located in the kitchen within the airflow.

- [CAD files.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetCAD) The 3D models were created by the [RobotAtVirtualHome](https://github.com/DavidFernandezChaves/RobotAtVirtualHome) project and converted into CAD models.
- [CFD data.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetCFD) CFD simulations were conducted using the [OpenFOAM](https://openfoam.org/) 11 software. The solver used is the simpleFOAM solver, and the model used is the $k-\epsilon$ model.
- [Source diffusion simulation data.](https://huggingface.co/datasets/WangHaaa/SourceSearchingDatasetGADEN) The diffusion of substances throughout the environment is simulated using [GADEN](https://github.com/MAPIRlab/gaden). You need to replace the absolute path in the launch file with the path of the corresponding file on your computer.

## Videos

- [1 House01-1-2-08-KI](https://youtu.be/3SqdmiUk0OE)
- [2 House01-1-2-08-BO](https://youtu.be/NEAkdBrY6AU)
- [3 House14-1-3-04-KI](https://youtu.be/8pUgce2rRew)
- [4 Scenario 1](https://youtu.be/WWvrSr8OD-Q)
- [5 Scenario 4](https://youtu.be/82lw_Nb8ELA)

## Cite this paper

Paper information will be updated after the paper is accepted.
