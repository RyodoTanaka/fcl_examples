# fcl_examples
[Flexible Collision Libraries](https://github.com/flexible-collision-library/fcl) examples.

|  |Sphere|Box|Cylinder|Others|
|:---:|:---:|:---:|:---:|:---:|
|**Sphere**| :o: | :o: | :x: | :x: |
|**Box**| :o: | :o: | :x: | :x: |
|**Cylinder**| :x: | :x: | :x: | :x: |
|**Others**| :x: | :x: | :x: | :x: |

## Requirements
[ROS melodic](https://ros.org)  
[fcl >= 0.6.0](https://github.com/flexible-collision-library/fcl)

## Installation
```bash
$ mkdir <catkin_ws>/src -p
$ cd <catkin_ws>/src
$ git clone https://github.com/RyodoTanaka/fcl_examples.git
$ cd <catkin_ws>
$ wstool init src
$ wstool merge -t src src/fcl_examples/dependencies.rosinstall
$ wstool up -t src
$ rosdep update
$ rosdep install -i -y -r --from-paths src
$ catkin b
```
