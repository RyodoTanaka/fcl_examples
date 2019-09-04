# fcl_examples
[Flexible Collision Libraries](https://github.com/flexible-collision-library/fcl) examples.

|  |Sphere|Box|Cylinder|Triangle|Plane|Convex|Capsule|Ellipsoid|Cone|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|**Sphere**| :o: | :o: | :o: | :x: | :x: | :x: | :x: | :x: | :x: |
|**Box**| :o: | :o: | :o: | :x: | :x: | :x: | :x: | :x: | :x: |
|**Cylinder**| :o: | :o: | :o: | :x:|  :x: | :x: | :x: | :x: | :x: |
|**Triangle**| :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |
|**Plane**| :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |
|**Convex**| :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |
|**Capsule**| :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |
|**Ellipsoid**| :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |
|**Cone**| :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: | :x: |

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
