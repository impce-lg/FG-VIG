# FG-VIG



This repository focuses on the simulated Monocular Vision/INS/GNSS fusion based on Factor Graph optimization.
It includes experimental and simulated versions in /Examples. 
Now only the simulated data is available.
The difference of the experimental and simulated data is mainly on the vision front-end, the format of INS and GNSS is same to the real tests.

The data is produced by a self-developed software, 
you can contact Dr. Ronghe Jin (huanhexiao@whu.edu.cn or 773792173@qq.com) for the simulation code if you have a need.

The experimental data will be available as soon as it get ready.

## Functions

- GNSS/INS integration

- Monocular Vision/INS/GNSS fusion



## Prerequisites

- gtsam (https://github.com/borglab/gtsam) 

- Eigen

- OpenCV 3.4



## Usage

```shell

git clone https://github.com/huanhexiao/FG-VIG

mkdir build && cd build 

cmake .. && make -j3

```

## License
The source code is released under GPLv3 license.

Feel free to propose any issue you get confused, or you can to email me directly.
