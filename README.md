# CNR Control Toolbox #

[![ROS INDUSTRIAL CI][a]][1]
[![codecov][b]][2]
[![Codacy Badge][c]][3]
[![FOSSA Status][d]][4]

## Aim ##

state_space_systems is an Eigen implementation of a discrete state space linear system, including special case like: low- and high-pass first-order filters.

## Package Organization ##

## Classes Available ##

## DiscreteStateSpace is generic discrete state space linear system ##

```c+++
x state
y output
u input

k actual step

y(k)=C*x(k)+D*u(k)
x(k+1)=A*x(k)+B*u(k)
```
### USAGE ###

#### Basic Usage ####
```c+++
#include <state_space_systems/discrete_state_space_systems.h>
```

```c++

  unsigned int order=10; // system order
  unsigned int nin=1;    // number of inputs
  unsigned int nout=1;   // number of outputs
  
  Eigen::MatrixXd A(order,order);
  Eigen::MatrixXd B(order,nin);
  Eigen::MatrixXd C(nout,order);
  Eigen::MatrixXd D(nout,nin);
  
  A.setRandom();
  B.setRandom();
  C.setRandom();
  D.setRandom();
  
 
  eigen_control_toolbox::DiscreteStateSpace ss(A,B,C,D);

  
  Eigen::VectorXd u(nin);   //input vector
  Eigen::VectorXd y(nout);  //output vector
  
  u.setRandom();
  y.setRandom();
  
  ss.setStateFromLastIO(u,y); // initialize initial state value for dumpless startup 
  ROS_INFO_STREAM("state:\n"<<ss.getState());
  ROS_INFO_STREAM("output:\n"<<ss.getOutput() << "\ndesired:\n"<<y);
  
  y=ss.update(u); // computing one step, updating state and output
```

#### Loading the Matrices from Param ####

```c+++
#include <state_space_systems/discrete_state_space_systems.h>
```

```c++

  unsigned int order=10; // system order
  unsigned int nin=1;    // number of inputs
  unsigned int nout=1;   // number of outputs
  
  
 
  eigen_control_toolbox::DiscreteStateSpace ss;
  if (!ss.importMatricesFromParam(nh,"ss")) // reading matrices from ss parameter (see below)
  {
    ROS_ERROR("error");
    return -1;
  }
  

  
  Eigen::VectorXd u(nin);   //input vector
  Eigen::VectorXd y(nout);  //output vector
  
  u.setRandom();
  y.setRandom();
  
  ss.setStateFromLastIO(u,y); // initialize initial state value for dumpless startup 
  ROS_INFO_STREAM("state:\n"<<ss.getState());
  ROS_INFO_STREAM("output:\n"<<ss.getOutput() << "\ndesired:\n"<<y);
  
  y=ss.update(u); // computing one step, updating state and output
```

Required parameters:
```yaml
ss:
  A:
  - [0, 1]
  - [0, 0]
  B:
  - [0]
  - [1]
  C:
  - [1, 0]
  D:
  - [0]  
```

## FirstOrderLowPass and FirstOrderHighPass are low-pass and high-pass first-order filters ##

```cpp
Low-pass filter: discretized version of 1/(tau*s+1)
High-pass filter: discretized version of tau*s/(tau*s+1)
```

### Usage of the FirstOrderLowPass ###

```c+++
#include <state_space_systems/eigen_common_filters.h>
```

```c++

  double natural_frequency = 500; // [rad/s]
  double sampling_period=0.001; // s
  eigen_control_toolbox::FirstOrderLowPassX lpf(natural_frequency,sampling_period); // the same for FirstOrderHighPass

  // initialization
  double u=0;
  double y=0;
  lpf.setStateFromLastIO(u,  y);

  // computing one step
  u=1;
  y=lpf.update(u);
   
```

### Load from params ###
you can load from param with the command:

```c+++
eigen_control_toolbox::FirstOrderLowPassX lpf;
lpf.importMatricesFromParam(nh,"/filter"); 
```

The ROS parameter can be equal to:
```yaml
filter:
  frequency: 5 # [Hz]
  sampling_period: 0.01 # [s]
```
or equal to:
```yaml
filter:
  natural_frequency: 2 # [rad/s]
  sampling_period: 0.01 # [s]
```
_Software License Agreement (BSD License)_    
_Copyright (c) 2010, National Research Council of Italy, Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing_    
_All rights reserved._

## Contribution guidelines ##

### Contact ###

<mailto:<mailto:manuel.beschi@stiima.cnr.it>>
<mailto:<mailto:nicola.pedrocchi@stiima.cnr.it>>

[a]:https://github.com/CNR-STIIMA-IRAS/cnr_control_toolbox/actions/workflows/industrial_ci_action.yml/badge.svg
[1]:https://github.com/CNR-STIIMA-IRAS/cnr_control_toolbox/actions/workflows/industrial_ci_action.yml

[b]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_control_toolbox/branch/master/graph/badge.svg?token=D29UN0QD0X
[2]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_control_toolbox

[c]:https://api.codacy.com/project/badge/Grade/7f1834c02aa84b959ee9b7529deb48d6
[3]:https://app.codacy.com/gh/CNR-STIIMA-IRAS/cnr_control_toolbox?utm_source=github.com&utm_medium=referral&utm_content=CNR-STIIMA-IRAS/cnr_control_toolbox&utm_campaign=Badge_Grade_Dashboard

[d]:https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_control_toolbox.svg?type=shield
[4]:https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_control_toolbox?ref=badge_shield