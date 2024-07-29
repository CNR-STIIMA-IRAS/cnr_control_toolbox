# CNR Control Toolbox #

[![codecov][b]][2]
[![Codacy Badge][c]][3]
[![FOSSA Status][d]][4]

## Aim ##

state_space_systems is an Eigen implementation of a discrete state space linear system, including special case like: low- and high-pass first-order filters.

## Package Organization ##

## Classes Available ##


## NONLINEAR SYSTEMS ##

_DynamicSystem_

Description:
An abstract base class for dynamic systems. It defines the interface for any dynamic system, including methods for computing dynamic functions and output functions, as well as methods for handling the state of the system.

Key Members:

* name_: Name of the system.
* xdim_: Dimension of the state.
* udim_: Dimension of the input.
* ydim_: Dimension of the output.
* state_: State vector.
* Ptr and ConstPtr: Shared pointer types for the class.

Key Methods:

* xDim(): Returns the dimension of the state.
* uDim(): Returns the dimension of the input.
* yDim(): Returns the dimension of the output.
* getName(): Returns the name of the system.
* setName(const std::string&): Sets the name of the system.
* dynamicFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Pure virtual function to compute the dynamic function.
* outputFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Pure virtual function to compute the output function.
* regularizeState(const Eigen::VectorXd&): Pure virtual function to regularize the state vector.
* setState(const Eigen::VectorXd&): Sets the state vector.
* getState(): Returns the state vector.
* operator<<: Friend function to print the matrices of the system.
to_string(const DynamicSystem&): Friend function to convert the system to a string.

_ContinuousDynamicSystem_

Description:
A template for continuous dynamic systems. It inherits from DynamicSystem and defines the interface specific to continuous systems.


Key Methods:

* dynamicFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Pure virtual function to compute the dynamic function for continuous systems.
* outputFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Pure virtual function to compute the output function for continuous systems.

_DiscreteDynamicSystem_

Description:
A template for discrete dynamic systems. It inherits from DynamicSystem and defines the interface specific to discrete systems.

Key Methods:

* dynamicFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Pure virtual function to compute the dynamic function for discrete systems.
* outputFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Pure virtual function to compute the output function for discrete systems.

_FirstOrderContinuousSystem_

Description:
A class representing a first-order continuous dynamic system. It inherits from ContinuousDynamicSystem and provides concrete implementations for the dynamic and output functions.

Key Members:

* regressive_coef_: Regressive coefficient. dx/dt=regressive_coef_*(x-u)

Key Methods:

* FirstOrderContinuousSystem(const double&): Constructor that initializes the dimensions and the regressive coefficient.
* dynamicFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Computes the dynamic function.
* regularizeState(const Eigen::VectorXd&): Regularizes the state vector.
* outputFcn(const Eigen::VectorXd&, const Eigen::VectorXd&): Computes the output function.

This structure provides a comprehensive framework for modeling and simulating both continuous and discrete dynamic systems, with FirstOrderContinuousSystem being a specific example of a continuous system.


_Integrator_

An abstract base class for an integrator, providing a common interface for integrating continuous dynamic systems.

Methods

* integrate(const ContinuousDynamicSystem& system, const Eigen::VectorXd& state, const Eigen::VectorXd& input, const double& step_time) = 0
Virtual method to integrate a continuous dynamic system over a specified time step.
Parameters:
* system: Continuous dynamic system.
* state: Current state.
* input: System input.
* step_time: Time step for integration.
Returns: Integrated state as Eigen::VectorXd.

RungeKutta4
A concrete implementation of the Integrator class using the Runge-Kutta 4th order method.

Methods
integrate(const ContinuousDynamicSystem& system, const Eigen::VectorXd& state, const Eigen::VectorXd& input, const double& step_time) override
Implements the Runge-Kutta 4th order integration method for a continuous dynamic system.
Parameters:
system: Continuous dynamic system.
state: Current state.
input: System input.
step_time: Time step for integration.
Returns: Integrated state as Eigen::VectorXd.
DiscretizedDynamicSystem
A class for converting a continuous dynamic system into a discrete dynamic system using a specified integrator and time step.

Members
continuous_system_: Pointer to the continuous dynamic system (ContinuousDynamicSystem::Ptr).
integrator_: Pointer to the integrator (Integrator::Ptr).
step_time_: Time step for discretization (double).
Methods
DiscretizedDynamicSystem(const ContinuousDynamicSystem::Ptr& continuous_system, const Integrator::Ptr& integrator, const double& step_time)
Constructor for DiscretizedDynamicSystem.
Parameters:
continuous_system: Pointer to the continuous dynamic system.
integrator: Pointer to the integrator.
step_time: Time step for discretization.
Eigen::VectorXd dynamicFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
Computes the dynamic function for the discretized system.
Parameters:
x: State vector.
u: Input vector.
Returns: Result of the dynamic function as Eigen::VectorXd.
Eigen::VectorXd outputFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
Computes the output function for the discretized system.
Parameters:
x: State vector.
u: Input vector.
Returns: Result of the output function as Eigen::VectorXd.
Eigen::VectorXd regularizeState(const Eigen::VectorXd& x) const
Regularizes the state vector of the discretized system.
Parameters:
x: State vector.
Returns: Regularized state vector as Eigen::VectorXd.

## LINEAR SYSTEMS ##

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


## Contribution guidelines ##

### Contact ###

<mailto:<mailto:manuel.beschi@unibs.it>>
<mailto:<mailto:nicola.pedrocchi@stiima.cnr.it>>



[b]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_control_toolbox/branch/master/graph/badge.svg?token=D29UN0QD0X
[2]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_control_toolbox

[c]:https://api.codacy.com/project/badge/Grade/7f1834c02aa84b959ee9b7529deb48d6
[3]:https://app.codacy.com/gh/CNR-STIIMA-IRAS/cnr_control_toolbox?utm_source=github.com&utm_medium=referral&utm_content=CNR-STIIMA-IRAS/cnr_control_toolbox&utm_campaign=Badge_Grade_Dashboard

[d]:https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_control_toolbox.svg?type=shield
[4]:https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_control_toolbox?ref=badge_shield
