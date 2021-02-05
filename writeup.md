# PID_CONTROL_PROJECT

> AUTHOR: SungwookLE(joker1251@naver.com)  
> DATE: '21.2/5

## 01. INTRODUCTION

This Project is PID Path Control Project. This project purpose is similar to previous project. [behavior_clonning](https://github.com/SungwookLE/udacity_behavioral_cloning)

### 01-1. Requirements

Requirements are follows as below. [HERE](https://review.udacity.com/#!/rubrics/1972/view)

## 02. MAIN

### 02-1. CODE

In main.cpp, top of that function, I declare the two PID classes, one is pid_steer and another one is pid_speed.

```c++
  PID pid_steer;
  PID pid_speed;
  /**
   * TODO: Initialize the pid variable.
   */
  //double Kp_steer=0.13,Kd_steer=.21,Ki_steer=0.002;
  double Kp_steer=0.17,Kd_steer=.5,Ki_steer=0.0002;
  double Kp_speed = 0.07, Kd_speed = 0.01, Ki_speed = 0;
  pid_steer.Init(Kp_steer, Ki_steer, Kd_steer);
  pid_speed.Init(Kp_speed, Ki_speed, Kd_speed);
```

I declare the each controller gain `Kp, Kd, Ki`. And using `Init()` member, I save the number to each instance.

### 02-2. DESCRIBE the each effect of the P,I,D

P gain is the most effective parameter to reduce the error. However, if it is bigger than proper value, then the system can be occured overshoot. In this point, parameter D can be help to reduce the overshoot. D parameter works in response to derivative of error value. I parameter is useful to reduce the steady state error. But I found that I parameter can be increase the oscilation effect if I parameter is too big.

### 02-3. How to choose the parameter

In this project, iterational parameter tuning algorithm such as twiddle can't be adapted. Because twiddle algorithm needs the iteration process, but this project is a little inconvenient to make it iteratable in specific course.  
In this time, I tuned the controller parameter by my self(i.e. manual) with the knowledge of understanding of the effect of each P, I, D parameters.

First, I only tuned the P parameter. As I expeceted, cte was reduced but overshoot effect also was occured. Therefore, I tuned the D parameter. Then overshoot effect was reduced. Next, In high curvature road, the vehicle touch the side of the road for a seconds, So i tuned the I parameter. Finally, overall the performance of the path tracking was better.

PID class was very simplely implemented as below.

```c++

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  Kp=Kp_;
  Ki=Ki_;
  Kd=Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  i_error+=cte;
  d_error =cte- p_error;
  // std::cout << "p_error: " << cte << "/ d_error: " << d_error << std::endl;
  p_error = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double res = 0;
  res = i_error + d_error + p_error;
  return res;  // TODO: Add your total error calc here!
}

void PID::Calculate_Control_Value(double& input){

  input = -(p_error * Kp + i_error * Ki + d_error * Kd);

}
```

## 03. RESULTS

**Parameter:** `double Kp_steer=0.17,Kd_steer=.5,Ki_steer=0.0002;`  
One more thing, I tuned the speed controller as follow.  
**Parameter:** `double Kp_speed = 0.07, Kd_speed = 0.01, Ki_speed = 0;`

### 03-1. Vedio

No tire may leave the drivable portion of the track surface.  
![vedio_result](./1.gif)
