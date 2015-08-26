# SIL

Checkout the branch for the chapter you want to use! or keep reading.

## What is in this repo?

This repo contains code for compile sfuctions or mexfunctions that are run by the matlab simulator contained in the [Matlab_simulator](https://github.com/MAGICC-UAVbook/Matlab_simulator) repository. These repositories are pretty tightly coupled, meaning this repo will only work with the corisponding branch of that repo.

## What is the concept behind Software in the Loop (SIL)?

Matlab code is very different from c/c++ code.  Before trying to write code on the autopilot and debuging it there, it would be better to first run c code in the simulator where you know all the inputs and the outputs are working correctly and its not a problem with your code.  The idea is that the same (or nearly the same) code that is run by this sfunction could also be compiled for the pixhawk autopilot. Write the code here and then copy-paste it into the autopilot and run it there, and as long as the inputs and outputs are the same form and structure (see directly below) then it should work bug free (in theory). 

## I see a ####_base and ####_example class in each branch. What are they for?

This is inheratance and polymophism at its finest.  The base class if for communication.  It defines how things go in and out of the function.  The example class inherits a pure virtual funtion from the base clase that preforms the actual function of the application.  Since this code is based on a [college textbook](http://uavbook.byu.edu/doku.php), it is concivable that a future student will want to rewrite the code in the example class but not want to dig into how the inputs and params got there and where the outputs go. The controller_example class from chapter6 could eaily be changed to controller_STUDENTA and very little from the rest of code would need to be changed.

## How do I compile this stuff to run in matlab?

To my knowledge, this code has only successfully been compiled in Ubuntu 14.04 but there is no reason someone with the right skills couldn't get it to work in Windows as well.  Compiling requires the right compiler for matlab (gcc/g++ 4.7 for linux) and telling where the compiler can find the right matlab headers to make it a sfunction.  See the wiki associated with this repo for more infomation.

## You obviously didn't figure this all out by yourself. Where are your credits.

Lots of this sfunction stuff comes from matlab documentation.  Much of which can be found at MATLABROOT/toolbox/simulink/simdemos/simfeatures/src/ on your computer, including the sfun_counter_cpp.cpp file that shows how to uses pointers and classes within a compiled sfuncstion.  I also used [this tutorial](http://www.mathworks.com/matlabcentral/fileexchange/45522-mex-cmake) by Fang Liu to fingure out how to compile it with cmake (sorry its note better credited throughout the code).
