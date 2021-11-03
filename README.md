# CarND-pj6_Kidnapped-Vehicle
Udacity Self-Driving Car Engineer - Project6: Kidnapped Vehicle (Localization with particle filter)

This is the repository of the localization project of the Udacity Self-Driving Car Nanodegree.

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project a 2 dimensional [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) is implemented in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

This is the output of the project:

![output](./output/output.gif)


## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./particle_filter`

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. `./clean.sh`
2. `./build.sh`
3. `./run.sh`

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

***INPUT***: values provided by the simulator to the c++ program
<ul>
  <li>sense noisy position data from the simulator
    <ul>
      <li>["sense_x"]</li>
      <li>["sense_y"]</li>
      <li>["sense_theta"]</li>
    </ul>
  </li>
  <li>get the previous velocity and yaw rate to predict the particle's transitioned state
    <ul>
      <li>["previous_velocity"]</li>
      <li>["previous_yawrate"]</li>
    </ul>
  </li>
  <li>receive noisy observation data from the simulator, in a respective list of x/y values
    <ul>
      <li>["sense_observations_x"]</li>
      <li>["sense_observations_y"]</li>
    </ul>
  </li>
</ul>


***OUTPUT***: values provided by the c++ program to the simulator
<ul>
  <li>best particle values used for calculating the error evaluation
    <ul>
      <li>["best_particle_x"]</li>
      <li>["best_particle_y"]</li>
      <li>["best_particle_theta"]</li>
    </ul>
  </li>
  <li>Optional message data used for debugging particle's sensing and associations
      <ul>
      <li>for respective (x,y) sensed positions ID label</li>
      <ul>
        <li>["best_particle_associations"]</li>
      </ul>
      <li>for respective (x,y) sensed positions
        <ul>
          <li>["best_particle_sense_x"] <= list of sensed x positions</li>
          <li>["best_particle_sense_y"] <= list of sensed y positions</li>
          </ul>
        </li>
    </ul>
  </li>
</ul>

#### The Map
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria

The things the project is looking for are:

1. **Accuracy**: the particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: the particle filter should complete execution within the time of 100 seconds.
