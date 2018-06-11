# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

[![Project Video]](http://recordit.co/v1fbcmRwYP "project video")


#### Results
Using a particle filter (101 particles) based approach, I was able to pass the simulator test with the following errors (cumulative weighted errors)

X: 0.116
Y: 0.107
Yaw: 0.004

## Given a Map with landmark co-ordinates in format x,y,#:
```
92.064  -34.777 1
61.109  -47.132 2
17.42 -4.5993 3
-7.1285 -34.54  4
232.32  32.032  5
177.43  28.083  6
```
## Given a base main.cpp file with websocket connection to simulator [see more details at bottom for type of data passed to/from simulator]

## Given initial parameters
```
//Set up parameters here
  double delta_t = 0.1; // Time elapsed between measurements [sec]
  double sensor_range = 50; // Sensor range [m]

  double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigma_landmark [2] = {0.3, 0.3}; // Landma
```

## Some key functions implemented in the file particle_filter.cpp:

Initialize Particle filter
```
void ParticleFilter::init(double x, double y, double theta, double std[]) {...}
//Initialize Normal distributions for Particle x,y,theta values 
normal_distribution<double> N_x_init(0, std[0]);
normal_distribution<double> N_y_init(0, std[1]);
normal_distribution<double> N_theta_init(0, std[2]);

//Initialize particle positions, with normal noise
// init particles
  for (int i = 0; i < num_particles; i++) {
    //Defined in particle_filter.h
    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0;

    // add noise
    p.x += N_x_init(gen);
    p.y += N_y_init(gen);
    p.theta += N_theta_init(gen);
    //save particles to  vector of type Particle, defined in particle_filter.h as:
    //public:
		// Set of current particles
		//std::vector<Particle> particles;
    particles.push_back(p);
  }

  is_initialized = true;
}
```
Predict position
```
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {...}
//For each of the particles, predict net position (with added noise) using motion physics
for (int i = 0; i < num_particles; i++) {

    // calculate/predict new state, based on motion model. If yaw rate is negligible, dont add angle
    if (fabs(yaw_rate) < 0.00001) {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } 
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // add noise
    particles[i].x += N_x(gen);
    particles[i].y += N_y(gen);
    particles[i].theta += N_theta(gen);
  }

```
update weights of each particle 
```
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    std::vector<LandmarkObs> observations, Map map_landmarks) {...}
```
In main.cpp
```
// receive noisy observation data from the simulator
// sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]

vector<LandmarkObs> noisy_observations;
string sense_observations_x = j[1]["sense_observations_x"];
string sense_observations_y = j[1]["sense_observations_y"];
...
//NOTE: Each reading can contain sensor data from multiple landmarks
for(int i = 0; i < x_sense.size(); i++)
          {
            LandmarkObs obs;
            obs.x = x_sense[i];
        obs.y = y_sense[i];
        noisy_observations.push_back(obs);
          }
```
In particle_filter.cpp
```
for (int j = 0; j < predicted.size(); j++) {
      m_dist = dist(observations[i].x,observations[i].y, predicted[j].x, predicted[j].y);

      if (m_dist < min_dist){
        min_dist = m_dist;
        closest_landmark = predicted[j].id;
      }
    }
    observations[i].id = closest_landmark;
```


#### Submission
All you will submit is your completed version of `particle_filter.cpp`, which is located in the `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time.)

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

mkdir build
cd build
cmake ..
make
./particle_filter

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |
|   |   map_data.txt
|
|
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass!

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.
