/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
  //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles = 101;

  // define normal distributions for sensor noise
  // std is a vector of standard deviations
  // This initializes normally distributed random distribution, ready to generate (gen)
  //random numbers

  normal_distribution<double> N_x_init(0, std[0]);
  normal_distribution<double> N_y_init(0, std[1]);
  normal_distribution<double> N_theta_init(0, std[2]);

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

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // define normal distributions for sensor noise
  normal_distribution<double> N_x(0, std_pos[0]);
  normal_distribution<double> N_y(0, std_pos[1]);
  normal_distribution<double> N_theta(0, std_pos[2]);

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
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//Nearest neighbour calculation

	double m_dist = 0;
	for (int i = 0; i<observations.size(); i++){

		double min_dist = 999999;
		int closest_landmark = -1;

		for (int j = 0; j < predicted.size(); j++) {
			m_dist = dist(observations[i].x,observations[i].y, predicted[j].x, predicted[j].y);

			if (m_dist < min_dist){
				min_dist = m_dist;
				closest_landmark = predicted[j].id;
			}
		}
		observations[i].id = closest_landmark;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	weights.clear();

	for (int i=0; i<particles.size();i++){
		// create a vector to hold the map landmark locations predicted to be within sensor range of the particle
		vector<LandmarkObs> observations_map;
		 // create and populate a copy of the list of observations transformed from vehicle coordinates to map coordinates

		for (int j=0; j<observations.size(); j++){
			LandmarkObs m_observation;

			m_observation.x = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
			m_observation.y = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;
			m_observation.id = -1;

			observations_map.push_back(m_observation);
		}


		vector<LandmarkObs> pred_meas;

		for (int j = 0; j <map_landmarks.landmark_list.size(); j++) {
			double m_dist_particle_obs;
			// only consider landmarks within sensor range of the particle ( the "dist" method considering a circular 
      		// region around the particle, rectangular region is computationally faster but not using rectangular for now)
			m_dist_particle_obs = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f );

			if (m_dist_particle_obs<sensor_range){
				LandmarkObs pred_landmark;
				pred_landmark.id = map_landmarks.landmark_list[j].id_i;
				pred_landmark.x = map_landmarks.landmark_list[j].x_f;
				pred_landmark.y = map_landmarks.landmark_list[j].y_f;

				pred_meas.push_back(pred_landmark);

			}
		}

		dataAssociation(pred_meas, observations_map);
		double prob = 1;
		double prob_j;
		double obs_w;

		for (int j = 0; j < pred_meas.size(); j++) {
			int id_min = -1;
			double min_dist = 99999;

			for (int k = 0; k < observations_map.size(); k++) {
				double m_dist = dist(pred_meas[j].x, pred_meas[j].y, observations_map[k].x, observations_map[k].y);

				if (m_dist< min_dist){
					min_dist = m_dist;
					id_min = k;
				}
			}

			if (id_min != -1){
				// calculate weight for  observation with multivariate Gaussian, using observed values as mean
				obs_w = ( 1/(2*M_PI*std_landmark[0]*std_landmark[1])) * exp( -( pow(pred_meas[j].x-observations_map[id_min].x,2)/(2*pow(std_landmark[0], 2)) + (pow(pred_meas[j].y-observations_map[id_min].y,2)/(2*pow(std_landmark[1], 2))) ) );
				prob=prob*obs_w;
			}
		}

		weights.push_back(prob);
		particles[i].weight = prob;

	}


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	//starting seed
	std::random_device seed;
	//generate randon numbers based off of seed
	std::mt19937 generator_wts(seed());


	// Creates a discrete distribution for weight , taking weight of each of the particles.
	// Uniformly sample
	std::discrete_distribution<int> distribution_wts(weights.begin(), weights.end());
	std::vector<Particle> resampled_particles;

	// Resample particles, sample particles more frequently if the weight is higher
	for(int i=0;i<num_particles;i++){
		Particle particles_i = particles[distribution_wts(generator_wts)];
		resampled_particles.push_back(particles_i);
	}
	particles = resampled_particles;



}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

//Functions provided to get Associations, get X, Y sensed values
string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
