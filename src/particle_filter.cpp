/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
//   std::cout<<"[init] Start"<< std::endl;
  num_particles = 100;  // Number of particles
  
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  
  for (int i = 0; i < num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);    
    p.weight = 1.0;
    this->particles.push_back(p);
  }
  
  this->is_initialized = true;
  
//   std::cout<<"[init] End"<< std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  
//   std::cout<<"[prediction] Start"<< std::endl;
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);
 
  for (int i = 0; i < num_particles; ++i) {
    
    if (fabs(yaw_rate) > 0.00001) {
      particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t)-sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta)-cos(particles[i].theta + yaw_rate*delta_t)); 
      particles[i].theta += yaw_rate * delta_t;
    }else{
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
//   std::cout<<"[prediction] End"<< std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  
//   std::cout<<"[dataAssociation] Start"<< std::endl;
//   std::cout<<"[dataAssociation] observations.size() = "<<observations.size()<< std::endl;
//   std::cout<<"[dataAssociation] observations.size() = "<<predicted.size()<< std::endl;
  
  for (int i = 0; i < observations.size(); ++i) {    
    double min_distance = -1;
    int min_id = -1;
  	for (int j = 0; j < predicted.size(); ++j) {
  		double distance = sqrt(pow(observations[i].x-predicted[j].x,2)+pow(observations[i].y-predicted[j].y,2));
    	if(min_id == -1 || distance<min_distance){
          min_distance = distance;
          min_id = predicted[j].id;
        }          
    }
    observations[i].id = min_id;
  }
//   std::cout<<"[dataAssociation] End"<< std::endl;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  
//   std::cout<<"[updateWeights] Start"<< std::endl;
      
  for (int i = 0; i < num_particles; ++i) {
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;
    
  	// HOMOGENOUS TRANSFORMATION: The observations are given in the VEHICLE'S coordinate system, 
  	// but particles are located according to the MAP'S coordinate system.
    vector<LandmarkObs> t_observations;
    for (int j = 0; j < observations.size(); ++j) {
      int id = observations[j].id;
      double xm = particle_x + cos(particle_theta)*observations[j].x - sin(particle_theta)*observations[j].y;
      double ym = particle_y + sin(particle_theta)*observations[j].x + cos(particle_theta)*observations[j].y;
      t_observations.push_back(LandmarkObs{id, xm, ym});
    }
  
    // for each particle -> predict position of each landmark
    vector<LandmarkObs> predicted_landmarks;    
    for(int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
      
      // Get distance between particle and landmarks
      double landmark_x = map_landmarks.landmark_list[j].x_f;
      double landmark_y = map_landmarks.landmark_list[j].y_f;      
      double dist = sqrt(pow(particle_x - landmark_x,2)+pow(particle_y - landmark_y,2));

      if(dist<=sensor_range){
        int landmark_id = map_landmarks.landmark_list[j].id_i;
        predicted_landmarks.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
      }      
    }  
    
    // For each particle, associate the predicted landmarks with the transformed observations
  	dataAssociation(predicted_landmarks, t_observations);
    
    
    particles[i].weight = 1.0;
    
    for(int j = 0; j < t_observations.size(); ++j) {
      
      // Find and create a landmark associated with the observation.
      int landmark_id = t_observations[j].id;
      double landmark_x;
      double landmark_y;
      for(int k = 0; k < predicted_landmarks.size(); ++k){
        if (predicted_landmarks[k].id == landmark_id) {
          landmark_x = predicted_landmarks[k].x;
          landmark_y = predicted_landmarks[k].y;
          break;
        }
      }
      
    double dist_x = t_observations[j].x - landmark_x;
    double dist_y = t_observations[j].y - landmark_y;
      
    double w = (1/(2*M_PI*std_landmark[0]*std_landmark[1])) * exp(-(pow(dist_x, 2)/(2*pow(std_landmark[0], 2)) + pow(dist_y, 2)/(2*pow(std_landmark[1], 2))));
      
	if (w == 0)
        particles[i].weight *= 0.00001;
	else
        particles[i].weight *= w;
    }
  }    
//   std::cout<<"[updateWeights] End"<< std::endl;
}

void ParticleFilter::resample() {
  
//   std::cout<<"[resample] Start"<< std::endl;
  
  double w_max = -1.0;
  for(int i = 0; i < num_particles; ++i){    
    if(particles[i].weight > w_max){
    	w_max = particles[i].weight;
    }
  }
  
  std::default_random_engine gen;
  std::uniform_real_distribution<double> dist_beta(0, 2.0 * w_max);
  std::uniform_int_distribution<int> dist_index(0, num_particles - 1);
  
  int index = dist_index(gen);
  double beta = 0.0;  
  vector<Particle> new_particles;
  
  for (int i = 0; i < num_particles; ++i) {
    beta += dist_beta(gen);
	while (beta > particles[index].weight){
		beta -= particles[index].weight;
		index = (index + 1) % num_particles;
    }
	new_particles.push_back(particles[index]);
  }
  
  this->particles = new_particles;
//   std::cout<<"[resample] End"<< std::endl;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}