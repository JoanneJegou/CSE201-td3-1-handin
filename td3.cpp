#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {

    double* A = new double[new_size] ;
    for (int i=0;i<length;i++) {
        A[i] = array[i] ;
    }
    for (int i=length;i<new_size;i++){
        A[i] = 0 ;
    }
    delete[] array ;

  return A;
}

double* shrink_array(double* array, int length, int new_size) {

  double* B = new double[new_size] ;
  for (int i=0;i<new_size;i++){
        B[i] = array[i];
  }
  delete[] array ;
  return B;

  return array;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {

  if (current_size==max_size){
        array = extend_array(array,current_size,max_size+5);
        max_size = max_size+5;
        *(array+current_size) = element  ;
        current_size = current_size+1 ;
        return array;
  }
  else {
        array[current_size] = element ;
        current_size = current_size+1;
        return array;
  }
}


double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {

  current_size = current_size-1 ;
  array[current_size] = 0 ;

  if (max_size-current_size>=5) {
        array = shrink_array(array,max_size,current_size) ;
        max_size = current_size ;
  }

  return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {

        telemetry = append_to_array(t,telemetry,telemetry_current_size,telemetry_max_size);
        telemetry = append_to_array(x,telemetry,telemetry_current_size,telemetry_max_size);
        telemetry = append_to_array(y,telemetry,telemetry_current_size,telemetry_max_size);


    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
    }
  }

  return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {

  int pointers[tot_telemetries] ; //in each pointer we are recording which time coordinate we are at
  for (int i=0 ; i<tot_telemetries ; i++){
    pointers[i]=0; // initialize the pointer at 0
  } // check that no pb of pointesr/values here

  int expected = 0 ; // expected value of global_telemetry_max_size
  for (int i=0;i<tot_telemetries;i++){
    expected += telemetries_sizes[i];
  }

  double min_t = -1; //we are going to be looking for the smallest time thus we need a reference for comparison
  int min_telemetry = -1;

  while (global_telemetry_current_size != expected){ //while all coordinates have not been merged in global_telemetry

    min_t = -1;
    min_telemetry = -1;

    for (int i=0 ; i<tot_telemetries ; i++) { //i design the index of each telemetry in telemetries, we go through each index

      if (pointers[i]>=telemetries_sizes[i]){
          continue;
      }

      else {
          double time = telemetries[i][pointers[i]] ;

          if (min_t == double(-1)){
              min_t = time ;
              min_telemetry = i;
          }

          else if (time < min_t){ //i-th telemetry
              min_t = time; //we keep track of the telemetry with the smallest time
              min_telemetry = i ;
          }
      }


    }

    if (min_telemetry != -1) {
      global_telemetry = append_to_array(telemetries[min_telemetry][pointers[min_telemetry]],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);
      global_telemetry = append_to_array(telemetries[min_telemetry][pointers[min_telemetry]+1],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);
      global_telemetry = append_to_array(telemetries[min_telemetry][pointers[min_telemetry]+2],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);
      //remark that global_telemetry_max_size increases by 3
      pointers[min_telemetry] += 3; //in the telemetry with the smallest time, we move the pointer to the next point // pointers working
    }


  }



}
