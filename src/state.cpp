/*
 * state.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: Devin
 */

#include "state.h"
#include <math.h>

State::State() {
  x = 0.0;
  y = 0.0;
  phi = 0.0;
  v = 0.0; // assume start from rest
  omega = 0.0; // assume start from rest
  segDistDone = 0.0;
  spinAngDone = 0.0;
};

void State::updateState(double v_cmd, double omega_cmd, double dt) {
  double avg_v = (v+v_cmd)/2; // average the velocity over this time step
  double avg_omega = (omega+omega_cmd)/2; // average angular velocity over this time step
  double avg_phi = phi + avg_omega*dt/2; // average heading over time step

  x = x+avg_v*dt*cos(avg_phi); // advance x coordinate
  y = y+avg_v*dt*sin(avg_phi); // advance y coordinate
  phi = phi + avg_omega*dt; // advance the heading
  segDistDone = segDistDone+fabs(avg_v)*dt; // update the distance traveled
  spinAngDone = spinAngDone+fabs(avg_omega)*dt; // update the sin-in-place angle done

  v = v_cmd;
  omega = omega_cmd;
}

void State::stop()
{
  v = 0.0;
  omega = 0.0;
}

double State::getX() {
  return x;
}

double State::getY() {
  return y;
}

double State::getPhi() {
  return phi;
}

double State::getVCmd() {
  return v;
}

double State::getOCmd() {
  return omega;
}

double State::getDistDone() {
  return segDistDone;
}

double State::getAngDone() {
  return spinAngDone;
}
