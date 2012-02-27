/*
 * state.h
 *
 *  Created on: Feb 27, 2012
 *      Author: Devin
 */

#ifndef STATE_H_
#define STATE_H_

class State
{
public:
  State();
  void updateState(double v_cmd, double omega_cmd, double dt);
  void stop();
  // getters
  double getX();
  double getY();
  double getPhi();
  double getVCmd();
  double getOCmd();
  double getDistDone();
  double getAngDone();
private:
  double x,y,phi,v,omega,segDistDone,spinAngDone;
};

#endif /* STATE_H_ */
