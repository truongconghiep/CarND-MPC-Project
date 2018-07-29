#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define LF (float)2.67

typedef unsigned int uint_t;

using namespace std;

class MPC {
 public:
  const double Lf = LF;
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve( Eigen::VectorXd state, 
                        Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
