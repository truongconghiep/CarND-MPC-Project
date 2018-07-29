#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define LF (float)2.67

/* MPC cost weights */
#define CTE_COST_WEIGHT                 (double)2000
#define EPSI_COST_WEIGHT                (double)2000
#define DELTA_COST_WEIGHT_PROPORTIONAL  (double)35
#define A_COST_WEIGHT_PROPORTIONAL      (double)35
#define DELTA_COST_WEIGHT_DIFFERENT     (double)5000000
#define A_COST_WEIGHT_DIFFERENT         (double)20000

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
