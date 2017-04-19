#include <iostream>
#include "tools.h"
#include <stdexcept>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   TODO:
   * Calculate the RMSE here.
   */
  
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // TODO: YOUR CODE HERE
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }
  
  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    
    VectorXd residual = estimations[i] - ground_truth[i];
    
    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  
  //calculate the mean
  rmse = rmse/estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  
  try {
    float px2 = pow(px,2);
    float py2 = pow(py,2);
    float psq = sqrt(px2+py2);
    float p32 = pow((px2+py2),3/2);
    
    float h11 = px/psq;
    float h21 = py/psq;
    float h12 = -py/(px2+py2);
    float h22 = px/(px2+py2);
    float h13 = py*(vx*py-vy*px)/p32;
    float h23 = px*(vy*px-vx*py)/p32;
    float h33 = h11;
    float h43 = h21;
    
    //compute the Jacobian matrix
    Hj <<   h11,h21,0,0,
            h12,h22,0,0,
            h13,h23,h33,h43;
    
    
  }
  //check division by zero
  catch (std::logic_error e) {
    std::cerr << "division by zero" << e.what() << std::endl;
    
  }
  
  return Hj;
}
