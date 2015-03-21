#ifndef PQP_ENVIRONMENT_H_INCLUDED
#define PQP_ENVIRONMENT_H_INCLUDED

#include <vector>
#include <string>
#include <memory>
#include "PQP/PQP.h"
#include "Eigen/Dense"
#include <flann/flann.hpp>

class PqpEnvironment {
public:
  PqpEnvironment (const std::string& robot_model_file, 
                  const std::string& dh_table_file,
                  const std::string& obstacles_model_file);
  
  double CheckCollision (std::vector<double> q);
private:
  bool LoadRobot (const std::string& robot_mode_file);
  bool LoadDh (const std::string& dh_table_file);
  bool LoadObstacles (const obstacles_model_file);
  std::unique_ptr<PQP_Model> obstacles_;
  std::vector<std::unique_ptr<PQP_Model>> segments_;
  flann::Index<flann::L2<double>> conf_sample_space_;
};


#endif // PQP_ENVIRONMENT_H_INCLUDED
