#ifndef PQP_ENVIRONMENT_H_INCLUDED
#define PQP_ENVIRONMENT_H_INCLUDED

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include "PQP/PQP.h"
#include "Eigen/Dense"
#include <flann/flann.hpp>
#include "dh_parameter.h"
#include "random_generator/random_point_generator_interface.h"

class PqpEnvironment {
  typedef flann::Index<flann::L2<double>> FlannPointArray;
public:
  PqpEnvironment(const std::string& robot_model_file,
                 const std::string& dh_table_file,
                 const std::string& obstacles_model_file);

  double CheckCollision(std::vector<double> q);

private:
  bool LoadRobotModel(const std::string& robot_mode_file);
  bool LoadDhTable(const std::string& dh_table_file);
  bool LoadObstacles(const std::string& obstacles_model_file);
  std::unique_ptr<PQP_Model> obstacles_;
  std::vector<std::unique_ptr<PQP_Model>> segments_;
  std::vector<DhParameter> dh_table_;
  std::unique_ptr<FlannPointArray> conf_sample_space_;
};

#endif // PQP_ENVIRONMENT_H_INCLUDED
