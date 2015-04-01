#ifndef PQP_ENVIRONMENT_H_INCLUDED
#define PQP_ENVIRONMENT_H_INCLUDED

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <PQP/PQP.h>
#include <Eigen/Dense>
#include <flann/flann.hpp>

#include "dh_parameter.h"
#include "random_generator/random_space_generator_interface.h"

class PqpEnvironment {
  typedef flann::Index<flann::L2<double>> FlannPointArray;
  typedef Eigen::VectorXd EVectorXd;
public:
  PqpEnvironment(const std::string& robot_model_file,
                 const std::string& dh_table_file,
                 const std::string& obstacles_model_file,
                 RandomSpaceGeneratorInterface* random_generator,
                 const int sample_space_size = 1000000);

  int sample_space_size() { return conf_sample_space_->size(); }
  int dimension() { return dimension_; }
  double* GetPoint(int point_index) const {
    return conf_sample_space_->getPoint(point_index);
  }
  // Adds a point and returns it's index
  size_t AddPoint(EVectorXd& q);
  double CheckCollision(EVectorXd& q);
  // KnnQuery returns indexes
  std::vector<int> KnnQuery(EVectorXd& q, int k);

private:
  bool LoadRobotModel(const std::string& robot_mode_file);
  bool LoadDhTable(const std::string& dh_table_file);
  bool LoadObstacles(const std::string& obstacles_model_file);
  bool GenerateSampleSpace(
    RandomSpaceGeneratorInterface* random_generator,
    const int sample_space_size);
  PQP_Model* ParseModel(const std::string& model_file);

  std::unique_ptr<PQP_Model> obstacles_;
  std::vector<std::unique_ptr<PQP_Model>> segments_;
  std::vector<DhParameter> dh_table_;
  FlannPointArray* conf_sample_space_;
  int sample_space_size_;
  int dimension_;
};

#endif // PQP_ENVIRONMENT_H_INCLUDED
