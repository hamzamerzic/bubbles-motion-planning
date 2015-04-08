/*
 * Copyright (C) 2015 Hamza Merzić
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef MODEL_PARSER_H_INCLUDED
#define MODEL_PARSER_H_INCLUDED

#include <string>
#include <cmath>
#include <PQP/PQP.h>
#include <Eigen/Dense>

class ModelParser {
  typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> EMatrix;
  typedef Eigen::Vector3f EVector3f;
public:
  ModelParser() {}
  PQP_Model* GetTransformModel(const std::string& robot_model_file,
                             const EMatrix& R, const EVector3f& T);
  PQP_Model* GetModel(const std::string& model_file);
  // TODO: Add robot parameters getter

};

#endif // MODEL_PARSER_H_INCLUDED