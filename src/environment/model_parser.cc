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
#include "model_parser.h"

#include <cmath>
#include <fstream>
#include <memory>

#include <PQP/PQP.h>
#include <Eigen/Dense>

PQP_Model* ModelParser::GetTransformModel(const std::string& model_file,
                                           const EMatrix& R,
                                           const EVector3f& T) {
  std::ifstream input_file (model_file.c_str(), std::ios::binary);
  try {
    std::unique_ptr<PQP_Model> model (new PQP_Model);

    char header[80] = "";         // Reads STL binary header
    input_file.read(header, 80);

    unsigned num_tris (0);  // Reads number of triangles
    input_file.read(reinterpret_cast<char*>(&num_tris), sizeof(num_tris));

    EVector3f vertex[3];  // Triangle represented as three vertices
    float surf_vec[3];

    model->BeginModel();

    short temp (0);  // Used for taking two bits of data after a triangle
    unsigned long counter (0);
    while(counter < num_tris) {
      input_file.read(reinterpret_cast<char*>(surf_vec), sizeof(surf_vec));

      input_file.read(reinterpret_cast<char*>(&vertex[0]), sizeof(vertex[0]));
      input_file.read(reinterpret_cast<char*>(&vertex[1]), sizeof(vertex[1]));
      input_file.read(reinterpret_cast<char*>(&vertex[2]), sizeof(vertex[2]));
      vertex[0] = R * vertex[0] + T;
      vertex[1] = R * vertex[1] + T;
      vertex[2] = R * vertex[2] + T;
      model->AddTri(vertex[0].data(), vertex[1].data(), vertex[2].data(),
        counter);

      input_file.read(reinterpret_cast<char*>(&temp), sizeof(temp));
      ++counter;
    }

    model->EndModel();
    return model.release();
  }
  catch(...) {
    throw "File " + model_file + " error!";
  }
}

// TODO: Try to get rid of code repetition
PQP_Model* ModelParser::GetModel(const std::string& model_file) {
  std::ifstream input_file (model_file.c_str(), std::ios::binary);
  try {
    std::unique_ptr<PQP_Model> model (new PQP_Model);

    char header[80] = "";         // Reads STL binary header
    input_file.read(header, 80);

    unsigned num_tris (0);  // Reads number of triangles
    input_file.read(reinterpret_cast<char*>(&num_tris), sizeof(num_tris));

    float vertex[3][3];  // Triangle represented as three vertices
    float surf_vec[3];

    model->BeginModel();

    short temp (0);  // Used for taking two bits of data after a triangle
    unsigned long counter (0);
    while(counter < num_tris) {
      input_file.read(reinterpret_cast<char*>(surf_vec), sizeof(surf_vec));

      input_file.read(reinterpret_cast<char*>(&vertex[0]), sizeof(vertex[0]));
      input_file.read(reinterpret_cast<char*>(&vertex[1]), sizeof(vertex[1]));
      input_file.read(reinterpret_cast<char*>(&vertex[2]), sizeof(vertex[2]));
      model->AddTri(vertex[0], vertex[1], vertex[2], counter);

      input_file.read(reinterpret_cast<char*>(&temp), sizeof(temp));
      ++counter;
    }

    model->EndModel();
    return model.release();
  }
  catch(...) {
    throw "File " + model_file + " error!";
  }
}

