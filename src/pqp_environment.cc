#include "pqp_environment.h"
#include <fstream>
#include <sstream>
#include <string>

PqpEnvironment::PqpEnvironment(const std::string& robot_model_file,
                               const std::string& dh_table_file,
                               const std::string& obstacles_model_file) :
                               conf_sample_space_(nullptr),
                               obstacles_(new PQP_Model) {
  LoadRobotModel(robot_model_file);
  LoadDhTable(dh_table_file);
  LoadObstacles(obstacles_model_file);
}

//TODO: Link number of segments
bool PqpEnvironment::LoadRobotModel(const std::string& robot_model_file) {
  std::ifstream input_file (robot_model_file.c_str());
  if (input_file) {
    try {
      input_file.seekg(0, std::ios::end);            //End of file
      std::streampos length (input_file.tellg());    //Read the size
      input_file.seekg(0, std::ios::beg);            //Return to beginning

      std::vector<char> buffer (length);
      input_file.read(&buffer[0], length);

      //Move buffer to stringstream parser
      std::stringstream parser;
      parser.rdbuf()->pubsetbuf(&buffer[0], length);

      const int num_segments (2);                    //TODO: Make a function
      PQP_REAL tri1[3][3], tri2[3][3];                       //to read these numbers
      segments_.resize(num_segments);
      for(int i = 0; i < num_segments; ++i)
      {
        segments_.at(i) = std::unique_ptr<PQP_Model> (new PQP_Model);
        segments_.at(i)->BeginModel();
        parser >> tri1[0][0] >> tri1[0][1] >> tri1[0][2] >>
                  tri1[1][0] >> tri1[1][1] >> tri1[1][2] >>
                  tri1[2][0] >> tri1[2][1] >> tri1[2][2];
        segments_.at(i)->AddTri(tri1[0], tri1[1], tri1[2], 0);
        parser >> tri2[0][0] >> tri2[0][1] >> tri2[0][2] >>
                  tri2[1][0] >> tri2[1][1] >> tri2[1][2] >>
                  tri2[2][0] >> tri2[2][1] >> tri2[2][2];
        segments_.at(i)->AddTri(tri2[0], tri2[1], tri2[2], 1);
        segments_.at(i)->EndModel();
      }
      //TODO: Add parsing options for two segment hand and test the results
      return true;
    }
    catch(...) { //TODO: Improve this catch block
      throw "File error!";
    }
  }
  return false; //Input file not present
}

bool PqpEnvironment::LoadDhTable(const std::string& dh_table_file) {
  std::ifstream input_file (dh_table_file.c_str());
  if (input_file) {
    try {
      input_file.seekg(0, std::ios::end);            //End of file
      std::streampos length (input_file.tellg());    //Read the size
      input_file.seekg(0, std::ios::beg);            //Return to beginning

      std::vector<char> buffer (length);
      input_file.read(&buffer[0], length);

      //Move buffer to stringstream parser
      std::stringstream parser;
      parser.rdbuf()->pubsetbuf(&buffer[0], length);

      const int num_segments (2); //Or make it go until parser.eof()
      double theta, d, a, alpha;
      for(int i = 0; i < num_segments; ++i)
      {
        parser >> theta >> d >> a >> alpha;
        dh_table_.emplace_back(theta, d, a, alpha);
      }
      return true;
    }
    catch(...) {
      throw "File error!";
    }
  }
  return false; //Input file not present
}

//Hopefully no modifications needed for general case
bool PqpEnvironment::LoadObstacles(const std::string& obstacles_model_file) {
  std::ifstream input_file (obstacles_model_file.c_str());
  if (input_file) {
    try {
      input_file.seekg(0, std::ios::end);            //End of file
      std::streampos length (input_file.tellg());    //Read the size
      input_file.seekg(0, std::ios::beg);            //Return to beginning

      std::vector<char> buffer (length);
      input_file.read(&buffer[0], length);

      //Move buffer to stringstream parser
      std::stringstream parser;
      parser.rdbuf()->pubsetbuf(&buffer[0], length);

      obstacles_->BeginModel();
      int counter (0);
      PQP_REAL tri[3][3];
      while(!parser.eof()) {
      parser >> tri[0][0] >> tri[0][1] >> tri[0][2] >>
                tri[1][0] >> tri[1][1] >> tri[1][2] >>
                tri[2][0] >> tri[2][1] >> tri[2][2];
        obstacles_->AddTri(tri[0], tri[1], tri[2], counter);
      }
      obstacles_->EndModel();
      return true;
    }
    catch(...) {
      throw "File error!";
    }
  }
  return false; //Input file not present
}

