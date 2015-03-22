#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>

void LoadRobot(const std::string& robot_model_file) {
  std::ifstream input_file(robot_model_file.c_str());
  if (input_file) {
    try {
      input_file.seekg(0, std::ios::end);            //End of file
      std::streampos length = input_file.tellg();    //Read the size
      input_file.seekg(0, std::ios::beg);            //Return to beginning

      std::vector<char> buffer(length);
      input_file.read(&buffer[0], length);

      //Move buffer to stringstream parser
      std::stringstream parser;
      parser.rdbuf()->pubsetbuf(&buffer[0], length);
      std::puts(parser.str().c_str());
      double d;
      parser >> d;
      printf("%lf\n", d);
      parser >> d;
      printf("%lf\n", d);
      parser >> d;
      printf("%lf\n", d);

      std::string s;
      parser.ignore(1000, '\n');
      parser.clear();
      std::getline(parser, s);
      printf("%s\n", s.c_str());
      parser >> s >> std::ws;
      printf("%s\n", s.c_str());
      parser >> s >> std::ws;
      printf("%s last\n", s.c_str());
      while(std::getline(parser, s)) {
        std::puts(s.c_str());
      }
      printf(parser ? "good\n" : "bad\n");
    }
    catch(...) {
      throw "File error!";
    }
  }
}

int main() {
  printf("%d\n", 15);
  LoadRobot("robot.txt");
  return 0;
}