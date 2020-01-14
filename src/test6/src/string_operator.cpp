/*
 * Copyright 2020 <copyright holder> <email>
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

#include "string_operator.h"
#include <glog/logging.h>
using namespace std;
StringOperator::StringOperator()
{

}

StringOperator::~StringOperator()
{

}

int main(int argc, char** argv)
{
  const std::string path = "/home/cyy/map/.1010_g2/";
//   std::size_t t_position = path.rfind('/');
// //   std::string id_string = path.substr(0,t_position);
// //   const int pos = t_position;
// //   cout << path <<endl;
//   std::size_t t_position2 = path.rfind('/',t_position-1);
//   std::string map_name = path.substr(t_position2+1,t_position-t_position2-1);
//   std::string map_path = path.substr(0,t_position2+1);
  std::size_t t_position = path.rfind('.');
  if(t_position == path.npos)
    cout << "not find" <<endl;
  else{
    cout << t_position <<endl;
    std::string path1 = path.substr(0,t_position);
    std::string path2 = path.substr(t_position+1);
    cout <<  path1 + path2<<endl;
  }
//   cout << id_string2 <<endl;
//   cout << t_position <<endl;
//   cout << t_position2 <<endl;

//   LOG(INFO) << id_string;
  return 0;
}