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
#include <ThreadPool.h>
#include <vector>
#include <chrono>
#include <string>
#include <memory>
#include <fstream>
#include <cassert>
#include <functional>

using namespace std;
using namespace cv;

StringOperator::StringOperator()
{

}

StringOperator::~StringOperator()
{

}

struct s1{
  Eigen::Vector3d a;
  Eigen::Vector3i b;
};
void testPrint(const Eigen::Vector3d a= {0,0,1})
{
  cout << a.transpose() <<endl;
}
int main(int argc, char** argv)
{
//   ThreadPool pool(4);
//   std::vector< std::future<int> > results;
// 
//   for(int i = 0; i < 8; ++i) {
//       results.emplace_back(
//           pool.enqueue([i] {
//               std::cout << "hello " << i << std::endl;
//               std::this_thread::sleep_for(std::chrono::seconds(1));
//               std::cout << "world " << i << std::endl;
//               return i*i;
//           })
//       );
//   }
// 
//   for(auto && result: results)
//       std::cout << result.get() << ' ';
//   std::cout << std::endl;
//   
//   return 0;
  Eigen::Isometry3f lidar2base_;
    Eigen::Vector3f t1111(lidar2base_.translation());
  Eigen::Quaternionf q1111(lidar2base_.rotation());
  
  std::queue<std::unique_ptr<int>> temp_string;

//   std::unique_ptr<int> a1 = std::make_unique<int>(1);
  cout << (temp_string.front() == nullptr) <<endl;;
  std::unique_ptr<int> a2 ;//= std::make_unique<int>(2);
//   std::unique_ptr<string> a2("aaaadf");
  temp_string.push(std::make_unique<int>(1));
  temp_string.push(std::make_unique<int>(2));
  
  std::unique_ptr<int> a3 = move(a2);
  a2 = std::make_unique<int>(662);
//   cout << *a3 <<endl;
//   cout << *a2 <<endl;
  a3  = move(a2);
    cout << *a3 <<endl;

//   temp_string.push_back(move(a2));
  s1 s;
  cout << s.b.transpose() <<endl;
  return 1;
  std::vector<int> a{3,4,5,6,7,8,1,2};
  for(vector<int>::iterator it = a.begin(); it != a.end(); )
  {
    if(*it >5)
    {
      a.erase(it);
      continue;
    }
    
    cout << *it;
    it++;
  }
  return 1;
  testPrint(Eigen::Vector3d(1,2,3));
  Eigen::Vector3d eul(0.1,0.,0);
  Eigen::Matrix3d matrix_tmp;
  matrix_tmp = (Eigen::AngleAxisd(eul[0],Eigen::Vector3d::UnitZ()) 
        * Eigen::AngleAxisd(eul[1],Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(eul[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Quaterniond q1(matrix_tmp);

//   Eigen::Quaterniond q1(0,0,0,1);
//   Eigen::Quaterniond q = Eigen::Quaterniond::Identity().slerp(0.1,q1);
//   cout << q1.toRotationMatrix().eulerAngles(2,1,0).transpose()<<endl;
//   cout << q.toRotationMatrix().eulerAngles(2,1,0).transpose()<<endl;
  return 1;
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