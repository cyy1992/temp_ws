#include "generate_pbstream_from_pcd.h"
#include <cartographer/io/submap_painter.h>

#include <eigen_conversions/eigen_msg.h>
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "vtr_msgs/GlobalLocalizationPose.h"
#include "std_msgs/Empty.h"
#include "cartographer_ros/node_options.h"
#include <ros/package.h>
#include <cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace pcl;
// using namespace pcl::io;
// using namespace pcl::console;
using namespace std;
using namespace cv;
using namespace cartographer;
using namespace cartographer::mapping;

GeneratePbstreamFromPcd::GeneratePbstreamFromPcd()
{
  
}

GeneratePbstreamFromPcd::~GeneratePbstreamFromPcd()
{

}

void GeneratePbstreamFromPcd::getPbstream(const vector< Eigen::Vector3f >& points)
{
  std::vector< std::shared_ptr< cartographer::mapping::Submap3D > > submaps;
  
  submaps.push_back(PointsToSubmap(points));
  WriteToPbstream(submaps);
}

std::shared_ptr<Submap3D> GeneratePbstreamFromPcd::PointsToSubmap(const std::vector< Eigen::Vector3f >& points)
{
   
  cartographer::mapping::proto::RangeDataInserterOptions3D range_data_inserter_options;
  {
    range_data_inserter_options.set_hit_probability(0.8);
    range_data_inserter_options.set_miss_probability(0.3);
    range_data_inserter_options.set_num_free_space_voxels(2);
  }
  cartographer::mapping::RangeDataInserter3D range_data_inserter_(range_data_inserter_options);
  NodeData node_data;
  sensor::RangeData range_data;
  for(auto point:points)
  {
    sensor::RangefinderPoint temp;
    temp.position = point;
    temp.intensity = 0;
    range_data.returns.emplace_back(temp);
  }
  range_data.origin = Eigen::Vector3f(0,0,0);
  node_data.range_data = range_data;
  node_data.global_pose = transform::Rigid3d::Identity();
  node_data.local_from_gravity_aligned = Eigen::Quaterniond::Identity();
  node_data.rotational_scan_matcher_histogram_in_gravity = 
    scan_matching::RotationalScanMatcher::ComputeHistogram(
      sensor::TransformPointCloud(
      range_data.returns,
      transform::Rigid3f::Rotation(node_data.local_from_gravity_aligned.cast<float>())),
      120);
  const Eigen::VectorXf initial_rotational_scan_matcher_histogram =
  Eigen::VectorXf::Zero(node_data.rotational_scan_matcher_histogram_in_gravity.size());
      Eigen::Vector3f origin_in_tracking(0,0,0);
  Eigen::Vector3f origin_in_map(0,0,0);
  
  sensor::PointCloud high_resolution_point_cloud_in_tracking = node_data.range_data.returns;
  sensor::PointCloud low_resolution_point_cloud_in_tracking = node_data.range_data.returns;
  
  sensor::PointCloud high_resolution_point_cloud_in_map = high_resolution_point_cloud_in_tracking;
  
  sensor::PointCloud low_resolution_point_cloud_in_map = low_resolution_point_cloud_in_tracking;

  std::unique_ptr<cartographer::mapping::Submap3D> merge_submap(new mapping::Submap3D(
  0.2, 1.0, 
  transform::Rigid3d::Identity(),
  initial_rotational_scan_matcher_histogram));
  
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();    
  merge_submap->InsertData(origin_in_map,
                            high_resolution_point_cloud_in_map,
                            low_resolution_point_cloud_in_map,
                            range_data_inserter_,
                            q,
                            initial_rotational_scan_matcher_histogram);
  return merge_submap;
}
void bash(std::string cmd)
{
  LOG(INFO) << cmd ;
  auto status = system(cmd.c_str());
  assert(status != -1);
}
void GeneratePbstreamFromPcd::WriteToPbstream(const std::vector< std::shared_ptr< cartographer::mapping::Submap3D > >& submaps)
{
  int i=0;
  string path = "/home/cyy/map/temp3d/";
  boost::property_tree::xml_writer_settings<std::string> setting(' ', 2);
  for(auto& it:submaps)
  {
    string full_path = path + "frames/" + to_string(i) + "/";
    bash("mkdir -p " + full_path);
    {
      boost::property_tree::ptree p_map;
      transform::Rigid3d pose;
      transform::Rigid3d::Vector translation;
      transform::Rigid3d::Quaternion quaternion;
      std::string data;
      pose = transform::Rigid3d::Identity();
      translation = pose.translation();
      quaternion = pose.rotation();
      for (int i = 0; i < 3; i++)
        data = data + std::to_string(translation[i]) + " ";
      for (int i = 0; i < 4; i++)
        data = data + std::to_string(quaternion.coeffs()[i]) + " ";
      p_map.put("id", i);
      p_map.put("submap_global_pose", data);
      p_map.put("pose", data);
      boost::property_tree::write_xml(full_path + "data.xml", p_map,
                                      std::locale(), setting);
    }
    writeSubmap3DToPbstream(it, full_path + "submap.pbstream",i);
    i++;
  }
  saveShowImage(path);
}

void GeneratePbstreamFromPcd::writeSubmap3DToPbstream(const std::shared_ptr<Submap3D> & submap, 
                                           const string& save_path,
                                           const int& index)
{
  cartographer::io::ProtoStreamWriter writer(save_path);
  cartographer::mapping::proto::SerializedData proto;
  auto* const submap_proto = proto.mutable_submap();
  *submap_proto = submap->ToProto(true);
  submap_proto->mutable_submap_id()->set_trajectory_id(0);
  submap_proto->mutable_submap_id()->set_submap_index(index);
  writer.WriteProto(proto);
  writer.Close();
}

void GeneratePbstreamFromPcd::saveShowImage(const string& path)
{
  float save_image_resolution_ = 0.05;
  boost::property_tree::ptree pt;
  
  //get submap size
  {
    boost::property_tree::read_xml(path + "map.xml", pt);
    if (pt.empty())
    {
      const std::string error_string =
        "Load " + path + "map.xml failed!";
      
      throw std::runtime_error(error_string);
      return;
    }
  }
  int submap_cnt = pt.get<int>("map.property.node_count");
  std::vector<std::shared_ptr<cartographer::mapping::Submap3D> > submaps;
  std::vector<cartographer::transform::Rigid3d> global_poses;

  {
    for(int i =0; i < submap_cnt; i++)
    {
      std::shared_ptr<Submap3D> submap_ptr  = nullptr;
      cartographer::io::ProtoStreamReader stream(path + "frames/"+ to_string(i) + "/submap.pbstream");
    
      cartographer::mapping::proto::SerializedData proto;
      while(stream.ReadProto(&proto)) 
      {
        if(proto.data_case()==cartographer::mapping::proto::SerializedData::kSubmap) 
        {
          submap_ptr =
            std::make_shared<cartographer::mapping::Submap3D>(proto.submap().submap_3d());
        }
      }
      submaps.push_back(submap_ptr);
      
      //get submap global pose
      {
        boost::property_tree::ptree pt,pt_poses;
        boost::property_tree::read_xml(path + "frames/"+to_string(i) + "/data.xml", pt_poses);
        if(pt_poses.empty())
        {
          LOG(WARNING) << "data.xml is empty! ";
          return ;
        }
        std::string data;
        data = pt_poses.get<std::string>("submap_global_pose");
        double pose[7];
        sscanf(data.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &pose[0], &pose[1], &pose[2],
                &pose[3], &pose[4], &pose[5], &pose[6]);

        Eigen::Quaterniond q = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
        global_poses.push_back(cartographer::transform::Rigid3d({pose[0], pose[1], pose[2]}, q));
      }
    }
    
    std::map<cartographer::mapping::SubmapId,cartographer::io::SubmapSlice> submap_slices;

    for(int i=0;i < submap_cnt;i++)
    {
      submap_slices.insert(make_pair(SubmapId(0,i), 
                    submapToSlice(submaps[i],global_poses[i], true)));
    }
    
    auto painted_slices = cartographer::io::PaintSubmapSlices(submap_slices, save_image_resolution_);
    const Mat show_img = getMatMap(painted_slices);
    double max_box[2];
    max_box[0] = (show_img.rows - painted_slices.origin.x()) * save_image_resolution_;
    max_box[1] = painted_slices.origin.y() * save_image_resolution_;
    double size_of_img[2];
    size_of_img[0] = show_img.rows;
    size_of_img[1] = show_img.cols;
    
    {
      imwrite(path + "show_map.png",show_img);
      imwrite(path + "map.png",show_img);
      {
        boost::property_tree::ptree p_map;
        std::string data;
        data = to_string(save_image_resolution_);
        p_map.put("resolution", data);
        data.clear();
        data = to_string(max_box[0]) + " "
                + to_string(max_box[1]);
        p_map.put("max_box", data);
        data.clear();
        boost::property_tree::xml_writer_settings<std::string> setting(' ', 2);
        boost::property_tree::write_xml(path + "show_map_data.xml", p_map,
                                        std::locale(), setting);
      }
      
      {
        boost::property_tree::ptree p_map,p_top;
        std::string data;
        const int width = show_img.cols;
        const int height = show_img.rows;
        data = to_string(height);
        p_map.put("width", data);
        data.clear();
        data = to_string(width);
        p_map.put("height", data);
        data.clear();
        data = to_string(save_image_resolution_);
        p_map.put("resolution", data);data.clear();
        double x = max_box[0] / save_image_resolution_;
        double y = max_box[1] / save_image_resolution_;
        cartographer::transform::Rigid3d::Vector translation(y,x,0);
        cartographer::transform::Rigid3d::Quaternion quaternion(0, -sqrt(2)/2,sqrt(2)/2,0);
        for (int i = 0; i < 3; i++)
          data = data + std::to_string(translation[i]) + " ";
        for (int i = 0; i < 3; i++)
          data = data + std::to_string(quaternion.coeffs()[i]) + " ";
        data = data + std::to_string(quaternion.coeffs()[3]);
        p_map.put("pose", data);data.clear();
        p_top.add_child("mapPng", p_map);

        boost::property_tree::xml_writer_settings<std::string> setting(' ', 2);
        boost::property_tree::write_xml(path + "map_data.xml", p_top,
                                        std::locale(), setting);
      }

    }
  }
  LOG(WARNING) << "save all done";
}

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  geometry_msgs::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

geometry_msgs::Pose ToGeometryMsgPose(const cartographer::transform::Rigid3d& rigid3d) {
  geometry_msgs::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}
Eigen::Affine3d ToEigen(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return cartographer::transform::Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}
cartographer::io::SubmapSlice
GeneratePbstreamFromPcd::submapToSlice(const std::shared_ptr<Submap3D > submap,  
                                 const cartographer::transform::Rigid3d& global_pose,
                                 const bool& is_high_resolution)
{
  proto::SubmapQuery::Response response_proto;
  submap->ToResponseProto(global_pose, &response_proto);
  cartographer_ros_msgs::SubmapQuery::Response response;
  for (const auto& texture_proto : response_proto.textures())
  {
    response.textures.emplace_back();
    auto& texture = response.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
//    LOG(INFO) << texture.width <<", " <<texture.height ;
  }
  response.status.message = "Success.";
  
  cartographer::io::SubmapSlice submap_slice;
  submap_slice.pose = global_pose;

  auto fetched_textures = std::make_shared<::cartographer::io::SubmapTextures>();
//   fetched_textures->version = response.submap_version;
  for (const auto& texture : response.textures) {
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
    fetched_textures->textures.emplace_back(::cartographer::io::SubmapTexture{
        ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                              texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  
  CHECK(!fetched_textures->textures.empty());
  
  auto fetched_texture = fetched_textures->textures.begin();
  if(!is_high_resolution)
    fetched_texture +=1;
  submap_slice.width = fetched_texture->width;
  submap_slice.height = fetched_texture->height;
  submap_slice.slice_pose = fetched_texture->slice_pose;
  submap_slice.resolution = fetched_texture->resolution;
  submap_slice.cairo_data.clear();
  submap_slice.surface = ::cartographer::io::DrawTexture(
      fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
      fetched_texture->width, fetched_texture->height,
      &submap_slice.cairo_data);
  return submap_slice;
  
}

const Mat GeneratePbstreamFromPcd::getMatMap(const cartographer::io::PaintSubmapSlicesResult& painted_slices,MAP_FORMART format)
{
  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height = cairo_image_surface_get_height(painted_slices.surface.get());
  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  cv::Mat pub_img(height,width,CV_8UC4);
  cv::Mat show_img(height,width,CV_8UC4);
  cv::Mat save_img(height,width,CV_16UC1);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int delta =
        128 - int(color);
//      const unsigned char alpha = delta > 0 ? delta : -delta;
//      LOG(INFO) << int(alpha) <<"  ";
//      LOG(INFO) << int(color) ;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
//      if(observed!=0)
//      if (color == 128)
//        img.at<uint16_t>(y, x) = 0;
//      else
      if (format == PUB_MAP)
      {
        if (color == 128)
        {
          pub_img.at<Vec4b>(y, x)[0] = 128;
          pub_img.at<Vec4b>(y, x)[1] = 128;
          pub_img.at<Vec4b>(y, x)[2] = 128;
          pub_img.at<Vec4b>(y, x)[3] = 0;
        }
        else{
          pub_img.at<Vec4b>(y, x)[0] = color;
          pub_img.at<Vec4b>(y, x)[1] = color;
          pub_img.at<Vec4b>(y, x)[2] = color;
//          pub_img.at<Vec4b>(y, x)[3] = alpha;
  //        const int delta = 128 - int(value / 256);
          if(delta < 0)
          {
            pub_img.at<Vec4b>(y, x)[0] = 255;
            pub_img.at<Vec4b>(y, x)[1] = 255;
            pub_img.at<Vec4b>(y, x)[2] = 255;
          }
//          const unsigned char alpha1 = delta > 0 ? delta : -delta;
  //        const unsigned char value1 = delta > 0 ? delta : 0;
          const double alpha1 = delta > 0 ? delta : -delta;
          pub_img.at<Vec4b>(y, x)[3] = 255* sqrt(sqrt(alpha1/128.));
//          pub_img.at<Vec4b>(y, x)[3] = -255* log(1-alpha/128)
        }

      }
      else if(format == SAVE_MAP)
      {
        if(color == 128)
          save_img.at<uint16_t>(y, x) = 0;
        else
          save_img.at<uint16_t>(y, x) = color * 128;
      }
      else{
        show_img.at<Vec4b>(y, x)[0] = color;
        show_img.at<Vec4b>(y, x)[1] = color;
        show_img.at<Vec4b>(y, x)[2] = color;
        show_img.at<Vec4b>(y, x)[3] = 255;
//        show_img.at<uint8_t>(y, x) = color;
      }
    }
  }
  if(format == PUB_MAP){
    Mat pub_img_flip;
    transpose(pub_img,pub_img_flip);
    cv::flip(pub_img_flip,pub_img_flip,0);
    return pub_img_flip.clone();
  }
  else if(format == SAVE_MAP){
    Mat save_img_flip;
    transpose(save_img,save_img_flip);
    cv::flip(save_img_flip,save_img_flip,0);
    return save_img_flip.clone();
  }
  else{
    Mat show_img_flip;
    transpose(show_img,show_img_flip);
    cv::flip(show_img_flip,show_img_flip,0);
    return show_img_flip.clone();
  }
}
int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  //
  //*打开点云文件
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/cyy/saveMap.pcd", *cloud) == -1) {
      PCL_ERROR("Couldn't read file rabbit.pcd\n");
      return(-1);
  }
  std::cout << "Loaded:" << cloud->width*cloud->height<<"data points from test_pcd.pcd with the following fields:"<< std::endl;
  vector<Eigen::Vector3f> points;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    points.emplace_back(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
  }
  GeneratePbstreamFromPcd temp;
  temp.getPbstream(points);
  pcl::visualization::CloudViewer viewer("cloud viewer");
  viewer.showCloud(cloud);
  pause();

  
  return 0;
}
