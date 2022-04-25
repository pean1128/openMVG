// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2018 Yan Qingsong, Pierre Moulon

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef IO_READ_GT_MFT3D_HPP
#define IO_READ_GT_MFT3D_HPP

#include "io_readGTInterface.hpp"
#include "io_loadImages.hpp"

#include "openMVG/cameras/PinholeCamera.hpp"

#include <algorithm>

bool getPriorCamera(
	const std::string& prior_camera_file,
  Eigen::Matrix4d& Twc, Eigen::Matrix3d& K)
{	
  std::cerr << "[getPriorCamera] get prior camera information >> " << prior_camera_file << std::endl;

  std::ifstream fin(prior_camera_file);
  if (!fin.is_open())
  {
    std::cerr << "[getPriorCamera] can not open file >> " << prior_camera_file << std::endl;
    std::abort();
  }
  else
  {
    std::string line_in;
    std::getline(fin, line_in);
    if (line_in != "extrinsic") 
    {
      std::cerr << "[getPriorCamera] current line should be extrinsic!" << std::endl;
      exit(EXIT_FAILURE);
    }

    Twc.setIdentity();

    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    std::getline(fin, line_in);
    sscanf(line_in.c_str(), "%lf %lf %lf %lf", &R(0, 0), &R(0, 1), &R(0, 2), &t(0, 0));
    
    std::getline(fin, line_in);
    sscanf(line_in.c_str(), "%lf %lf %lf %lf", &R(1, 0), &R(1, 1), &R(1, 2), &t(1, 0));
    
    std::getline(fin, line_in);
    sscanf(line_in.c_str(), "%lf %lf %lf %lf", &R(2, 0), &R(2, 1), &R(2, 2), &t(2, 0));

    // Eigen::Vector3f C = -R.inverse() * t;
    Twc.block(0, 0, 3, 3) = R;
    Twc.block(0, 3, 3, 1) = t;

    std::cerr << R << std::endl;
    std::cerr << t << std::endl;

    std::getline(fin, line_in);
    std::getline(fin, line_in);

    std::getline(fin, line_in);
    if (line_in != "intrinsic") 
    {
      std::cerr << "[getPriorCamera] current line should be intrinsic!" << std::endl;;
      std::abort();
    }

    K.setIdentity();

    std::getline(fin, line_in);
    sscanf(line_in.c_str(), "%lf %lf %lf", &K(0, 0), &K(0, 1), &K(0, 2));
    
    std::getline(fin, line_in);
    sscanf(line_in.c_str(), "%lf %lf %lf", &K(1, 0), &K(1, 1), &K(1, 2));
    
          std::getline(fin, line_in);
    sscanf(line_in.c_str(), "%lf %lf %lf", &K(2, 0), &K(2, 1), &K(2, 2));

    std::cerr << K << std::endl;

		fin.close();
  }
  return true;
}

// The feature of the MFT3D's Data:
// 1. all the gt information are stored in one file
// 2. the cameras' intrinsic information are stored in camera.txt
class SfM_Data_GT_Loader_MFTMVS : public SfM_Data_GT_Loader_Interface
{
private:
  std::vector<cameras::PinholeCamera> cameras_data_; // Store all the camera information
public:
  bool loadGT() override
  {
    // Check all the files under the path
    std::vector<std::string> gt_files = stlplus::folder_files( this->gt_dir_ );

    // Read the camera file
    // Fix name "_cam.txt"
    // Reference:https://github.com/MFT3D/format-loader

    std::sort(gt_files.begin(), gt_files.end());
    std::stable_sort(gt_files.begin(), gt_files.end(), 
        [] (const std::string& a, const std::string& b) {return a.size() < b.size();});
    
    int image_number_count = static_cast<int>(gt_files.size());
    cameras_data_.reserve(image_number_count);

    int view_count = 0;
    for (const auto& gt_file : gt_files)
    {
      // std::ifstream camera_data_file( stlplus::create_filespec(this->gt_dir_, gt_file), std::ifstream::in);
      // if (!camera_data_file)
      // {
      //   std::cerr << "Error: Failed to open file '" << stlplus::create_filespec(this->gt_dir_, gt_file) << "' for reading" << std::endl;
      //   return false;
      // }
      // else 
      // {
      //   std::cerr << "read prior camera file >> " << stlplus::create_filespec(this->gt_dir_, gt_file) << std::endl;
      // }
      // camera_data_file.close();

      std::string git_file_absolute_path = stlplus::create_filespec(this->gt_dir_, gt_file);
      /* world to camera */
      Eigen::Matrix4d Twc; 
      Eigen::Matrix3d K;
      getPriorCamera(git_file_absolute_path, Twc, K);
      
      // Read image info line.
      Mat3 R;
      Vec3 t;
      R = Twc.block(0, 0, 3, 3);
      t = Twc.block(0, 3, 3, 1);

      Mat34 P;
      P_From_KRt(K, R, t, &P);
      cameras_data_.emplace_back(P);

      // Parse image name
      std::size_t npos = gt_file.find("_cam");
      std::string image_name = gt_file.substr(0, npos) + ".jpg";
      images_.emplace_back(image_name);

      if (view_count > 10)
      {
        break;
      }
      else 
      {
        view_count++;
      }
    }

    return true;
  }

  bool loadImages() override
  {
    return LoadImages(this->image_dir_, this->images_, this->cameras_data_, this->sfm_data_);
  }
};


#endif // IO_READ_GT_MFT3D_HPP
