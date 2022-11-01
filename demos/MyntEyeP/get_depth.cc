
#include <iostream>
#include <stdio.h>
#include <functional>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"

#include "util/cam_utils.h"
#include "util/counter.h"
#include "util/cv_painter.h"
#include <string.h>
#include <string>
#include "mynteyed/stubs/types_calib.h"
#include <util/pc_utils.cc>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <windows.h>
#include <iomanip>
#include <mynteyed/nlohmann/json.hpp>
#include <dirent.h>
#include <errno.h>
#include <opencv2/opencv.hpp>


MYNTEYE_USE_NAMESPACE
using json = nlohmann::json;
//
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#define CAMERA_FACTOR 1000.0
struct TimeStruct
{
    long hours;
    long minutes;
    long seconds;
    float milliseconds;
};
TimeStruct now_time()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now =
        boost::posix_time::microsec_clock::local_time();
    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();
    //
    // Extract hours, minutes, seconds and milliseconds.
    TimeStruct ts;
    ts.hours = td.hours();
    ts.minutes = td.minutes();
    ts.seconds = td.seconds();
    ts.milliseconds = td.total_milliseconds() -
        ((ts.hours * 3600 + ts.minutes * 60 + ts.seconds) * 1000);
    // Format like this:
    //      hh:mm:ss.SSS
    //char buf[40];
    //sprintf(buf, "%02ld:%02ld:%02ld.%03ld",hours, minutes, seconds, milliseconds);
    //return buf;
    return ts;
}
double time_diff_str(TimeStruct startingPoint, TimeStruct currentPoint) {
    float currentSec = currentPoint.milliseconds / 1000 + currentPoint.seconds + currentPoint.minutes * 60 + currentPoint.hours * 3600;
    float startingSec = startingPoint.milliseconds / 1000 + startingPoint.seconds + startingPoint.minutes * 60 + startingPoint.hours * 3600;
    float diffSec = currentSec - startingSec;
    std::string strTmp = std::to_string(diffSec);
    std::cout << strTmp + "\n";
    return diffSec;
}
void writeIntrinsicsToFile(CameraIntrinsics intrinsics, std::string files_path) {
    char intrinsics_path[255];
    strcpy(intrinsics_path, (files_path + "\\intrinsics.txt").c_str());
    FILE* file = fopen(intrinsics_path, "w");
    if (file == NULL) {
        printf("Error in writeIntrinsicsToFile\n");
    }
    char dataTypes[9][10] = { "coeffs","cx","cy","fx","fy","height","p","r","width" };
    char strToWrite[2295];
    strcpy(strToWrite, dataTypes[0]);
    strcat(strToWrite, " ");
    for (int i = 0; i < sizeof(intrinsics.coeffs) / sizeof(double); i++) {
        strcat(strToWrite, std::to_string(intrinsics.coeffs[i]).c_str());
        strcat(strToWrite, " ");
    }
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[1]);
    strcat(strToWrite, " ");
    strcat(strToWrite, std::to_string(intrinsics.cx).c_str());
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[2]);
    strcat(strToWrite, " ");
    strcat(strToWrite, std::to_string(intrinsics.cy).c_str());
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[3]);
    strcat(strToWrite, " ");
    strcat(strToWrite, std::to_string(intrinsics.fx).c_str());
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[4]);
    strcat(strToWrite, " ");
    strcat(strToWrite, std::to_string(intrinsics.fy).c_str());
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[5]);
    strcat(strToWrite, " ");
    strcat(strToWrite, std::to_string(intrinsics.height).c_str());
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[6]);
    strcat(strToWrite, " ");
    for (int i = 0; i < sizeof(intrinsics.p) / sizeof(double); i++){
        strcat(strToWrite, std::to_string(intrinsics.p[i]).c_str());
        strcat(strToWrite, " ");
    }
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[7]);
    strcat(strToWrite, " ");
    for (int i = 0; i < sizeof(intrinsics.r) / sizeof(double); i++) {
        strcat(strToWrite, std::to_string(intrinsics.r[i]).c_str());
        strcat(strToWrite, " ");
    }
    strcat(strToWrite, "\n");
    strcat(strToWrite, dataTypes[8]);
    strcat(strToWrite, " ");
    strcat(strToWrite, std::to_string(intrinsics.width).c_str());
    strcat(strToWrite, "\n");
    fwrite(strToWrite, (unsigned)strlen(strToWrite)*sizeof(char), 1, file);
    fclose(file);
}
void writeIntrinsicsToFileJson(CameraIntrinsics intrinsics, std::string files_path) {
    char intrinsics_path[255];
    strcpy(intrinsics_path, (files_path + "\\camera_intrinsic.json").c_str());
    /*FILE* file = fopen(intrinsics_path, "w");
    if (file == NULL) {
        printf("Error in writeIntrinsicsToFile\n");
    }
    char strToWrite[2295];*/
    std::ofstream output_file(intrinsics_path);
    json outJson;
    outJson["width"] = intrinsics.width;
    outJson["height"] = intrinsics.height;
    float intrinsics_arr[9] = {(float)intrinsics.fx,0, 0, 0, (float)intrinsics.fy, 0, (float)intrinsics.cx, (float)intrinsics.cy, 1 };
    outJson["intrinsic_matrix"] = intrinsics_arr;
    output_file << outJson.dump(4);
    output_file.close();
}
void writeTimeToFile(float time[],std::string path,int framenum) {
    char time_path[255];
    strcpy(time_path, (path + "\\acqTime.bin").c_str());
    FILE* file = fopen(time_path, "wb");
    if (file == NULL) {
        printf("Error in writingTimeToFile\n");
    }
    for (int i = 0; i < framenum; i++) {
        fwrite(&time[i], sizeof(&time[0]), 1, file);
    }
    fclose(file);
}
void saveFrames(std::string str_name,char str[],int iter,std::string files_path,int MAX_ROW, int MAX_COL,uint16_t** array3D) {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << iter;
    str_name = ss.str();
    strcpy(str, (files_path+"\\depth").c_str());
    strcat(str, "_");
    strcat(str, str_name.c_str());
    strcat(str, ".bin");
    FILE* file = fopen(str, "wb");
    if (file == NULL) {
        printf("Error");
    }
    for (int j = 0; j < MAX_ROW; ++j)
        fwrite(array3D[j], sizeof(array3D[j][0]), MAX_COL, file);
    fclose(file);
}
void createDirIfNotExists(std::string files_path) {
    DIR* dir = opendir(files_path.c_str());
    if (dir) {
        /* Directory exists. */
        closedir(dir);
    }
    else if (ENOENT == errno) {
        CreateDirectory(files_path .c_str(), NULL);
    }
    else {
        printf("Failed creating or opening directory");
    }
}
void savePNGColor(int index,std::string files_path, cv::Mat color) {
    std::stringstream ss1;
    ss1 << std::setw(6) << std::setfill('0') << index;
    std::string str_name = ss1.str();
    char str_1[255];
    strcpy(str_1, (files_path + "\\depth").c_str());
    strcat(str_1, "_");
    strcat(str_1, str_name.c_str());
    strcat(str_1, ".png");
    cv::imwrite(str_1, color);
}
CameraIntrinsics get_camera_intrinsics(const Camera& camera) {
    auto stream_mode = camera.GetOpenParams().stream_mode;
    return camera.GetStreamIntrinsics(stream_mode).left;
}
int main(int argc, char const* argv[]) {
  Camera cam;
  DeviceInfo dev_info;
  int imgWidth, imgHeight, fps,exp_time, global_gain, ir_intensity, framenum, acq_fps;
  std::string auto_exp_status,global_gain_status,awBalance,files_path;
  if (!util::select(cam, &dev_info)) {
    return 1;
  }
  // get camera parameters from user
  std::cout << "Advised depth modes to use: \n";
  std::cout << "640 x 480 at 60 fps \n";
  std::cout << "1280 x 720 at 30 fps \n";
  std::cout << "Insert image width (options: 640, 1280): ";
  std::cin >> imgWidth;
  std::cout << "Insert image heigth (options: 480, 720): ";
  std::cin >> imgHeight;
  std::cout << "Insert camera framerate (options: 30, 60): ";
  std::cin >> fps;
  std::cout << "Insert acquisition framerate: ";
  std::cin >> acq_fps;
  std::cout << "Enable autoexposure? (Y/N) ";
  std::cin >> auto_exp_status;
  if (auto_exp_status == "N") {
      //float time;
      //cam.GetExposureTime(time);
      //std::string time_str = std::to_string(time);
      std::cout << "Insert exposure time [1-655]: ";
      std::cin >> exp_time;
  }
  std::cout << "Set global gain? (Y/N) ";
  std::cin >> global_gain_status;
  if (global_gain_status == "Y") {
      std::cout << "Insert global gain [1-16]: ";
      std::cin >> global_gain;
  }
  std::cout << "Insert IR intensity [0-10] (default: 4): ";
  std::cin >> ir_intensity;
  std::cout << "Auto-white balance? (Y/N): ";
  std::cin >> awBalance;
  std::cout << "Enter the path to save files: ";
  std::cin >> files_path;
  std::cout << "Enter the number of frames to acquire: ";
  std::cin >> framenum;
  std::string color_path = files_path + "\\color";
  std::string depth_path = files_path + "\\depth";
  createDirIfNotExists(color_path);
  createDirIfNotExists(depth_path);
  //cout << "Open device: " << dev_info.index << ", "
    //  << dev_info.name << endl << endl;
  OpenParams params(dev_info.index);
  {
    // Framerate: 10(default), [0,60], [0,30](STREAM_2560x720)
    params.framerate = fps;
    
    // Color mode: raw(default), rectified
    params.color_mode = ColorMode::COLOR_RECTIFIED;

    // Depth mode: colorful(default), gray, raw
    // Note: must set DEPTH_RAW to get raw depth values
    params.depth_mode = DepthMode::DEPTH_RAW;
    // Stream mode: left color only
    if (imgWidth == 1280 && imgHeight == 720) {
        params.stream_mode = StreamMode::STREAM_1280x720; // hd
    }
    else {
        params.stream_mode = StreamMode::STREAM_640x480; // vga
    } 
    //params.stream_mode = StreamMode::STREAM_1280x720;  // hd
    // Stream mode: left+right color
    // params.stream_mode = StreamMode::STREAM_1280x480;  // vga
    // params.stream_mode = StreamMode::STREAM_2560x720;  // hd
    
    // Auto-exposure: true(default), false
    if (auto_exp_status == "Y") {
        params.state_ae = true;
    }
    else {
        params.state_ae = false;
    }
    // Auto-white balance: true(default), false
    if (awBalance == "Y") {
        params.state_awb = true;
    }
    else {
        params.state_awb = false;
    }
    // Infrared intensity: 0(default), [0,10]
    params.ir_intensity = ir_intensity;
  }
  util::print_stream_infos(cam, dev_info.index);
  cam.Open(params);
  if (auto_exp_status == "N") {
      cam.SetExposureTime(exp_time);
  }
  if (global_gain_status == "Y") {
      cam.SetGlobalGain(global_gain);
  }
  float exposure, gain;
  cam.GetExposureTime(exposure);
  cam.GetGlobalGain(gain);
  std::string exposure_str = std::to_string(exposure);
  std::string gain_str = std::to_string(gain);
  std::cout << "Exposure: " + exposure_str + "\n";
  std::cout << "Global gain: " + gain_str+"\n";
  //std::vector<mynteyed::DistanceData> vect;
  //vect = cam.GetDistanceDatas();
  CameraIntrinsics intrinsics;
  intrinsics = get_camera_intrinsics(cam);
  writeIntrinsicsToFileJson(intrinsics, files_path);
  //cout << endl;
  if (!cam.IsOpened()) {
    cerr << "Error: Open camera failed" << endl;
    return 1;
  }
  cout << "Open device success" << endl << endl;
  //cout << "Press ESC/Q on Windows to terminate" << endl;
  int MAX_ROW = imgHeight;
  int MAX_COL = imgWidth;
  //cv::namedWindow("color");
  //cv::namedWindow("depth");
  //cv::namedWindow("region");
  uint16_t** array3D = new uint16_t * [MAX_ROW];
  ushort** array3D_color = new ushort * [MAX_ROW];
  for (int i = 0; i < MAX_ROW; i++) {
    array3D[i] = new uint16_t[MAX_COL];
    array3D_color[i] = new ushort[MAX_COL];
  }
  //uint16_t** array2D = new uint16_t * [MAX_ROW];
  
  float* time = new float[framenum];
  long count = 0;
  char str[255];
  std::stringstream ss;
  std::string str_name;
  TimeStruct startingPoint;
  TimeStruct currentPoint;
  startingPoint=now_time();
  char key;
  int stopTime;
  //util::PCViewer viewer(1280, 720);
  std::cout << std::fixed;
  float clipping_distance = 1000.0;
  //std::cout << std::setprecision(9);
  for (int index=0;index<framenum;index++) {
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = util::get_point_cloud(&cam, CAMERA_FACTOR);
    //if (cloud) {
        //viewer.Update(cloud);
        //pcl::PointCloud<pcl::PointXYZRGBA> cloud2 = *cloud;
    //}
    //if (viewer.WasStopped()) break;
    cam.WaitForStream();
    //counter.Update();
    //std::string s_fxintrinsics.fx
    //std::string fx_str = std::to_string(intrinsics.fx);
    //std:cout << fx_str << endl;
    char str1[255] = "";
    auto image_depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    auto image_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    if ((image_depth.img) && (image_color.img)) {
      cv::Mat color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
      cv::Mat depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();

      for (int i = 0; i < depth.rows; i++) {
          for (int j = 0; j < depth.cols; j++) {
              if (depth.at<uint16_t>(i, j)<clipping_distance) {
                  array3D[i][j] = depth.at<uint16_t>(i, j);
              }
              else {
                  array3D[i][j] = 0;
              }
              //array3D_color[i][j] = color.at<cv::Vec3b>(100, 100)[0];
          }
      }
      cv::imshow("color", color);
      cv::imshow("depth", depth * 10);
      savePNGColor(index, color_path, color);
      //savePNGColor(index, array3D, depth);
      char key = cv::waitKey(1);
      if (key == 27) {
          break;
      }
      saveFrames(str_name, str, index, depth_path, MAX_ROW, MAX_COL, array3D);
      currentPoint = now_time();
      
      //std::string s = std::to_string(depth.at<uint16_t>(271, 485));
      //std::cout << s + "\n";
      //depth_region.ShowElems<ushort>(depth, [](const ushort& elem) {
      //  return std::to_string(elem);
      //}, 80, depth_info);
      
      //string ty = type2str(depth.type());
      //printf("Matrix: %s %dx%d \n", ty.c_str(), depth.cols, depth.rows);
      //std:string s=GetMatType(depth);
      time[count] = time_diff_str(startingPoint, currentPoint);
    }
    
    //const int type = CV_64F;
    //cv::Mat mat(10, 10, type);
    //std::cout << "Press ENTER to acquire one depth frame or insert \"q\" to quit (MAX 1000 frames allowed)";
    //key=getchar();
    stopTime = 1000 * 1 / acq_fps;
    Sleep(stopTime);
    //char key = static_cast<char>(cv::waitKey(0));
    //if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
    //  break;
    //}
    count += 1;
  }
  //for (int i = 0; i < framenum; i++) {
  //    saveFrames(str_name, str, i, files_path, MAX_ROW, MAX_COL, array3D[i]);
  //}
  for (int j = 0; j < MAX_ROW; j++) {
    delete[] array3D[j];
  }
  delete[] array3D;
  for (int j = 0; j < MAX_ROW; j++) {
      delete[] array3D_color[j];
  }
  delete[] array3D_color;
  writeTimeToFile(time,files_path,count+1);
  delete[] time;
  //for (int i = 0; i < MAX_ROW; i++) {
  //    delete[] array2D[i];
  //}
  //std:cout << time_diff_str(startingPoint, currentPoint);
  cam.Close();
  cv::destroyAllWindows();
  return 0;
}
