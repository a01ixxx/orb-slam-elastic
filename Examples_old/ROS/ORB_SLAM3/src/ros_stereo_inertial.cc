/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include <unistd.h>

// For saving files
#include <fstream>

// For queue
#include <ros/callback_queue.h>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

// For elastic scheduling
#include "harmonic.h"
#include"../../../include/ElasticParameters.h"


using namespace std;


// Ao added
///////////////
// Structure to store CPU statistics
typedef struct {
    unsigned long long user;
    unsigned long long nice;
    unsigned long long system;
    unsigned long long idle;
    unsigned long long iowait;
    unsigned long long irq;
    unsigned long long softirq;
} CPUStats;

vector<std::pair<double, double>> imu_exe_times;
vector<std::pair<double, double>> left_camera_exe_times;
vector<std::pair<double, double>> right_camera_exe_times;
vector<std::pair<double, double>> tracking_exe_times;
vector<std::pair<double, double>> ba_exe_times;
vector<std::pair<double, double>> fusion_exe_times;
vector<std::pair<double, double>> loop_closing_exe_times;


/*
 * Skip factors to control the periods
 */
int image_to_skip = 1;
int image_count = 0;

int imu_to_skip = 1;
int imu_count = 0;

int ba_to_skip = 1;
int ba_count = 0;

#ifdef ELASTIC_SCHED
Harmonic_Elastic elastic_space {3};
#endif
// First Task -- IMU
// Second Task -- Image
// Third Task -- BA

bool imu_period_need_update = false;
bool image_period_need_update = false;
bool ba_period_need_update = false;

CPUStats last_stats, current_stats;
double cpu_utilization;


// Function to read CPU statistics from /proc/stat
void read_cpu_stats(CPUStats *stats) {
    FILE *stat_file = fopen("/proc/stat", "r");
    if (!stat_file) {
        perror("Failed to open /proc/stat");
        exit(EXIT_FAILURE);
    }

    char line[256];
    while (fgets(line, sizeof(line), stat_file)) {
        if (strncmp(line, "cpu ", 4) == 0) {
            sscanf(line + 4, "%llu %llu %llu %llu %llu %llu %llu",
                   &stats->user, &stats->nice, &stats->system, &stats->idle,
                   &stats->iowait, &stats->irq, &stats->softirq);
            break;
        }
    }

    fclose(stat_file);
}

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void m_GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void imu_thread_function();

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    void m_GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg);
    void right_image_thread_function();

    void m_GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg);
    void left_image_thread_function();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


/* Ao Added - time_saver()
 * For logging execution times profiling results
 */
int times_saver() {

    // Open a file in write mode
    std::ofstream imu_exe_times_file("us_imu_exe_times_file.txt");

    // Check if the file is open
    if (!imu_exe_times_file.is_open()) {
        std::cerr << "Unable to open file";
        return 1;
    }
    // Write the vector data to the file
    for (const auto& val : imu_exe_times) {
        imu_exe_times_file << setprecision(19) << val.first << setprecision(6) << "," << val.second << '\n';
    }
    // Close the file
    imu_exe_times_file.close();
    // End

//////////////////////////////////////////////

    // Open a file in write mode
    std::ofstream left_camera_exe_times_file("us_left_camera_exe_times_file.txt");

    // Check if the file is open
    if (!left_camera_exe_times_file.is_open()) {
        std::cerr << "Unable to open file";
        return 1;
    }
    // Write the vector data to the file
    for (const auto& val : left_camera_exe_times) {
        left_camera_exe_times_file << setprecision(19) << val.first << setprecision(6) << "," << val.second << '\n';
    }
    // Close the file
    left_camera_exe_times_file.close();
    // End

//////////////////////////////////////////////

    // Open a file in write mode
    std::ofstream right_camera_exe_times_file("us_right_camera_exe_times_file.txt");

    // Check if the file is open
    if (!right_camera_exe_times_file.is_open()) {
        std::cerr << "Unable to open file";
        return 1;
    }
    // Write the vector data to the file
    for (const auto& val : right_camera_exe_times) {
        right_camera_exe_times_file << setprecision(19) << val.first << setprecision(6) << "," << val.second << '\n';
    }
    // Close the file
    right_camera_exe_times_file.close();
    // End

//////////////////////////////////////////////

    // Open a file in write mode
    std::ofstream tracking_exe_times_file("ms_tracking_exe_times_file.txt");

    // Check if the file is open
    if (!tracking_exe_times_file.is_open()) {
        std::cerr << "Unable to open file";
        return 1;
    }
    // Write the vector data to the file
    for (const auto& val : tracking_exe_times) {
        tracking_exe_times_file << setprecision(19) << val.first << setprecision(6) << "," << val.second << '\n';
    }
    // Close the file
    tracking_exe_times_file.close();
    // End
//////////////////////////////////////////////
      // Open a file in write mode
    std::ofstream ba_exe_times_file("ms_ba_exe_times_file.txt");

    // Check if the file is open
    if (!ba_exe_times_file.is_open()) {
        std::cerr << "Unable to open file";
        return 1;
    }
    // Write the vector data to the file
    for (const auto& val : ba_exe_times) {
        ba_exe_times_file << setprecision(19) << val.first << setprecision(6) << "," << val.second << '\n';
    }
    // Close the file
    ba_exe_times_file.close();
    // End

    // Open a file in write mode
    std::ofstream loop_closing_exe_times_file("ms_loop_closing_exe_times_file.txt");

    // Check if the file is open
    if (!loop_closing_exe_times_file.is_open()) {
        std::cerr << "Unable to open file";
        return 1;
    }
    // Write the vector data to the file
    for (const auto& val : loop_closing_exe_times) {
        loop_closing_exe_times_file << setprecision(19) << val.first << setprecision(6) << "," << val.second << '\n';
    }
    // Close the file
    loop_closing_exe_times_file.close();
    // End
  std::cout << "Timing information logging finished" << std::endl;
  return 0;
} // End - time_saver()

///////////////////////////////////////////////////////////////
void ImuGrabber::m_GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // if(imu_period_need_update == true) {
    //   // set up the period

    //   imu_period_need_update = false;
    // }
    if (++imu_count < imu_to_skip) {

#ifdef DEBUG_HARMONIC
    std::cout << "imu frame skipped" << imu_to_skip << std::endl;
#endif
      return;
    }

    imu_count = 0;
    // struct timespec res;
    // if (clock_getres(CLOCK_THREAD_CPUTIME_ID, &res) == -1) {
    //     perror("clock_getres");
    //     return;
    // }
    // printf("Resolution: %ld seconds and %ld nanoseconds\n", res.tv_sec, res.tv_nsec);

    struct timespec start, end;
    
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  // Check the pthread id
    // pthread_t tid = pthread_self();
    // printf("This is in my crafted queue Thread ID: %lu\n", (unsigned long)tid);
  // End Check the pthread

   mBufMutex.lock();
   imuBuf.push(imu_msg);
   mBufMutex.unlock();

  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);

  double time_spent = (end.tv_sec - start.tv_sec) * 1000000.0 +
                      (end.tv_nsec - start.tv_nsec) / 1000.0;
  double timestamp = imu_msg->header.stamp.toSec();
  std::pair<double, double> curr_pair = std::make_pair(timestamp, time_spent);
  imu_exe_times.push_back(curr_pair);

// End of Imu Driver
  return;
}

ros::CallbackQueue imu_queue;

void ImuGrabber::imu_thread_function()
{
    ros::NodeHandle imu_nh;

    imu_nh.setCallbackQueue(&imu_queue);
    ros::Subscriber sub = imu_nh.subscribe("/imu", 1000, &ImuGrabber::m_GrabImu, this);

    ros::Rate rate(400);  // 10 Hz
    while (ros::ok())
    {
        imu_queue.callAvailable();
        rate.sleep();
    }
}
////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////
void ImageGrabber::m_GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Check the pthread id
    // pthread_t tid = pthread_self();
    // printf("Right image Thread ID: %lu\n", (unsigned long)tid);
    // End Check the pthread

    struct timespec start, end; 
    // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
      imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
    double time_spent = (end.tv_sec - start.tv_sec) * 1000000.0 +
                          (end.tv_nsec - start.tv_nsec) / 1000.0;
    double timestamp = img_msg->header.stamp.toSec();
    std::pair<double, double> curr_pair = std::make_pair(timestamp, time_spent);
    right_camera_exe_times.push_back(curr_pair);

    // right_camera_exe_times.push_back(time_spent);
}

ros::CallbackQueue right_img_queue;

void ImageGrabber::right_image_thread_function()
{
    ros::NodeHandle right_img_nh;

    right_img_nh.setCallbackQueue(&right_img_queue);
    ros::Subscriber sub = right_img_nh.subscribe("/camera/right/image_raw", 100, &ImageGrabber::m_GrabImageRight, this);

    ros::Rate rate(40);  // 10 Hz
    while (ros::ok())
    {
        right_img_queue.callAvailable();
        rate.sleep();
    }
}
////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////
void ImageGrabber::m_GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Check the pthread id
    // pthread_t tid = pthread_self();
    // printf("Left image Thread ID: %lu\n", (unsigned long)tid);
    // End Check the pthread

    struct timespec start, end;
      
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
      imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);

    double time_spent = (end.tv_sec - start.tv_sec) * 1000000.0 +
                          (end.tv_nsec - start.tv_nsec) / 1000.0;

        
    double timestamp = img_msg->header.stamp.toSec();
    std::pair<double, double> curr_pair = std::make_pair(timestamp, time_spent);
    left_camera_exe_times.push_back(curr_pair);
}

ros::CallbackQueue left_img_queue;

void ImageGrabber::left_image_thread_function()
{
    ros::NodeHandle left_img_nh;

    left_img_nh.setCallbackQueue(&left_img_queue);
    ros::Subscriber sub = left_img_nh.subscribe("/camera/left/image_raw", 100, &ImageGrabber::m_GrabImageLeft, this);

    ros::Rate rate(40);  // 10 Hz
    while (ros::ok())
    {
        left_img_queue.callAvailable();
        rate.sleep();
    }
}
////////////////////////////////////////////////////

void update_cpu_utilization() {

#ifdef DEBUG_OVERHEAD
  struct timespec t1, t2, t3, t4, t5;
  float tmp_reader_1, tmp_reader_2, tmp_reader_3;
  double time_spent_1, time_spent_2, time_spent_3, time_spent_4;
#endif

  while (true) {
#ifdef DEBUG_OVERHEAD
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
#endif
      // Your code to be executed periodically goes here
      read_cpu_stats(&current_stats);

      unsigned long long total1 = last_stats.user + last_stats.nice + last_stats.system + last_stats.idle + last_stats.iowait + last_stats.irq + last_stats.softirq;
      unsigned long long total2 = current_stats.user + current_stats.nice + current_stats.system + current_stats.idle + current_stats.iowait + current_stats.irq + current_stats.softirq;
      unsigned long long total_diff = total2 - total1;
      unsigned long long idle_diff = current_stats.idle - last_stats.idle;
      cpu_utilization = ((double)(total_diff - idle_diff) / total_diff) * 100.0;
#ifdef DEBUG_OVERHEAD
      std::cout << "cpu_utilization" << cpu_utilization << std::endl;
#endif

    // Emulate the CPU bandwidth
    std::srand(std::time(0));  // Use current time as seed for random generator
    int random_number = std::rand() % (BANDWIDTH_RANGE + 1) + BANDWIDTH_MIN;

    int current_bandwidth = random_number;
#ifdef DEBUG_OVERHEAD
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
#endif
    // Get the period for each tasks
    char command[100];
    sprintf(command, "sudo cgset -r cpu.cfs_quota_us=%d orb_cgroup", current_bandwidth * 1000);
    system(command);
    system("sudo cgset -r cpu.cfs_period_us=100000 orb_cgroup");  
    std::cout << "The CPU Bandwidth is modified to " << current_bandwidth << std::endl;

#ifdef DEBUG_OVERHEAD
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t3);
#endif
#ifdef ELASTIC_SCHED
    elastic_space.assign_periods(current_bandwidth/ 100.0);
#endif

#ifdef DEBUG_OVERHEAD
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t4);
#endif

#ifdef DEBUG_OVERHEAD

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t5);
#ifdef ELASTIC_SCHED
    tmp_reader_1 = elastic_space.get_tasks()[0].t;
    tmp_reader_2 = elastic_space.get_tasks()[1].t;
    tmp_reader_3 = elastic_space.get_tasks()[2].t;

    std::cout << "Current bandwidth : " << current_bandwidth << std::endl;
    std::cout << "Task 1 period : " << tmp_reader_1 << std::endl;
    std::cout << "Task 2 period : " << tmp_reader_2 << std::endl;
    std::cout << "Task 3 period : " << tmp_reader_3 << std::endl;
#endif

    time_spent_1 = (t2.tv_sec - t1.tv_sec) * 1000000.0 +
                          (t2.tv_nsec - t1.tv_nsec) / 1000.0;
    
    time_spent_2 = (t3.tv_sec - t2.tv_sec) * 1000000.0 +
                          (t3.tv_nsec - t2.tv_nsec) / 1000.0;

    time_spent_3 = (t4.tv_sec - t3.tv_sec) * 1000000.0 +
                          (t4.tv_nsec - t3.tv_nsec) / 1000.0;

    time_spent_4 = (t5.tv_sec - t4.tv_sec) * 1000000.0 +
                          (t5.tv_nsec - t4.tv_nsec) / 1000.0;
#endif

    // std::cout << "Time (1) : " << time_spent_1 << std::endl;
    // std::cout << "Time (2) : " << time_spent_2 << std::endl;
    // std::cout << "Time (3) : " << time_spent_3 << std::endl;
    // std::cout << "Time (4) : " << time_spent_4 << std::endl;

#ifdef ELASTIC_SCHED
    if (elastic_space.get_tasks()[0].t == 0) {
          std::cout << "Current bandwidth : " << current_bandwidth << std::endl;
    } else {

    auto & tasks = elastic_space.get_tasks();
    imu_to_skip = tasks[0].t / 5; 
    image_to_skip = (tasks[1].t) / 50; 
    // BA skip should account for image's skip factor
    ba_to_skip =  tasks[2].t / (50 * image_to_skip);
#ifdef DEBUG_HARMONIC
    std::cout << "Adjusted periods: " << tasks[0].t << ' ' << tasks[1].t << ' ' << tasks[2].t << std::endl;
    std::cout << "Skipping: " << imu_to_skip << ' ' << image_to_skip << ' ' << ba_to_skip << std::endl;
#endif
    }
#endif

    // imu_to_skip = 1;
    // image_to_skip = 1;
    // std::cout << "Current bandwidth : " << current_bandwidth << std::endl;
    // std::cout << "imu_to_skip : " << imu_to_skip << std::endl;
    // std::cout << "image_to_skip : " << image_to_skip << std::endl;
    // std::cout << "ba_to_skip : " << ba_to_skip << std::endl;

    /////////////////////
    last_stats.user = current_stats.user;
    last_stats.nice = current_stats.nice;
    last_stats.system = current_stats.system;
    last_stats.idle = current_stats.idle;
    last_stats.iowait = current_stats.iowait;
    last_stats.irq = current_stats.irq;
    last_stats.softirq = current_stats.softirq;
    ////////////////////
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Sleep for 1 second
  }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 7)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

// Set the RR scheduling
    // pid_t pid = getpid();
    // struct sched_param param;
    // param.sched_priority = 20;
    // if (sched_setscheduler(pid, SCHED_RR, &param) == -1) {
    //     perror("sched_setscheduler");
    //     return 1;
    // }
// End - Set the RR scheduling


#ifdef RESTRICT_BANDWIDTH
  // Set the CGROUP
  // Step 1: Create a cgroup
  system("sudo cgcreate -g cpu:orb_cgroup");

  char command[100];
  sprintf(command, "sudo cgclassify -g cpu:orb_cgroup %d", getpid());
  system(command);
  //Set current bandwidth back to 100
    sprintf(command, "sudo cgset -r cpu.cfs_quota_us=100000 orb_cgroup");
    system(command);
    system("sudo cgset -r cpu.cfs_period_us=100000 orb_cgroup");  
  system(command);

  // End - Set the CGROUP
#endif

// To collect elasticity data

  std::string sbRect(argv[3]);
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  if (argc == 7) {
    image_to_skip = std::stoi(argv[5]);
    imu_to_skip= std::stoi(argv[6]);
  }
// End - To collect elasticity data


#ifdef ELASTIC_SCHED

// First Task -- IMU
// Second Task -- Image
// Third Task -- BA
  //Task takes T_min, T_max, C, E
  // elastic_space.add_task(Task {5, 20, 0.0015, 0.263});
  // elastic_space.add_task(Task {50, 200, 19.5, 4006});
  // elastic_space.add_task(Task {50, 1200, 270,  114000});

  elastic_space.add_task(Task {5, 20, 0.0015, 0.263});
  elastic_space.add_task(Task {50, 200, 19.5, 4006});
  elastic_space.add_task(Task {50, 1200, 163,  114000});
  elastic_space.generate();

#endif

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);

// Avoid runtime re-allocation
  imu_exe_times.reserve(30000);
  left_camera_exe_times.reserve(3000);
  right_camera_exe_times.reserve(3000);
  tracking_exe_times.reserve(3000);
  fusion_exe_times.reserve(3000);
  ba_exe_times.reserve(3000);

  std::cout << "Hello Whale" << std::endl;

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,sbRect == "true",bEqual);
  
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // Maximum delay, 5 seconds
  // ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  // ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  // ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  std::thread imu_grab_thread(&ImuGrabber::imu_thread_function, &imugb);
  std::thread right_img_grab_thread(&ImageGrabber::right_image_thread_function, &igb);
  std::thread left_img_grab_thread(&ImageGrabber::left_image_thread_function, &igb);

#ifdef RESTRICT_BANDWIDTH
  std::thread t(update_cpu_utilization);
#endif

  ros::AsyncSpinner spinner(4);  // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  // ros::spin();
  cout << "I am saving the trajectories and execution times" << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

  
  // Save the execution times
  times_saver();

  cout << "Hello Bear" << endl;

  ros::shutdown();

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{

    // Check the pthread id
    // pthread_t tid = pthread_self();
    // printf("Left image Thread ID: %lu\n", (unsigned long)tid);
  // End Check the pthread

  struct timespec start, end;
    
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();

  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);

  double time_spent = (end.tv_sec - start.tv_sec) * 1000000.0 +
                        (end.tv_nsec - start.tv_nsec) / 1000.0;
  
  double timestamp = img_msg->header.stamp.toSec();
  std::pair<double, double> curr_pair = std::make_pair(timestamp, time_spent);

  left_camera_exe_times.push_back(curr_pair);
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Check the pthread id
    // pthread_t tid = pthread_self();
    // printf("Right image Thread ID: %lu\n", (unsigned long)tid);
    // End Check the pthread

  struct timespec start, end; 
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();

  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
  double time_spent = (end.tv_sec - start.tv_sec) * 1000000.0 +
                        (end.tv_nsec - start.tv_nsec) / 1000.0;
  double timestamp = img_msg->header.stamp.toSec();
  std::pair<double, double> curr_pair = std::make_pair(timestamp, time_spent);
  right_camera_exe_times.push_back(curr_pair);
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{

  struct timespec start, end;
  double time_spent;

  const double maxTimeDiff = 0.01;
  while(1)
  {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
        continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(imLeft,imLeft);
        mClahe->apply(imRight,imRight);
      }

      // // End of Fusion in ms
      // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
      // time_spent = (end.tv_sec - start.tv_sec) * 1000.0 +
      //                   (end.tv_nsec - start.tv_nsec) / 1000000.0;
      // fusion_exe_times.push_back(time_spent);

// Tracking Latency
      struct timespec start, end;
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

      if(do_rectify)
      {
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
      }
      if (++image_count >= image_to_skip) {
        image_count = 0;
        mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
      } else {
#ifdef DEBUG_HARMONIC
        std::cout << "image frame skipped" << std::endl;
#endif
      }


      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
      time_spent = (end.tv_sec - start.tv_sec) * 1000.0 +
                        (end.tv_nsec - start.tv_nsec) / 1000000.0;

      std::pair<double, double> curr_pair = std::make_pair(tImLeft, time_spent);
      tracking_exe_times.push_back(curr_pair);

      // End of Tracking

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}


void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // struct timespec res;
    // if (clock_getres(CLOCK_THREAD_CPUTIME_ID, &res) == -1) {
    //     perror("clock_getres");
    //     return;
    // }
    // printf("Resolution: %ld seconds and %ld nanoseconds\n", res.tv_sec, res.tv_nsec);

    struct timespec start, end;
    
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  // Check the pthread id
    // pthread_t tid = pthread_self();
    // printf("Thread ID: %lu\n", (unsigned long)tid);
  // End Check the pthread

   mBufMutex.lock();
   imuBuf.push(imu_msg);
   mBufMutex.unlock();

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);

    double time_spent = (end.tv_sec - start.tv_sec) * 1000000000.0 +
                        (end.tv_nsec - start.tv_nsec);
    
    double timestamp = imu_msg->header.stamp.toSec();
    std::pair<double, double> curr_pair = std::make_pair(timestamp, time_spent);
    imu_exe_times.push_back(curr_pair);

    // printf("Thread CPU time used: %lf nanoseconds\n", time_spent);

// End of Imu Driver
  return;
}


