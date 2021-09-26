/* Video Undistort Software using OpenCV
*
*  This program reads a video file contains a predefined checherboard pattern, then finds the camera parameters using OpenCV
*
*  Published for WalkWithMe Youtube Channel
*  Written by Ahn, Jeeho
*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <experimental/filesystem>
#include <boost/filesystem.hpp>
#include <memory>

#include "calibrationIO.h"
#include "undistortVideo.h"

#include <omp.h>
#include <time.h>
#include <thread>


using namespace cv;

typedef std::shared_ptr<cv::Mat> cvMatPtr;
typedef std::vector<std::string> stringvec;

//subfunction for reading a directory
struct path_leaf_string
{
	std::string operator()(const boost::filesystem::directory_entry& entry) const
	{
		return entry.path().leaf().string();
	}
};

//subfunction for sorting
bool cmp(std::string a, std::string b)
{
	int p = std::stoi(split(a, ".")[0]); //can use custom function or std::stoi
	int q = std::stoi(split(b, ".")[0]); // converting string to integer
	return p < q;
}

//read all files in a directory
void read_directory(const std::string& folder, stringvec& pcd_list)
{
	std::vector<std::string> files_list;
	boost::filesystem::path p(folder);
	boost::filesystem::directory_iterator start(p);
	boost::filesystem::directory_iterator end;
	std::transform(start, end, std::back_inserter(files_list), path_leaf_string());

	//take files only
	pcd_list.clear();
	for (std::vector<std::string>::iterator it = files_list.begin(); it != files_list.end(); ++it)
		if (split(*it, ".").size() == 2)
			pcd_list.push_back(*it);
	//else
	//	pcd_list.push_back(*it);

	//must be in order
	//sort(pcd_list.begin(), pcd_list.end(), cmp);

	//std::cout << "PCD Files in " << folder << std::endl;
	for (size_t n = 0; n < pcd_list.size(); n++)
		std::cout << pcd_list[n] << std::endl;
}

//read all image files in a directory
std::vector<cvMatPtr> read_all_images(std::string folder)
{
	stringvec files;
	read_directory(folder, files);

	std::vector<cvMatPtr> out_list;
	for (size_t i = 0; i < files.size(); i++)
	{
		//read pcd files only
		//auto file_ext_compared = files[i].substr((files[i].size()-ext.size()-1),ext.size());
		auto file_name_sp = split(files[i], ".");

		cv::Mat img = cv::imread(folder + "/" + files[i]);
		if (img.empty())
		{
			std::cout << "Could not read the image: " << files[i] << std::endl;
			out_list.push_back(nullptr);
			//return 1;
		}
		else
			out_list.push_back(std::make_shared<cv::Mat>(img));
	}

	return out_list;
}

//dereference cv mat list
void cvmatPtr2value(std::vector<cvMatPtr> ptrVec, std::vector<cv::Mat>& out_vec)
{
	out_vec.resize(ptrVec.size());

	for (size_t n = 0; n < ptrVec.size(); n++)
		out_vec[n] = *(ptrVec.at(n));
}

//find corner pixels in an image with checkerboard
std::pair<std::vector<cv::Point2f>, bool> find_corners(cv::Mat& ckb_img, cv::Size patternSize)
{
	Mat gray;
	cvtColor(ckb_img, gray, COLOR_BGR2GRAY);//source image
	std::vector<Point2f> corners;

	bool patternfound = findChessboardCorners(gray, patternSize, corners,
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		+ CALIB_CB_FAST_CHECK);

	if (patternfound)
		cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
			TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));
			//TermCriteria(cv::CV_TERMCRIT_EPS + cv::CV_TERMCRIT_ITER, 30, 0.1));

	return std::make_pair(corners, patternfound);
}

int main(int argc, char** argv)
{
	std::cout << "\n\tCamera Calibration Program from WalkWithMe - using OpenCV\n" << std::endl;


	std::string binFile = std::string(argv[0]);
	boost::filesystem::path p(binFile);
	std::string folder = p.parent_path().generic_path().string() + "/calib_data";

	//checkboard definition
	cv::Size patternSize(10, 7);

	//may return 0 when not able to detect
	int num_of_threads = std::thread::hardware_concurrency();
	std::cout << "Number of Threads Available on this PC: " << num_of_threads << std::endl;

	if (num_of_threads == 0)
		num_of_threads = 4;
	//display
	/*
	for (size_t n = 0; n < ckb_img_list.size(); n++)
	{
		cv::Mat temp_img;
		ckb_img_list[n].copyTo(temp_img);
		drawChessboardCorners(temp_img, patternSize, Mat(corners_list[n]), corners_bool_list[n]);

		imshow("result", temp_img);
		moveWindow("result", temp_img.cols / 2, 100);
		waitKey(0);
	}
	*/

	cv::Mat cameraMatrix, distCoeffs, R, T;

	/*
   * Performing camera calibration by
   * passing the value of known 3D points (objpoints)
   * and corresponding pixel coordinates of the
   * detected corners (imgpoints)
  */

	std::string videoFile;

	if (argc == 1)
	{
		std::vector<std::string> files;
		read_directory(folder + "/original_video", files);

		if (files.size() == 0)
		{
			std::cout << "No file Found in " << folder + "/original_video" << std::endl;
			return -1;
		}
		else if (files.size() > 1)
		{
			std::cout << "More than one file found in " << folder + "/original_video" << std::endl;
			std::cout << "Using the first file in namely order" << std::endl;
		}

		std::cout << "using " << files[0] << std::endl;
		videoFile = folder + "/original_video/" + files[0];
	}
	else if (argc == 2)
	{
		std::cout << "Dragged a Video File to the Program Icon" << std::endl;
		std::string originalFile(argv[1]);
		boost::filesystem::path oFile(originalFile);
		std::string orig_file = oFile.generic_path().string();
		auto filename = oFile.filename().string();
		std::cout << "using " << filename << std::endl;
		videoFile = orig_file;
	}
	
	cv::VideoCapture vid(videoFile);
	if (!vid.isOpened())
		throw "Error when reading test video";

	auto frame_width = vid.get(CAP_PROP_FRAME_WIDTH);
	auto frame_height = vid.get(CAP_PROP_FRAME_HEIGHT);
	auto frame_bitrate = vid.get(CAP_PROP_BITRATE);

	auto total_frames = vid.get(CAP_PROP_FRAME_COUNT);
	//find targeting framerate
	auto frame_rate = vid.get(CAP_PROP_FPS);
	if (frame_rate > 29 && frame_rate < 30)
		frame_rate = 30;
	else if (frame_rate > 59 && frame_rate < 60)
		frame_rate = 60;
	//VideoWriter out_video;
	//out_video.open("appsrc ! videoconvert ! avenc_mpeg4 bitrate=94699 ! mp4mux ! filesink location=video.mp4", CAP_GSTREAMER, 0, frame_rate, Size(vid.get(cv::CAP_PROP_FRAME_WIDTH), vid.get(cv::CAP_PROP_FRAME_HEIGHT)), true);

	//namedWindow("Display frame", WINDOW_NORMAL);
	//cv::resizeWindow("Display frame", 1920, 1080);

	std::vector<std::vector<cv::Point2f>> corners_list2;
	std::vector<bool> corners_bool_list2;
	std::vector<std::shared_ptr<cv::Mat>> pickedFrames;

	int fcount = 0;
	
	clock_t start, end;
	start = clock();
	const int step = int(frame_rate / 2);
	for (int i=0; i<int(total_frames / step); i+= num_of_threads)
	{		
		std::vector<shared_ptr<cv::Mat>> temp_img_list(0);
		
		for (int n = 0; n < num_of_threads; n++)
		{
			cv::Mat tempF;
			for (int o = 0; o < step; o++)
				vid >> tempF;
			if (tempF.empty())
				break;
			temp_img_list.push_back(std::make_shared<cv::Mat>(tempF));
		}

		//vid >> tempF;

		//if (tempF.empty())
		//	break;
		//imshow("Display frame", tempF);
		//int pressed = waitKey(frame_rate); // waits to display frame
		//if (pressed > 0)
		//{
			//pickedFrames.push_back(std::make_shared<cv::Mat>(tempF));
		//	cv::Mat cpimg;
		//	tempF.copyTo(cpimg);
		//	pickedFrames.push_back(std::make_shared<cv::Mat>(cpimg));
		//}

		std::vector<std::vector<cv::Point2f>> temp_corners_list2(temp_img_list.size());
		std::vector<bool> temp_corners_bool_list2(temp_img_list.size());
		

			#pragma omp parallel for		
			for (int p = 0; p < temp_img_list.size(); p++)
			{
				
				auto find_corners_res = find_corners(*(temp_img_list[p]), patternSize);


				if (find_corners_res.second == true)
				{
					//corners_list2.push_back(find_corners_res.first);
					//corners_bool_list2.push_back(find_corners_res.second);
					temp_corners_list2[p] = find_corners_res.first;
					temp_corners_bool_list2[p] = find_corners_res.second;
					//cv::Mat cpimg;
					//tempF.copyTo(cpimg);
					//pickedFrames.push_back(std::make_shared<cv::Mat>(cpimg));
				}
				else
				{
					temp_corners_bool_list2[p] = find_corners_res.second;
				}
				
			}
		

		//copy temp lists to main list
		for (int y = 0; y < temp_corners_list2.size(); y++)
		{
			std::cout << "Finding Pattern " << fcount + 1 << " out of " << int(total_frames / step);
			if (temp_corners_bool_list2[y] == true)
			{
				std::cout << " --- OK" << std::endl;
				corners_list2.push_back(temp_corners_list2[y]);
			}
			else
				std::cout << " --- Failed" << std::endl;

			fcount++;
		}
		
		/*
		if (find_corners_res.second == true)
		{
			std::cout << "Frame " << fcount << " OK" << std::endl;
			fcount++;
			cv::Mat cpimg;
			tempF.copyTo(cpimg);
			pickedFrames.push_back(std::make_shared<cv::Mat>(cpimg));
		}
		else
		{
			std::cout << "Frame " << fcount << " Failed" << std::endl;
		}
		*/
	}
	end = clock();
	printf("Time Elapsed : %.2f seconds \n", ((float)(end - start) / CLOCKS_PER_SEC));

	//cvDestroyWindow("Display frame");


	//if (pickedFrames.size() == 0)
	//	return -1;
	if (corners_list2.size() == 0)
		return -1;

	//namedWindow("Patterns", WINDOW_NORMAL);
	//cv::resizeWindow("Patterns", 1920, 1080);

	//checkboard definition
	//cv::Size patternSize(10, 7);

	// Defining the world coordinates for 3D points

	std::vector<cv::Point3f> objp2;
	for (int i{ 0 }; i < 7; i++)
	{
		for (int j{ 0 }; j < 10; j++)
			objp2.push_back(cv::Point3f(i, j, 0));
	}

	std::vector<std::vector<cv::Point3f>> objp_vec2(corners_list2.size());
	for (size_t n = 0; n < corners_list2.size(); n++)
		objp_vec2[n] = objp2;


	//for (size_t n = 0; n < corners_list2.size(); n++)
	//{
		//if(find_corners_res.second == true)
		//	drawChessboardCorners(*(pickedFrames[n]), patternSize, Mat(corners_list2[n]), corners_bool_list2[n]);

		//imshow("Patterns", *(pickedFrames[n]));
		//waitKey(0);
	//}
	//cvDestroyWindow("Patterns");

	/*
	for (size_t n = 0; n < corners_bool_list2.size(); n++)
		if (corners_bool_list2[n] == false)
		{
			std::cout << "Failed to find checkboard corners at " << n << std::endl;
			//remove from list
			//pickedFrames.erase(pickedFrames.begin() + n);
			//corners_list2.erase(corners_list2.begin() + n);
			//objp_vec2.erase(objp_vec2.begin() + n);
		}
	*/
	cv::Mat cameraMatrix2, distCoeffs2, R2, T2;

	/*
   * Performing camera calibration by
   * passing the value of known 3D points (objpoints)
   * and corresponding pixel coordinates of the
   * detected corners (imgpoints)
  */
	std::cout << "Camera Calibration in Progress" << std::endl;
	cv::calibrateCamera(objp_vec2, corners_list2, cv::Size(frame_height, frame_width), cameraMatrix2, distCoeffs2, R2, T2);

	std::cout << "cameraMatrix : " << cameraMatrix2 << std::endl;
	std::cout << "distCoeffs : " << distCoeffs2 << std::endl;
	//std::cout << "Rotation vector : " << R2 << std::endl;
	//std::cout << "Translation vector : " << T2 << std::endl;

	//save to file
	calibIO::save(cameraMatrix2, distCoeffs2,folder + "/camera_intrinsics.txt");


	string originalVideoFilePath = videoFile;
	boost::filesystem::path oFilePath(originalVideoFilePath);	
	auto originalVideoFile = oFilePath.filename().string();
	auto file_sp = split(originalVideoFile, ".");
	//string resultingVideoFilePath = folder + "/result_video/" + file_sp[0] + "_undistorted." + file_sp[1];
	string resultingVideoFilePath = folder + "/result_video/" + file_sp[0] + "_undistorted." + "mp4";

	std::cout << "Generating Undistorted Video" << std::endl;
	undistortVideo(originalVideoFilePath, resultingVideoFilePath, &cameraMatrix2, &distCoeffs2, num_of_threads);
	

	namedWindow("Undistortion Preview", WINDOW_NORMAL);
	cv::resizeWindow("Undistortion Preview", 1920, 1080);
	VideoCapture result_v(resultingVideoFilePath);
	double play_interval = (1.0 / frame_rate)*1000; // ms
	while (1)
	{
		clock_t ss = clock();
		cv::Mat frame_p;
		result_v >> frame_p;
		if (frame_p.empty())
			break;
		cv::imshow("Undistortion Preview", frame_p);
		clock_t st = clock();
		double time_op = ((double)(st - ss));		

		int play_ms = play_interval - time_op;
		if (play_ms <= 0)
			play_ms = 1;
		int key = cv::waitKey(play_ms);
		if (key > 0)
			break;
	}

	cv::destroyWindow("Undistortion Preview");

	return 0;
}

