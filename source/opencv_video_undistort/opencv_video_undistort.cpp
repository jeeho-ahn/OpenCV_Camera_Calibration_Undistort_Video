/* Video Undistort Software using OpenCV
*  
*  This program reads predefined camera parameters from file, and undistort a video file using it
* 
*  Published for WalkWithMe Youtube Channel
*  Written by Ahn, Jeeho
*  2021.6.19
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
#include <thread>

#include "calibrationIO.h"
#include "undistortVideo.h"

int main(int argc, char** argv)
{
	std::cout << "\n\tVideo Undistort Program from WalkWithMe - using OpenCV\n" << std::endl;
	if (argc < 2)
	{
		std::cout << "Usage: Drag a Video File to the Program Icon" << std::endl;
		return -1;
	}

	std::vector<boost::filesystem::path> videoFilesPaths(argc-1);
	for (int n = 1; n < argc; n++)
	{
		std::string originalFile(argv[n]);
		boost::filesystem::path oFile(originalFile);
		//std::string orig_file = oFile.generic_path().string();
		videoFilesPaths[n - 1] = oFile;
	}

	std::string binFile = std::string(argv[0]);
	boost::filesystem::path p(binFile);
	std::string resultFolder = p.parent_path().generic_path().string() + "/Result";
	std::string calibFolder = p.parent_path().generic_path().string() + "/calib_data";

	auto calib_pair = calibIO::load(calibFolder + "/camera_intrinsics.txt");

	//may return 0 when not able to detect
	int num_of_threads = std::thread::hardware_concurrency();
	std::cout << "Number of Threads Available on this PC: " << num_of_threads << std::endl;

	std::cout << "Calibration Parameters Loaded" << std::endl;
	std::cout << *calib_pair.first << std::endl;
	std::cout << *calib_pair.second << std::endl;

	//std::cout << calib_pair.first << std::endl;
	//std::cout << calib_pair.second << std::endl;

	std::cout << "\n\tStart Undistorting " << argc-1 << " videos" << std::endl;

	for (size_t m = 0; m < videoFilesPaths.size(); m++)
	{
		auto filename = videoFilesPaths[m].filename().string();
		auto filename_sp = split(filename, ".");
		//string outFileName = filename_sp[0] + "_undistorted." + filename_sp[1];
		string outFileName = filename_sp[0] + "_undistorted." + "mp4";
		string outFilePath = resultFolder + "/" + outFileName;

		std::cout << "\nUndistorting " << filename << std::endl;

		//cv::Mat CamMat = calib_pair.first.clone();
		//cv::Mat DistC = calib_pair.second.clone();

		//std::cout << (int)calib_pair.first->data << std::endl;
		//delete(calib_pair.first->data);

		//calibIO::dummy(calibFolder + "/camera_intrinsics.txt");


		//std::cout << *calib_pair.first << std::endl;
		undistortVideo(videoFilesPaths[m].generic_path().string(), outFilePath, calib_pair.first, calib_pair.second, num_of_threads);		
	}

	delete(calib_pair.second->data);
	delete(calib_pair.first);
	delete(calib_pair.second);	

	return 0;
}
