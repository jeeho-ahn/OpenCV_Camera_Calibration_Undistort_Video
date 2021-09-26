/* Video Undistort Software using OpenCV
*
*  This program reads predefined camera parameters from file, and undistort a video file using it
*
*  Published for WalkWithMe Youtube Channel
*  Written by Ahn, Jeeho
*  2021.6.19
*/

#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include <opencv2/videoio/videoio.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int undistortVideo(string original_file, string result_file, cv::Mat* cameraMatrix, cv::Mat* distCoeffs, int num_of_threads = 8)
{
	//read video file
	VideoCapture vid = VideoCapture(original_file);
	//video info
	auto frame_width = vid.get(CAP_PROP_FRAME_WIDTH);
	auto frame_height = vid.get(CAP_PROP_FRAME_HEIGHT);
	auto bit_rate = vid.get(CAP_PROP_BITRATE);
	auto total_frames = vid.get(CAP_PROP_FRAME_COUNT);
	auto frame_rate = vid.get(CAP_PROP_FPS);
	
	//video to be written out
	//set to be mp4 format
	VideoWriter out_video(result_file, cv::VideoWriter::fourcc('m','p','4','v'), frame_rate, Size(vid.get(cv::CAP_PROP_FRAME_WIDTH), vid.get(cv::CAP_PROP_FRAME_HEIGHT)));

	//frame count
	int count = 1;
	//undistort each frame in loop
	while(count < total_frames)
	{
		std::vector<shared_ptr<cv::Mat>> temp_frames(0);

		//stack frames in a list for multithreading
		for (int a = 0; a < num_of_threads; a++)
		{
			cv::Mat frame;
			vid >> frame;
			if (frame.empty())
				break;
			temp_frames.push_back(std::make_shared<cv::Mat>(frame));
		}

		//imshow("d",frame);
		//waitKey(0);

		//output list
		std::vector<shared_ptr<cv::Mat>> temp_undist_imgs(temp_frames.size());

		//OpenMP for multithreading
		#pragma omp parallel for
		for (int l = 0; l < temp_frames.size(); l++)
		{
			cv::Mat undist_img_;
			cv::undistort(*temp_frames[l], undist_img_, *cameraMatrix, *distCoeffs);
			temp_undist_imgs[l] = make_shared<cv::Mat>(undist_img_);
			//imshow("w", *temp_frames[l]);
			//waitKey(0);
		}

		//imshow("w", frame);
		//waitKey(1); // waits to display frame

		//write all undistorted frames to a video file
		for (int u = 0; u < temp_undist_imgs.size(); u++)
		{
			out_video.write(*temp_undist_imgs[u]);
			//imshow("w", *temp_undist_imgs[u]);
			//waitKey(0);
			std::cout << "\r" << count << " frames out of " << total_frames << " frames" << std::flush;
			count++;
		}

	}
	//end = clock();
	//printf("\nTime Elapsed : %.2f seconds \n", ((float)(end - start) / CLOCKS_PER_SEC));
	//waitKey(0); // key press to close window
	out_video.release();

	return 0;
}