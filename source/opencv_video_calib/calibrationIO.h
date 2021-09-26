/* Video Undistort Software using OpenCV
*
*  Header file for camera parameter file IO
*
*  Published for WalkWithMe Youtube Channel
*  Written by Ahn, Jeeho
*  2021.6.19
*/

#pragma once
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <memory>

using namespace std;

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 9)
{
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}

/// Split a string by a delimiter
std::vector<std::string> split(std::string s, std::string delimiter) {
	size_t pos_start = 0, pos_end, delim_len = delimiter.length();
	std::string token;
	std::vector<std::string> res;

	while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
		token = s.substr(pos_start, pos_end - pos_start);
		pos_start = pos_end + delim_len;
		res.push_back(token);
	}

	res.push_back(s.substr(pos_start));
	return res;
}

namespace calibIO
{
	void save(cv::Mat paramMat, cv::Mat distCoeff, string file)
	{
		auto fx = paramMat.at<double>(0, 0);
		auto fy = paramMat.at<double>(1, 1);
		auto px = paramMat.at<double>(0, 2);
		auto py = paramMat.at<double>(1, 2);

		vector<double> dist_coef = { distCoeff.at<double>(0,0),distCoeff.at<double>(0,1),distCoeff.at<double>(0,2),distCoeff.at<double>(0,3),distCoeff.at<double>(0,4) };

		ofstream outfile;
		outfile.open(file, ios::out | ios::trunc);
		
		outfile << "fx:" + to_string_with_precision(fx) << std::endl;
		outfile << "fy:" + to_string_with_precision(fy) << std::endl;
		outfile << "px:" + to_string_with_precision(px) << std::endl;
		outfile << "py:" + to_string_with_precision(py) << std::endl;
		outfile << "dist:" + to_string_with_precision(dist_coef[0]) + "," + to_string_with_precision(dist_coef[1]) 
			+ "," + to_string_with_precision(dist_coef[2]) + "," + to_string_with_precision(dist_coef[3]) + "," + to_string_with_precision(dist_coef[4]);
		
		outfile.close();
	}

	std::pair<cv::Mat*, cv::Mat*> load(string filename)
	{		
		double fx, fy, px, py;
		std::vector<double>* dist =  new std::vector<double>(0);

		std::ifstream file(filename);
		string line;
		if (file.is_open())
		{
			while (std::getline(file, line))
			{
				//std::cout << "Read line: " << line << '\n';
				auto line_sp = split(line, ":");
				if (line_sp[0] == "fx")
					fx = stod(line_sp[1]);
				else if (line_sp[0] == "fy")
					fy = stod(line_sp[1]);
				else if (line_sp[0] == "px")
					px = stod(line_sp[1]);
				else if (line_sp[0] == "py")
					py = stod(line_sp[1]);
				else if (line_sp[0] == "dist")
				{
					auto dist_sp = split(line_sp[1], ",");
					dist->resize(dist_sp.size());
					for (size_t n = 0; n < dist_sp.size(); n++)
						dist->at(n) = stod(dist_sp[n]);
				}
			}
			file.close();
		}

		//std::cout << "Camera Intrinsics Loaded:\n" << "fx: " << fx << " fx: " << fy << " px: " << px << " py: " << py << std::endl;

		//double intrin_arr[3][3] = { { fx, 0, px},{ 0, fy, py}, {0, 0, 1} };
		std::vector<double>* intrin_arr = new std::vector<double>{ fx, 0, px, 0, fy, py, 0, 0, 1 };
		//std::cout << (int)intrin_arr << std::endl;
		cv::Mat* intrinsics = new cv::Mat(3, 3, CV_64F, intrin_arr->data());
		cv::Mat* dist_coeff = new cv::Mat(1, dist->size(), CV_64F, dist->data());

		return std::make_pair(intrinsics, dist_coeff);
	}

	void dummy(string filename)
	{
		double fx, fy, px, py;
		std::vector<double>* dist = new std::vector<double>(0);

		std::ifstream file(filename);
		string line;
		if (file.is_open())
		{
			while (std::getline(file, line))
			{
				//std::cout << "Read line: " << line << '\n';
				auto line_sp = split(line, ":");
				if (line_sp[0] == "fx")
					fx = stod(line_sp[1]);
				else if (line_sp[0] == "fy")
					fy = stod(line_sp[1]);
				else if (line_sp[0] == "px")
					px = stod(line_sp[1]);
				else if (line_sp[0] == "py")
					py = stod(line_sp[1]);
				else if (line_sp[0] == "dist")
				{
					auto dist_sp = split(line_sp[1], ",");
					dist->resize(dist_sp.size());
					for (size_t n = 0; n < dist_sp.size(); n++)
						dist->at(n) = stod(dist_sp[n]);
				}
			}
			file.close();
		}
	}
}