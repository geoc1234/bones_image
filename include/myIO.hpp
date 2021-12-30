#ifndef MY_IO
#define MY_IO

#include <iostream>
#include <vector>
#include <dirent.h>
#include <sys/stat.h>
#include <opencv2/core/utility.hpp> // for glob function
#include <opencv2/highgui.hpp>


// return the files in a given directory as a std vector.  Assumes all files are
// the same format. 
template <class myIO> 
std::vector<myIO> my_getfiles(std::string path);

// return the images in a given directory as a std vector.  Assumes all files are
// the same format.
std::vector<cv::Mat> my_getfiles(std::string path, bool verbose);


// get the names of all subdirectories in the directory given by path
std::vector<const char*> my_getdirectories(std::string path, bool verbose);

#endif