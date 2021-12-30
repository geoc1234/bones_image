#include "myIO.hpp"

template <class myIO> 
std::vector<myIO> my_getfiles(std::string path, bool verbose)
{
    std::vector<myIO> file;
    return file;
}

// get all the files in a directory, where path is to the directory
std::vector<cv::Mat> my_getfiles(std::string path, bool verbose)
{
    std::vector<cv::String> fn; //list of file names to be found by cv::glob
    std::vector<cv::Mat> images;

    cv::glob(path, fn, false);
    
    try
    {
        for (auto filename : fn){
            if(verbose) std::cout << "filename: " << filename << std::endl;
            images.push_back(cv::imread(filename));
            if(verbose) std::cout << "No. Images: " << images.size() << std::endl;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return images;
}

// get the names of all subdirectories in the directory given by path
std::vector<const char*> my_getdirectories(std::string path, bool verbose)
{
    std::vector<const char*> names;
    DIR *dir = opendir(path.c_str());
    struct dirent *entry = readdir(dir);
    while (entry != NULL)
    {
        if (entry->d_type == DT_DIR)
            if(entry->d_name[0] != '.')
            {
                if(verbose) printf("%s\n", entry->d_name);
                names.push_back(entry->d_name);
            }
        entry = readdir(dir);
    }
    closedir(dir);
    return names;
}