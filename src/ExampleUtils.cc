#include "ExampleUtils.h"
#include <iostream>
#include <sstream> 
#include <fstream>
#include <iomanip>

using namespace std;

string lastItemFromPath(const string &strPath)
{
    size_t pos_slash = strPath.find_last_of("\\/");
    size_t pos_dot = strPath.find_last_of('.');
    if (std::string::npos == pos_slash)
        pos_slash = 0;
    else
        pos_slash++;
    if (std::string::npos == pos_dot)
        pos_dot = strPath.length();
    return strPath.substr(pos_slash, pos_dot - pos_slash);
}



string generateTrajectoryFilename(const string &map_path, SlamMode mode, string* suffix, size_t scan_max)
{
    ostringstream prefix_ss;
    prefix_ss << lastItemFromPath(map_path);
    if (suffix != nullptr)
    {
        prefix_ss << '_';
        prefix_ss << lastItemFromPath(*suffix);
    }
    prefix_ss << mode;
    string prefix= prefix_ss.str();

    ostringstream path_ss;

    for(size_t cnt = 0U; cnt < scan_max;cnt++)
    {        
        ostringstream oss;
        oss << prefix << std::setfill('0') << std::setw(2) <<  cnt;
        string filename = oss.str();
        if (!ifstream(filename + ".traject"))
        {
            return filename;
        }
    }
    return prefix;


}
/*
string buildTrajectFilename(const string &prefixPath,const string &suffixPath, int mode)
{
    ostringstream name_ss;

    name_ss << lastItemFromPath(prefixPath);
    if (!suffixPath.empty())
    {
        name_ss << '_';
        name_ss << lastItemFromPath(suffixPath);
    }
    if (mode == 0)
        name_ss << "slam";
    else if (mode == 1)
        name_ss << "vo";
    else if (mode == 2)
        name_ss << "local";
    else if (mode == 3)
        name_ss << "seq";
    //name_ss << ext;
    
    string prefix_filename = name_ss.str();
    for(int cnt = 0; cnt < 100;cnt++)
    {        
        ostringstream oss;
        oss << prefix_filename << std::setfill('0') << std::setw(2) <<  cnt << ext;
        string filename = oss.str();
        if (!ifstream(filename))
        {
            return filename;
        }
    }
    return prefix_filename;
}*/