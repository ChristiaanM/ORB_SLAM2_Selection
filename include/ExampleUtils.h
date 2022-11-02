
#ifndef EXAMPLEUTILS
#define EXAMPLEUTILS

#include <string>
#include <iostream>

enum SlamMode
{
    
    bench = 0,
    slam = 1,
    vo = 2,
    local = 3,
    sequential = 4,
    cull = 5,
    bench10 = 10,
    bench20 = 20,
    bench30 = 30,
    bench40 = 40,
    slam10 = 11,
    slam20 = 21,
    slam30 = 32,
    slam40 = 41

};

inline  std::ostream& operator<<(std::ostream& str, SlamMode const& mode)
{
    switch (mode)
    {
        case bench:
            str << "bench";
            break;
        case slam:
            str << "slam";
            break;
        case vo:
            str << "vo";
            break;
        case local:
            str << "local";
            break;
        case cull:
            str << "cull";
        default:
            break;
    }
    return str;
}



void parseMode(const std::string &arg, SlamMode& mode, size_t& ba_iters, bool verbose = true)
{
    mode = (SlamMode) stoi(arg);
    if (mode < 10)
    {
        if (mode == bench)
            ba_iters = 40;
        else 
            ba_iters = 10;
    }
    else 
    {
        ba_iters = (mode / 10) * 10;
        mode = (SlamMode) ( mode % 10);
    }

    if (verbose)
    {
        std::cout << "Mode:  " << mode << std::endl;
        std::cout << "Iters: " << ba_iters << std::endl;
    }
}


std::string filnameFromPath(const std::string &strPath);
//std::string buildTrajectFilename(const std::string &prefixPath,const std::string &suffixPath, int mode, std::string ext = ".traject");
std::string generateTrajectoryFilename(const std::string &map_path, SlamMode mode, std::string* suffix = nullptr , size_t scan_max = 100U);

#endif // EXAMPLEUTILS
