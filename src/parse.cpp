#include "world.h"

#include <sstream>

#define RYML_SINGLE_HDR_DEFINE_NOW
#include "ryml.hpp"

std::string get_file_contents(std::string filename)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    if (!in) {
        std::cerr << "could not open " << filename << std::endl;
        exit(1);
    }
    std::ostringstream contents;
    contents << in.rdbuf();
    return contents.str();
}

areno::WorldConfig::WorldConfig(const std::string& path) {
    auto cont = get_file_contents(path);
    ryml::Tree tree = ryml::parse_in_place(ryml::to_substr((char*)cont.c_str()));
    ryml::NodeRef P_rtk_gnss_ = tree["gnss"]["P_rtk_gnss"];
    for (int i = 0; i < 3; ++i)
        P_rtk_gnss_[i] >> at(P_rtk_gnss, i);
    ryml::NodeRef T_enu_rtk_ = tree["rtk"]["T_wg_wr"];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_enu_rtk_[i][j] >> at(T_wg_wr, i, j);
    ryml::NodeRef T_o1d_gnss_ = tree["o1d"]["T_o1d_gnss"];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_o1d_gnss_[i][j] >> at(T_o1d_gnss, i, j);
    ryml::NodeRef rtk_format_name = tree["rtk"]["format"];
    rtk_format_name >> rtk_format;
    ryml::NodeRef imu_yml = tree["imu"];
    tree["imu"]["frequency"] >> imu_freq;
    imu_yml["gyr_noise_density"] >> imu_gyr_noise_density;
    imu_yml["gyr_random_walk"] >> imu_gyr_random_walk;
    imu_yml["acc_noise_density"] >> imu_acc_noise_density;
    imu_yml["acc_random_walk"] >> imu_acc_random_walk;
    //_propsIMU.gyroscopeNoiseDensity *= sqrt(_propsIMU.gyroscopeFrequency);
    //_propsIMU.gyroscopeRandomWalk /= sqrt(_propsIMU.gyroscopeFrequency);
    //_propsIMU.accelerometerNoiseDensity *= sqrt(_propsIMU.accelerometerFrequency);
    //_propsIMU.accelerometerRandomWalk /= sqrt(_propsIMU.accelerometerFrequency);
    imu_yml["time_offset"] >> time_offset;
    ryml::NodeRef T_gnss_imu_ = imu_yml["T_gnss_imu"];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_gnss_imu_[i][j] >> at(T_gnss_imu, i, j);
    tree["gnss"]["raw"] >> gnss_raw;
    tree["o1d"]["scale_v"] >> o1d_scale_v;
    tree["o1d"]["scale_w"] >> o1d_scale_w;
    tree["o1d"]["bias_v"] >> o1d_bias_v;
    tree["o1d"]["bias_w"] >> o1d_bias_w;
}