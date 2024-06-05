#pragma once

#include <string>
#include <mutex>
#include <vector>
#include <list>
#include <map>
#include <fstream>

#include "types.h"

typedef struct Vector3;
typedef struct Matrix;

namespace areno {

    class Renderer;

    struct WorldConfig {
        int imu_freq;
        float imu_gyr_noise_density;
        float imu_gyr_random_walk;
        float imu_acc_noise_density;
        float imu_acc_random_walk;
        float time_offset;
        Matrix T_gnss_imu;
        std::string rtk_format;
        Matrix T_wg_wr;
        Vector3 P_rtk_gnss;
        bool gnss_raw;
        Matrix T_o1d_gnss;
        float o1d_scale_v;
        float o1d_bias_v;
        float o1d_scale_w;
        float o1d_bias_w;
        WorldConfig(const std::string& path);
    };

    class World {
        friend class Renderer;
    public:
        World(WorldConfig config) : _config(config) {
            _startTime = std::chrono::system_clock::now();
        }

        void generate();
        void dump();
        void record(double dt);
        void updateCar(double dt);
        void update();

        void lock() {
            _lock.lock();
        }

        void unlock() {
            _lock.unlock();
        }

    private:
        WorldConfig _config;
        std::mutex _lock;

        std::chrono::system_clock::time_point _startTime;
        double _time = 0.0;
        float _dt = 0.0;
        bool _first = true;

        std::list<PlanePtr> _planes;
        std::list<BlockPtr> _blocks;
        std::list<PointMassPtr> _pointMasses;
        std::list<PointMassLinkPtr> _links;

        std::list<PointMassPtr> _frontWheels;
        std::list<PointMassPtr> _backWheels;
        std::list<PointMassPtr> _car;
        std::vector<PointMassPtr> _framePMs;
        std::list<PointMassLinkPtr> _wheelLinks;
        float _gas = 0.0f;
        float _steer = 0.0f;
        Vector3 _carCenter = Vector3{0,0,0};
        Vector3 _frameCenter = Vector3{0,0,0};
        Matrix _carMatrix, _gnssMatrix, _gnssPrvPrvMatrix;
        Vector3 _IMUrelVel;
        PointMassPtr _rtkPM, _gnssPM, _imuPM, _o1dPM;
        std::vector<PointMassPtr> _sensors;
        bool _recording = false;
        std::string _recDir = "";
        double _recTime = 0;
        double _lastGNSStime = 0;
        double _lastRTKtime = 0;
        double _gnssError = 0;
        Vector3 _imuAcc = Vector3Zero();
        Vector3 _imuGyr = Vector3Zero();
        std::map<size_t, std::pair<Vector3, float>> _gnssPosesNcov;
        std::map<size_t, Matrix> _rtkPoses;

        PlanePtr _addPlane(const Plane& p);
        BlockPtr _addBlock(const Block& b);
        PointMassPtr _addPointMass(const PointMass& pm);
        PointMassLinkPtr _addLink(const PointMassLink& l);
        void _removeLink(const PointMassLinkPtr& l);
    };   
}