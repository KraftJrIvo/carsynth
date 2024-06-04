#pragma once

#include <string>
#include <mutex>
#include <vector>
#include <list>
#include <fstream>

#include "raylib.h"
#include "raymath.h"
#include "types.h"

namespace areno {
    class Renderer;

    class World {
        friend class Renderer;
    public:
        World() { 
            _startTime = std::chrono::system_clock::now();
        }

        void generate();
        void updateCar(double dt);
        void update();

        void lock() {
            _lock.lock();
        }

        void unlock() {
            _lock.unlock();
        }

    private:
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
        Vector3 _carCenter = Vector3Zero();

        PlanePtr _addPlane(const Plane& p);
        BlockPtr _addBlock(const Block& b);
        PointMassPtr _addPointMass(const PointMass& pm);
        PointMassLinkPtr _addLink(const PointMassLink& l);
        void _removeLink(const PointMassLinkPtr& l);
    };   
}