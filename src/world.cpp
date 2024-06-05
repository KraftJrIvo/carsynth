#include "world.h"
#include "util.h"
#include "vec_funcs.h"

#include <raylib.h>
#include <raymath.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>

#define SUB_STEPS 4

using namespace areno;

void World::generate() {
    _addPlane(Plane({0,0,0}, {0, 1.0f, 0}, 0.99f, GRAY));
    _addPlane(Plane({0, -3.0f, 0}, Vector3RotateByAxisAngle({0, 1.0f, 0}, {0, 0, 1.0f}, 0.3), 0.99f, RED));
    _addPlane(Plane({0, -3.0f, 0}, Vector3RotateByAxisAngle({0, 1.0f, 0}, {0, 0, 1.0f}, -0.3), 0.99f, BLUE));
    _addPlane(Plane({0, -3.0f, 0}, Vector3RotateByAxisAngle({0, 1.0f, 0}, {1.0f, 0, 0}, 0.3), 0.99f, MAGENTA));
    _addPlane(Plane({0, -10.0f, 0}, Vector3RotateByAxisAngle({0, 1.0f, 0}, {1.0f, 0, 0}, -0.3), 0.99f, YELLOW));
    float CAR_LEN = 4.0f;
    float CAR_WID = 2.5f;
    float CAR_SIZ = 0.2f;
    float WHL_HGH = 0.25f;
    auto p1 = _addPointMass(PointMass({0, 1, 0}, 0.05f, 0.1f, 0.5f, BROWN, false, true));
    auto p2 = _addPointMass(PointMass({CAR_SIZ * CAR_WID, 1, 0}, 0.05f, 0.1f, 0.5f, BROWN, false, true));
    auto p3 = _addPointMass(PointMass({0, 1, CAR_SIZ * CAR_LEN}, 0.05f, 0.1f, 0.5f, BROWN, false, true));
    auto p4 = _addPointMass(PointMass({CAR_SIZ * CAR_WID, 1, CAR_SIZ * CAR_LEN}, 0.05f, 0.1f, 0.5f, BROWN, false, true));
    //p1->mass *= 10.0;
    //p2->mass *= 10.0;
    //p3->mass *= 10.0;
    //p4->mass *= 10.0;
    _addLink(PointMassLink(p1, p2, true, CAR_SIZ * CAR_WID, 1.0f));
    _addLink(PointMassLink(p3, p4, true, CAR_SIZ * CAR_WID, 1.0f));
    _addLink(PointMassLink(p1, p3, true, CAR_SIZ * CAR_LEN, 1.0f));
    _addLink(PointMassLink(p2, p4, true, CAR_SIZ * CAR_LEN, 1.0f));
    _addLink(PointMassLink(p1, p4, false, CAR_SIZ * sqrt(CAR_WID * CAR_WID + CAR_LEN * CAR_LEN), 1.0f));
    _addLink(PointMassLink(p2, p3, false, CAR_SIZ * sqrt(CAR_WID * CAR_WID + CAR_LEN * CAR_LEN), 1.0f));
    auto w1 = _addPointMass(PointMass({0, 1 - WHL_HGH, 0}, 0.1f, 0.01f, 0.5f, RED, false, true));
    auto w2 = _addPointMass(PointMass({CAR_SIZ * CAR_WID, 1 - WHL_HGH, 0}, 0.1f, 0.01f, 0.5f, RED, false, true));
    auto w3 = _addPointMass(PointMass({0, 1 - WHL_HGH, CAR_SIZ * CAR_LEN}, 0.1f, 0.01f, 0.5f, BLACK, false, true));
    auto w4 = _addPointMass(PointMass({CAR_SIZ * CAR_WID, 1 - WHL_HGH, CAR_SIZ * CAR_LEN}, 0.1f, 0.01f, 0.5f, BLACK, false, true));
    auto wl1 = _addLink(PointMassLink(p1, w1, false, WHL_HGH, 0.03f, 0.01, 5.0, {0, 1.0f, 0}));
    auto wl2 = _addLink(PointMassLink(p2, w2, false, WHL_HGH, 0.03f, 0.01, 5.0, {0, 1.0f, 0}));
    auto wl3 = _addLink(PointMassLink(p3, w3, false, WHL_HGH, 0.03f, 0.01, 5.0, {0, 1.0f, 0}));
    auto wl4 = _addLink(PointMassLink(p4, w4, false, WHL_HGH, 0.03f, 0.01, 5.0, {0, 1.0f, 0}));

    _frontWheels.push_back(w1);
    _frontWheels.push_back(w2);
    _backWheels.push_back(w3);
    _backWheels.push_back(w4);
    _framePMs.push_back(p1);
    _framePMs.push_back(p2);
    _framePMs.push_back(p3);
    _framePMs.push_back(p4);
    _wheelLinks.push_back(wl1);
    _wheelLinks.push_back(wl2);
    _wheelLinks.push_back(wl3);
    _wheelLinks.push_back(wl4);
    _car.push_back(w1);
    _car.push_back(w2);
    _car.push_back(w3);
    _car.push_back(w4);
    _car.push_back(p1);
    _car.push_back(p2);
    _car.push_back(p3);
    _car.push_back(p4);

    _rtkPM = _addPointMass(PointMass({ 0, 0, 0 }, 0.01f, 0, 0, RED, true, false));
    _gnssPM = _addPointMass(PointMass({ 0, 0, 0 }, 0.01f, 0, 0, YELLOW, true, false));
    _imuPM = _addPointMass(PointMass({ 0, 0, 0 }, 0.01f, 0, 0, GREEN, true, false));
    _o1dPM = _addPointMass(PointMass({ 0, 0, 0 }, 0.01f, 0, 0, SKYBLUE, true, false));
}

void World::dump() {
    _recTime = 0;
    _lastGNSStime = 0;
    _lastRTKtime = 0;
    _recDir = "";
}

void World::record(double dt) {
    double RTK_FREQ = 0.0025;
    double GNSS_FREQ = 0.2;
    double GNSS_MIN_ERR = 0.02;
    double GNSS_MAX_ERR = 2.0;

    _recTime += dt;
    size_t ts = 1714735849753723144 + _recTime * 1e9;

    if (_recTime - _lastRTKtime > RTK_FREQ) {
        _rtkPoses[ts] = _carMatrix;
        _lastRTKtime = _recTime;
    }
    if (_recTime - _lastGNSStime > GNSS_FREQ) {
        _gnssError += (rand() % 3 < 2) ? -0.01f : 0.01f;
        _gnssError = std::clamp(_gnssError, GNSS_MIN_ERR, GNSS_MAX_ERR);
        auto error = _gnssError * Vector3{ util::randFloat() * 2.0f - 1.0f, util::randFloat() * 2.0f - 1.0f, util::randFloat() * 2.0f - 1.0f };
        _gnssPosesNcov[ts] = { _gnssPM->pos + error, float(_gnssError * _gnssError)};
        _lastGNSStime = _recTime;
    }
}

void World::updateCar(double dt) {
    Vector3 fwd = Vector3Normalize(_frontWheels.front()->pos - _backWheels.front()->pos);
    Vector3 rgt = Vector3Normalize((*std::next(_frontWheels.begin()))->pos - _frontWheels.front()->pos);
    auto dsteer = IsKeyDown(KEY_RIGHT) - IsKeyDown(KEY_LEFT);
    auto dgas = IsKeyDown(KEY_UP) - IsKeyDown(KEY_DOWN);
    if (dgas == 0)
        _gas -= _gas / 10.0f;
    else
        _gas += dgas * dt * 0.1f;
    _gas = std::clamp(_gas, -1.0f, 1.0f);
    if (dsteer == 0)
        _steer -= _steer / 10.0f;
    else
        _steer += dsteer * dt;
    _steer = std::clamp(_steer, -1.0f, 1.0f);
    bool collided = false;
    for (auto& w : _frontWheels) {
        collided |= w->collided;
        w->collided = false;
    }
    Plane ppp(_framePMs[0]->pos, _framePMs[1]->pos, _framePMs[2]->pos);
    auto carup = ppp.normal;
    if (collided) {
        auto acc = Vector3RotateByAxisAngle(fwd, carup, _steer * PI/4);
        for (auto& pm : _frontWheels) {
            pm->applyForce(acc, _gas * 100.0f);
            pm->applyForce(Vector3Normalize(Vector3RotateByAxisAngle(rgt, carup, _steer * PI/4)), -Vector3DotProduct(pm->vel, rgt) * 10.0f);
        }
        for (auto& pm : _backWheels) {
            pm->applyForce(Vector3Normalize(Vector3RotateByAxisAngle(rgt, carup, _steer * PI/4)), -Vector3DotProduct(pm->vel, rgt) * 10.0f);
        }
        for (auto& pm : _sensors) {
            pm->applyForce(acc, _gas * 100.0f);
            pm->applyForce(Vector3Normalize(Vector3RotateByAxisAngle(rgt, carup, _steer * PI / 4)), -Vector3DotProduct(pm->vel, rgt) * 10.0f);
        }
    }
    for (auto& wl : _wheelLinks)
        wl->alignAxis = carup;

    _carCenter = Vector3Zero();
    for (auto& pm : _car)
        _carCenter += pm->pos;
    _carCenter = _carCenter / float(_car.size());
    _frameCenter = Vector3Zero();
    for (auto& pm : _framePMs)
        _frameCenter += pm->pos;
    _frameCenter = _frameCenter / float(_framePMs.size());

    if (_sensors.size() == 0) {
        _sensors = { _rtkPM, _gnssPM, _imuPM, _o1dPM };
    }

    Matrix newCarMatrix, newGNSSmatrix;

    newCarMatrix.m0 = fwd.x; newCarMatrix.m4 = carup.x; newCarMatrix.m8  = rgt.x; newCarMatrix.m12 = _frameCenter.x;
    newCarMatrix.m1 = fwd.y; newCarMatrix.m5 = carup.y; newCarMatrix.m9  = rgt.y; newCarMatrix.m13 = _frameCenter.y;
    newCarMatrix.m2 = fwd.z; newCarMatrix.m6 = carup.z; newCarMatrix.m10 = rgt.z; newCarMatrix.m14 = _frameCenter.z;
    newCarMatrix.m3 =     0; newCarMatrix.m7 =       0; newCarMatrix.m11 =     0; newCarMatrix.m15 =           1.0f;

    newGNSSmatrix.m0 = fwd.x; newGNSSmatrix.m4 = carup.x; newGNSSmatrix.m8 =  rgt.x; newGNSSmatrix.m12 = _gnssPM->pos.x;
    newGNSSmatrix.m1 = fwd.y; newGNSSmatrix.m5 = carup.y; newGNSSmatrix.m9 =  rgt.y; newGNSSmatrix.m13 = _gnssPM->pos.y;
    newGNSSmatrix.m2 = fwd.z; newGNSSmatrix.m6 = carup.z; newGNSSmatrix.m10 = rgt.z; newGNSSmatrix.m14 = _gnssPM->pos.z;
    newGNSSmatrix.m3 =     0; newGNSSmatrix.m7 =       0; newGNSSmatrix.m11 =     0; newGNSSmatrix.m15 =           1.0f;

    auto IMUmat = MatrixMultiply(MatrixInvert(_config.T_gnss_imu), newGNSSmatrix);
    auto prvprvIMUmat = MatrixMultiply(MatrixInvert(_config.T_gnss_imu), _gnssPrvPrvMatrix);
    auto prvIMUmat = MatrixMultiply(MatrixInvert(_config.T_gnss_imu), _gnssMatrix);
    auto relIMUmat = MatrixMultiply(MatrixInvert(prvIMUmat), IMUmat);
    auto prvRotMat = prvIMUmat; prvRotMat.m12 = 0; prvRotMat.m13 = 0; prvRotMat.m14 = 0;
    auto prvprvRotMat = prvprvIMUmat; prvprvRotMat.m12 = 0; prvprvRotMat.m13 = 0; prvprvRotMat.m14 = 0;
    auto prvIMUrelVel = Vector3Transform(_imuPM->prvvel, MatrixInvert(prvprvRotMat));
    auto curIMUrelVel = Vector3Transform(_imuPM->vel, MatrixInvert(prvRotMat));
    _imuAcc = Vector3Transform(_imuPM->prvacc, MatrixInvert(prvRotMat));
    auto relAA = Eigen::AngleAxisd(toEigen(QuaternionFromMatrix(relIMUmat)));
    _imuGyr = fromEigen(relAA.axis() * relAA.angle()) / dt;
    _IMUrelVel = curIMUrelVel;

    
    _carMatrix = newCarMatrix;
    _gnssPrvPrvMatrix = _gnssMatrix;
    _gnssMatrix = newGNSSmatrix;

    _rtkPM->pos = _frameCenter;
    _gnssPM->pos = Vector3Transform(_config.P_rtk_gnss, _carMatrix);

    _imuPM->pos = Vector3{ IMUmat.m12, IMUmat.m13, IMUmat.m14 };
    auto o1dmat = MatrixMultiply(_config.T_o1d_gnss, _gnssMatrix);
    _o1dPM->pos = Vector3{ o1dmat.m12, o1dmat.m13, o1dmat.m14 };

    if (_recording)
        record(dt);
}

void World::update() {
    std::chrono::duration<double> seconds = std::chrono::system_clock::now() - _startTime;
    float dt = (seconds.count() - _time);
    _dt = dt;
    float subdt = dt / SUB_STEPS;
    //float subdt2 = subdt * subdt;
    _time = seconds.count();

    lock();

    for (int s = 0; s < SUB_STEPS; ++s) {
        for (auto it = _pointMasses.begin(); it != _pointMasses.end(); ++it)
            (*it)->applyForce({ 0, -1.0f, 0 }, 10.0f);

        updateCar(subdt);
        
        for (auto& l : _links) {
            l->constrain(subdt);
            if (l->collideable)
                for (auto b : _blocks)
                    l->collide(b);
        }
        for (auto it = _pointMasses.begin(); it != _pointMasses.end(); ++it) {
            auto& p1 = *it;
            for (auto p : _planes)
                p1->collide(p);
            for (auto b : _blocks)
                p1->collide(b);
            for (auto l : _links)
                if (l->collideable)
                    l->collide(p1);
            for (auto jt = std::next(it); jt != _pointMasses.end(); ++jt) {
                auto& p2 = *jt;
                p1->collide(p2);
            }
            p1->update(subdt);
        }
    } 

    unlock();
}

PlanePtr World::_addPlane(const Plane& p) {
    _planes.push_back(std::make_shared<Plane>(p));
    return _planes.back();
}

BlockPtr World::_addBlock(const Block& p) {
    _blocks.push_back(std::make_shared<Block>(p));
    return _blocks.back();
}

PointMassPtr World::_addPointMass(const PointMass& pm) {
    _pointMasses.push_back(std::make_shared<PointMass>(pm));
    return _pointMasses.back();
}

PointMassLinkPtr World::_addLink(const PointMassLink& lnk) {
    _links.push_back(std::make_shared<PointMassLink>(lnk));
    return _links.back();
}

void World::_removeLink(const PointMassLinkPtr& l) {
    auto it = _links.begin();
    while (true) {
        if (*it == l) it = _links.erase(it);
        if (it == _links.end()) break;
        it++;
    }
}


