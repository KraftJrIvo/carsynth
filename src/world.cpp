#include "world.h"

#include <algorithm>
#include <raylib.h>
#include <raymath.h>

#include <chrono>
#include <memory>

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
    }
    for (auto& wl : _wheelLinks)
        wl->alignAxis = carup;

    _carCenter = Vector3Zero();
    for (auto& pm : _car)
        _carCenter += pm->pos;
    _carCenter = _carCenter / float(_car.size());
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


