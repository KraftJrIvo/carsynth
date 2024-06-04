#include "types.h"
#include "raylib.h"
#include "raymath.h"
#include <cmath>

#include "vec_funcs.h"
#include "util.h"

using namespace areno;

Plane::Plane(const Vector3& pos, const Vector3& normal, float friction, Color color) : 
    pos(pos), normal(normal), friction(friction), color(color) 
{ }

Plane::Plane(const Vector3& p1, const Vector3& p2, const Vector3& p3, float friction, Color color) :
    pos(p1),
    normal(Vector3Normalize(Vector3CrossProduct((p2 - p1),(p3 - p1)))),
    friction(friction),
    color(color)
{ }

Vector3 Plane::project(const Vector3& p) {
    return Vector3ProjectOntoPlane(pos, normal, p);
}

Block::Block(const Vector3& pos, const Vector3& size, float friction, Color color) :
    pos(pos), size(size), friction(friction), color(color)
{ }

bool Block::contains(const Vector3& v) {
    return Vector3InsideAABlock(pos, size, v);
}

void PointMass::applyForce(Vector3 dir, float strength) {
    if (!fixed)
	    acc += dir * strength;    
}

void PointMass::update(double dt) {
    if (fixed) return;
    mov = pos - prv;
    vel = mov / ((pdt == 0) ? dt : pdt);
    prv = pos;
    pos += vel * dt + 0.5f * acc * dt * dt;
    pdt = dt;
    acc = Vector3Zero();
}

void PointMass::draw() {

}

void PointMass::collide(PlanePtr p) {
    if (fixed || !collideable)
        return;
    auto proj = p->project(pos);
    bool collides = Vector3Distance(proj, pos) < radius;
    if (!collides)
        return;
    auto movproj = p->project(pos + vel * pdt) - proj;
    auto fricoeff = friction * p->friction;
    auto delta = (proj + p->normal * radius - pos);
    pos += (1.0f - bounce) * delta - movproj * fricoeff * pdt * 100.0f;
    collided = true;
}

void PointMass::collide(BlockPtr b) {
    if (fixed || !collideable)
        return;
    if (!Vector3InsideAABlock(b->pos, b->size + 2.0f * Vector3{radius, radius, radius}, pos))
        return;
    auto proj = Vector3ClosestPointOnBox(b->pos, b->size, pos);
    bool collides = Vector3Distance(proj, pos) < radius;
    if (!collides)
        return;
    auto normal = Vector3Normalize(pos - proj);
    auto movproj = Vector3ProjectOntoPlane(proj, normal, pos + vel * pdt) - proj;
    auto fricoeff = friction * b->friction;
    auto delta = (proj + normal * radius - pos);
    pos += (1.0f - bounce) * delta - movproj * fricoeff * pdt * 100.0f;
    collided = true;
}

void PointMass::collide(PointMassPtr pm) {
    if (!collideable || !pm->collideable)
	    return;
    if (fixed && pm->fixed)
        return;
    auto v = pm->pos - pos;
    float dist2 = v.x * v.x + v.y * v.y + v.z * v.z;
    float min_dist = radius + pm->radius;
    if (dist2 < min_dist * min_dist) {
        const float dist = sqrt(dist2);
        auto n = v / dist;
        const float sumass = mass + pm->mass;
        const float mr1 = fixed ? 1.0f : pm->fixed ? 0.0f : (mass / sumass);
        const float mr2 = 1.0f - mr1;
        const float delta = (dist - min_dist);
        if (!fixed) pos += n * mr2 * (1.0f - bounce) * delta;
        if (!pm->fixed) pm->pos -= n * (1.0f - pm->bounce) * mr1 * delta;
        collided = true;
    }
}

PointMassLink::PointMassLink(PointMassPtr pm1, PointMassPtr pm2, bool collideable, float length, float stiffness, float damping, 
            float alignStr, const Vector3& alignAxis) :
    pm1(pm1), pm2(pm2), collideable(collideable), length(length), stiffness(stiffness), damping(damping),
    alignStr(alignStr), alignAxis(alignAxis)
{ }

void PointMassLink::constrain(double dt) {
    if (Vector3Length(pm1->pos - pm2->pos) == 0)
        return;
    const float sumass = pm1->mass + pm2->mass;
    const float mr1 = pm1->fixed ? 1.0f : pm2->fixed ? 0.0f : (pm1->mass / sumass);
    const float mr2 = 1.0f - mr1;
    auto d = pm2->pos - pm1->pos;
    auto dist = Vector3Length(d);
    auto dir = d / dist;
    auto v1 = pm2->pos - dir * length;
    auto v2 = pm1->pos + dir * length;
    float stifcoeff = (stiffness < 1.0f) ? (stiffness * dt * 10.0f) : 1.0f;
    if (alignStr > 0.0f) {
        auto v1p = pm2->pos - length * alignAxis * Vector3DotProduct(Vector3Normalize(v1 - pm2->pos), -alignAxis);
        auto v2p = pm1->pos + length * alignAxis * Vector3DotProduct(Vector3Normalize(v2 - pm1->pos), alignAxis);
        v1 = v1 + (v1p - v1) * alignStr / stifcoeff * dt * 500.0f;
        v2 = v2 + (v2p - v2) * alignStr / stifcoeff * dt * 500.0f;
    }
    float dampcoeff = (damping < 1.0f) ? damping : 0.0f;
    if (!pm1->fixed) 
        pm1->pos += 0.5f * ((v1 - pm1->pos) * mr2 * stifcoeff + (pm2->mov - pm1->mov) * dampcoeff);
    if (!pm2->fixed) 
        pm2->pos += 0.5f * ((v2 - pm2->pos) * mr1 * stifcoeff + (pm1->mov - pm2->mov) * dampcoeff);
}

void PointMassLink::collide(BlockPtr b) {
    if (pm1->fixed && !pm2->fixed)
        return;
    Vector3 n1, n2;
    auto p1 = Vector3ClosestPointOnBox(b->pos, b->size, pm1->pos, &n1);
    auto p2 = Vector3ClosestPointOnBox(b->pos, b->size, pm2->pos, &n2);
    Vector3 proj, clos;
    if (util::valsAreClose(p1.x, p2.x) || util::valsAreClose(p1.y, p2.y) || util::valsAreClose(p1.z, p2.z)) {
        auto c1 = Vector3ClosestPointOnSegment(pm1->pos, pm2->pos, p1);
        auto c2 = Vector3ClosestPointOnSegment(pm1->pos, pm2->pos, p2);
        p1 = Vector3ClosestPointOnBox(b->pos, b->size, c1);
        p2 = Vector3ClosestPointOnBox(b->pos, b->size, c2);
        float t1 = Vector3Distance(c1, pm1->pos) / Vector3Distance(pm1->pos, pm2->pos);
        bool collides1 = (t1 > 0 && t1 < 1.0f);
        float t2 = Vector3Distance(c2, pm1->pos) / Vector3Distance(pm1->pos, pm2->pos);
        bool collides2 = (t2 > 0 && t2 < 1.0f);
        if (collides1 || collides2) {
            float r1 = (pm1->radius + (pm2->radius - pm1->radius) * t1);
            float r2 = (pm1->radius + (pm2->radius - pm1->radius) * t2);
            float d1 = Vector3Distance(c1, p1);
            float d2 = Vector3Distance(c2, p2);
            collides1 = d1 < r1;
            collides2 = d2 < r2;
            if (collides1 || collides2) {
                float sumass = pm1->mass + pm2->mass;
                float mr1 = pm1->fixed ? 1.0f : pm2->fixed ? 0.0f : (pm1->mass / sumass);
                float mr2 = 1.0f - mr1;
                if (collides1) {
                    auto normal = ((Vector3Distance(c1, b->pos) < Vector3Distance(p1, b->pos)) ? -1.0f : 1.0f) * Vector3Normalize(c1 - p1);
                    auto delta = Vector3Length(p1 + normal * r1 - c1);
                    auto bounce = (pm1->bounce + (pm2->bounce - pm1->bounce) * t1);
                    pm1->pos += (1.0f - bounce) * (normal * mr2 * (1.0f - t1)) * delta;
                    pm2->pos += (1.0f - bounce) * (normal * mr1 * t1) * delta;
                }
                if (collides2) {
                    auto normal = ((Vector3Distance(c2, b->pos) < Vector3Distance(p2, b->pos)) ? -1.0f : 1.0f) * Vector3Normalize(c2 - p2);
                    auto delta = Vector3Length(p2 + normal * r2 - c2);
                    auto bounce = (pm1->bounce + (pm2->bounce - pm1->bounce) * t2);
                    pm1->pos += (1.0f - bounce) * (normal * mr2 * (1.0f - t2)) * delta;
                    pm2->pos += (1.0f - bounce) * (normal * mr1 * t2) * delta;
                }
            }
        }
    } else {
        Vector3 p0, n;
        if (!Vector3PlanesIntersection(p1, n1, p2, n2, p0, n))
            return;
        auto halfSide = fabs(Vector3DotProduct(n, b->size) * 0.5);
        auto pc1 = Vector3ProjectOntoPlane(b->pos + n * halfSide, n, p0);
        auto pc2 = Vector3ProjectOntoPlane(b->pos - n * halfSide, -n, p0);
        auto pts = Vector3ClosestPointsOnSegments(pm1->pos, pm2->pos, pc1, pc2);
        clos = pts.first;
        proj = pts.second;
        float t = Vector3Distance(clos, pm1->pos) / Vector3Distance(pm1->pos, pm2->pos);
        if (t <= 0 || t >= 1.0f)
            return;
        auto dist = Vector3Distance(proj, clos);
        float r = (pm1->radius + (pm2->radius - pm1->radius) * t);
        bool collides = dist < r;
        if (!collides)
            return;
        float sumass = pm1->mass + pm2->mass;
        float mr1 = pm1->fixed ? 1.0f : pm2->fixed ? 0.0f : (pm1->mass / sumass);
        float mr2 = 1.0f - mr1;
        auto normal = ((Vector3Distance(clos, b->pos) < Vector3Distance(proj, b->pos)) ? -1.0f : 1.0f) * Vector3Normalize(clos - proj);
        auto delta = Vector3Length(proj + normal * r - clos);
        auto bounce = (pm1->bounce + (pm2->bounce - pm1->bounce) * t);
        pm1->pos += (1.0f - bounce) * (normal * mr2 * (1.0f - t)) * delta;
        pm2->pos += (1.0f - bounce) * (normal * mr1 * t) * delta;
    }
}

void PointMassLink::collide(PointMassPtr pm) {
    if (!pm->collideable || pm == pm1 || pm == pm2)
        return;
    auto proj = Vector3ClosestPointOnSegment(pm1->pos, pm2->pos, pm->pos);
    float t = Vector3Distance(proj, pm1->pos) / Vector3Distance(pm1->pos, pm2->pos);
    if (t <= 0 || t >= 1.0f)
        return;
    auto dist = Vector3Distance(proj, pm->pos);
    float r = (pm->radius + (pm1->radius + (pm2->radius - pm1->radius) * t));
    bool collides = dist < r;
    if (!collides)
        return;
    float sumass = pm1->mass + pm2->mass;
    float mr1 = pm1->fixed ? 1.0f : pm2->fixed ? 0.0f : (pm1->mass / sumass);
    float mr2 = 1.0f - mr1;
    float sumass2 = sumass + pm->mass;
    float mr1_ = (pm1->fixed && pm2->fixed) ? 1.0f : pm->fixed ? 0.0f : (sumass / sumass2);
    float mr2_ = 1.0f - mr1_;
    auto normal = Vector3Normalize(pm->pos - proj);
    auto delta = Vector3Length(proj + normal * r - pm->pos);
    auto movproj = Vector3ProjectOntoPlane(proj, normal, pm->pos + pm->vel * pm->pdt) - proj;
    auto fricoeff = (pm1->friction + (pm2->friction - pm1->friction) * t) * pm->friction;
    auto bounce = (pm1->bounce + (pm2->bounce - pm1->bounce) * t) * pm->bounce;
    if (!pm1->fixed || !pm2->fixed) {
        pm1->pos -= (1.0f - bounce) * (normal * mr2 * mr2_ * (1.0f - t)) * delta;
        pm2->pos -= (1.0f - bounce) * (normal * mr1 * mr2_ * t) * delta;
    }
    if (!pm->fixed) pm->pos += (1.0f - pm->bounce) * normal * mr1_ * delta - mr1_ * movproj * fricoeff * pm->pdt * 100.0;
}

void PointMassLink::draw() {

}

