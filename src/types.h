#pragma once

#include <memory>

#include "raylib.h"
#include "raymath.h"
#include "vec_ops.h"

namespace areno {

	struct Plane {
		Vector3 pos;
		Vector3 normal;
		float friction;
        Color color;
        Plane() = default;
        Plane(const Vector3& pos, const Vector3& normal, float friction = 0.0f, Color color = WHITE);
        Plane(const Vector3& p1, const Vector3& p2, const Vector3& p3, float friction = 0.0f, Color color = WHITE);
        Vector3 project(const Vector3& p);
	};
    typedef std::shared_ptr<Plane> PlanePtr;

    struct Block {
		Vector3 pos;
        Vector3 size;
		float friction;
        Color color;
        Block() = default;
        Block(const Vector3& pos, const Vector3& size, float friction = 0.0f, Color color = WHITE);
        bool contains(const Vector3& pos);
	};
    typedef std::shared_ptr<Block> BlockPtr;

    struct PointMass;
    typedef std::shared_ptr<PointMass> PointMassPtr;
    struct PointMass {
        Vector3 pos;
        float radius, friction, mass, bounce;
        Color color;
        bool fixed, collideable, collided;
        Vector3 prv, prvprv, mov, vel, prvvel, acc, prvacc;
        float pdt = 0;

        PointMass(const Vector3& pos, float radius, float friction, float bounce, Color color, bool fixed, bool collideable) : 
            pos(pos), radius(radius), friction(friction),
            mass((4.0f / 3.0f) * 3.14159f * radius * radius * radius), 
            bounce(bounce), color(color), fixed(fixed), collideable(collideable), collided(false),
            prv(pos), prvprv(pos), mov(Vector3Zero()), vel({0.0f, 0.0f, 0.0f }), prvvel({ 0.0f, 0.0f, 0.0f }), acc({ 0.0f, 0.0f, 0.0f }), prvacc({ 0.0f, 0.0f, 0.0f })
        { }

        void applyForce(Vector3 dir, float strength);
        void update(double dt);
        void draw();
        void collide(PlanePtr p);
        void collide(BlockPtr b);
        void collide(PointMassPtr pm);
    };

    struct PointMassLink {
        PointMassPtr pm1, pm2;
        bool collideable;
        float length;
        float stiffness;
        float damping;
        float alignStr;
        Vector3 alignAxis; 
        PointMassLink(PointMassPtr pm1, PointMassPtr pm2, bool collideable, float length, float stiffness, float damping = 0.0f, 
            float alignStr = 0.0f, const Vector3& alignAxis = Vector3Zero());
        void constrain(double dt);
        void collide(PlanePtr p);
        void collide(BlockPtr b);
        void collide(PointMassPtr pm);
        void draw();
    };
    typedef std::shared_ptr<PointMassLink> PointMassLinkPtr;
}