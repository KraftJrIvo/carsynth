#include "renderer.h"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <cmath>
#include <functional>
#include <map>

#include "vec_funcs.h"

using namespace areno;

#define SPEED 0.1f
#define FAST_COEFF 5.0f

Renderer::Renderer(World& w, const Vector2& winSize, const std::string& winName) :
	_w(w),
	_windowSize(winSize),
	_windowName(winName)
{
    _redrawer = std::thread([&]() {
        _init();
        while (!WindowShouldClose()) {
            _render();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    	}
        CloseWindow();
        done = true;
    });
}

Renderer::~Renderer() {
    _redrawer.join();
}

void Renderer::_resetPlayerPos() {
    _cam.position = { 0.0f, 2.0f, 4.0f };
    _cam.target = { 0.0f, 2.0f, 0.0f };
    _cam.up = { 0.0f, 1.0f, 0.0f };
    _cam.fovy = 60.0f;      
    _cam.projection = CAMERA_PERSPECTIVE;
}

void Renderer::_init() {
    InitWindow((int)_windowSize.x, (int)_windowSize.y, _windowName.c_str());
    _texBall = LoadTexture("res/ball.png");
    _texFloor = LoadTexture("res/checker.png");
    _chDefault = LoadTexture("res/crosshair.png");
    _chHand = LoadTexture("res/hand.png");
    _chGrab = LoadTexture("res/hand_grab.png");
    _resetPlayerPos();
    DisableCursor();
    SetTargetFPS(60);
}

void Renderer::_renderPointMasses() {
    lookingAtPM = nullptr;
    std::map<float, PointMassPtr, std::greater<float>> order;
    auto vm = GetCameraViewMatrix(&_cam); 
    Vector3 lookdir = {-vm.m2, -vm.m6, -vm.m10};
    Vector3 up = {vm.m1, vm.m5, vm.m9};
    for (auto it = _w._pointMasses.begin(); it != _w._pointMasses.end(); ++it) {
        auto pm = *it;
        auto to = pm->pos - _cam.position;
        bool inFront = Vector3DotProduct(lookdir, to) > 0;
        if (inFront && pm->radius > 0.0f) {
            auto dist = fabs(Vector3DotProduct(to, lookdir));
            order[dist] = pm;
        }
    }
    for (auto pmo : order) {
        auto pm = pmo.second;
        auto color = (pm == lookingAtPM) ? RED : pm->color;
        DrawBillboardPro(_cam, _texBall, {0.0f, 0.0f, 100.0f, 100.0f}, pm->pos, up, pm->radius * Vector2{2.0f, 2.0f}, {0.5f, 0.5f}, 0.0f, color);
        //links
        for (auto l : _w._links) {
            if (l->pm1 == pm) {
                DrawLine3D(l->pm1->pos, l->pm2->pos, BLACK);
                if (l->collideable) {
                    auto dir2d = Vector3Normalize(Vector3ProjectOntoPlane(_cam.position, lookdir, l->pm1->pos) - 
                        Vector3ProjectOntoPlane(_cam.position, lookdir, l->pm2->pos));
                    auto cwdir = Vector3RotateByAxisAngle(dir2d, lookdir, -PI / 2.0f);
                    DrawLine3D(l->pm1->pos + cwdir * l->pm1->radius, l->pm2->pos + cwdir * l->pm2->radius, BLACK);
                    DrawLine3D(l->pm1->pos - cwdir * l->pm1->radius, l->pm2->pos - cwdir * l->pm2->radius, BLACK);
                }
            }
        }
    }
}

void AllocateMeshData(Mesh* mesh, int triangleCount)
{
    mesh->vertexCount = triangleCount * 3;
    mesh->triangleCount = triangleCount;

    mesh->vertices = (float*)MemAlloc(mesh->vertexCount * 3 * sizeof(float));
    mesh->texcoords = (float*)MemAlloc(mesh->vertexCount * 2 * sizeof(float));
    mesh->normals = (float*)MemAlloc(mesh->vertexCount * 3 * sizeof(float));
}

Mesh MakeMesh(const PlanePtr p)
{
    Mesh mesh = { 0 };
    AllocateMeshData(&mesh, 2);

    float width = 100;
    float height = 100;

    float textureSize = 1.0f; // how big is the texture in world units

    auto& o = p->pos;
    auto orth = Vector3OrthogonalVector(p->normal);
    auto orth2 = Vector3RotateByAxisAngle(orth, p->normal, PI / 2.0f);
    auto p1 = o - 100 * orth;
    auto p2 = o + 100 * orth;
    auto p3 = o + 100 * orth2;
    auto p4 = o - 100 * orth2;

    // vertex at the origin
    mesh.vertices[0] = p1.x;
    mesh.vertices[1] = p1.y;
    mesh.vertices[2] = p1.z;
    mesh.normals[0] = p->normal.x;
    mesh.normals[1] = p->normal.y;
    mesh.normals[2] = p->normal.z;
    mesh.texcoords[0] = 0;
    mesh.texcoords[1] = 0;

    // vertex at 2,0,2
    mesh.vertices[3] = p2.x;
    mesh.vertices[4] = p2.y;
    mesh.vertices[5] = p2.z;
    mesh.normals[3] = p->normal.x;
    mesh.normals[4] = p->normal.y;
    mesh.normals[5] = p->normal.z;
    mesh.texcoords[2] = width/textureSize;
    mesh.texcoords[3] = height / textureSize;

    // vertex at 2,0,0
    mesh.vertices[6] = p3.x;
    mesh.vertices[7] = p3.y;
    mesh.vertices[8] = p3.z;
    mesh.normals[6] = p->normal.x;
    mesh.normals[7] = p->normal.y;
    mesh.normals[8] = p->normal.z;
    mesh.texcoords[4] = width / textureSize;
    mesh.texcoords[5] = 0;

    // vertex at the origin
    mesh.vertices[9] = p1.x;
    mesh.vertices[10] = p1.y;
    mesh.vertices[11] = p1.z;
    mesh.normals[9] =  p->normal.x;
    mesh.normals[10] = p->normal.y;
    mesh.normals[11] = p->normal.z;
    mesh.texcoords[6] = 0;
    mesh.texcoords[7] = 0;

    // vertex at 0,0,2
    mesh.vertices[12] = p4.x;
    mesh.vertices[13] = p4.y;
    mesh.vertices[14] = p4.z;
    mesh.normals[12] = p->normal.x;
    mesh.normals[13] = p->normal.y;
    mesh.normals[14] = p->normal.z;
    mesh.texcoords[8] = 0;
    mesh.texcoords[9] = height / textureSize;

    // vertex at 2,0,2
    mesh.vertices[15] = p2.x;
    mesh.vertices[16] = p2.y;
    mesh.vertices[17] = p2.z;
    mesh.normals[15] = p->normal.x;
    mesh.normals[16] = p->normal.y;
    mesh.normals[17] = p->normal.z;
    mesh.texcoords[10] = width / textureSize;
    mesh.texcoords[11] = height / textureSize;

    UploadMesh(&mesh, false);

    return mesh;
}

void Renderer::_renderPlanes() {
    static bool once = true;
    static std::map<PlanePtr, Model> models;
    if (once) {
        for (auto p : _w._planes) {
            Model model;
            model = LoadModelFromMesh(MakeMesh(p));
            model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = _texFloor;
            models[p] = model;
        }
        once = false;
    }
    for (auto p : _w._planes)
        DrawModel(models[p], Vector3Zero(), 1.0f, p->color);
}


void Renderer::_renderBlocks() {
    for (auto b : _w._blocks) {
        DrawCube(b->pos, b->size.x, b->size.y, b->size.z, b->color);
        DrawCubeWires(b->pos, b->size.x, b->size.y, b->size.z, BLACK);
    }
}

void Renderer::_render() {
    Vector3 rot = {GetMouseDelta().x*0.05f, GetMouseDelta().y*0.05f, 0.0f};
    bool shift = IsKeyDown(KEY_LEFT_SHIFT);
    float speed = SPEED * (shift ? FAST_COEFF : 1.0f);
    float forwards = ((IsKeyDown(KEY_W)) - (IsKeyDown(KEY_S))) * speed;
    Vector3 dir = Vector3Normalize(Vector3{_cam.target.x - _cam.position.x, _cam.target.y - _cam.position.y, _cam.target.z - _cam.position.z});
    float pitch = (dir.x * _cam.up.x + dir.y * _cam.up.y + dir.z * _cam.up.z) / (Vector3Length(dir));
    Vector3 forwardsR = Vector3RotateByAxisAngle(Vector3{ forwards, 0, 0 }, Vector3{ 0, 1.0f, 0 }, -pitch);
    float sideways = ((IsKeyDown(KEY_D)) - (IsKeyDown(KEY_A))) * speed;
    float vertical = ((IsKeyDown(KEY_E)) - (IsKeyDown(KEY_Q))) * speed;
    Vector3 verticalR = Vector3RotateByAxisAngle(Vector3{ 0, 0, vertical }, Vector3{ 0, 1.0f, 0 }, -pitch);


    _w.lock();
    if (IsKeyPressed(KEY_TAB))
        _carCamMode = !_carCamMode;
    if (_carCamMode) {
        auto wm = GetMouseWheelMove();
        if (wm != 0) {
            _targerCamDist += wm * 0.1f;
            _targerCamDist = std::clamp(_targerCamDist, 1.0f, 10.0f);            
        }
        _camDist += (_targerCamDist - _camDist) / 10.0f;
        auto angle = atan2(_cam.position.z - _w._carCenter.z, _cam.position.x - _w._carCenter.x);
        auto pos = Vector3{_w._carCenter.x + cos(angle) * _camDist, _w._carCenter.y + 1.0f, _w._carCenter.z + sin(angle) * _camDist};
        auto rpos = _w._carCenter + Vector3RotateByAxisAngle(pos - _w._carCenter, Vector3{0,1.0f,0}, -GetMouseDelta().x*0.001f);
        //rpos = _w._carCenter + Vector3RotateByAxisAngle(rpos - _w._carCenter, Vector3RotateByAxisAngle(verticalR, dir, PI / 2), -GetMouseDelta().y*0.001f);
        _cam.position = rpos;
        _cam.target = _w._carCenter;
        //UpdateCameraPro(&_cam, Vector3{ forwardsR.x + verticalR.x, forwardsR.y + sideways + verticalR.y, forwardsR.z + verticalR.z }, rot, 0.0f);
    } else {
        UpdateCameraPro(&_cam, Vector3{ forwardsR.x + verticalR.x, forwardsR.y + sideways + verticalR.y, forwardsR.z + verticalR.z }, rot, 0.0f);
    }
    _w.unlock();

    BeginDrawing();
    ClearBackground(RAYWHITE);
    {
        BeginMode3D(_cam);

        _renderPlanes();
        _renderBlocks();
        _renderPointMasses();

        EndMode3D();
    }
    auto chTex = grabbedPML ? _chGrab : lookingAtPM ? _chHand : _chDefault;
    Vector2 chPos = grabbedPML ? GetWorldToScreen(grabbedPML->pm2->pos, _cam) : Vector2{GetScreenWidth() * 0.5f, GetScreenHeight() * 0.5f};
    DrawTexture(chTex, chPos.x - chTex.width * 0.5f, chPos.y - chTex.height * 0.5f, WHITE);

    EndDrawing();
}