#pragma once

#include <thread>

#include "raylib.h"
#include "world.h"

#include <rcamera.h>

namespace areno
{
	class Renderer
	{
	public:
		Renderer(World& w, const Vector2& winSize, const std::string& winName);
		~Renderer();

		bool done = false;

	private:
		std::thread _redrawer;


		World& _w;
		Vector2 _windowSize;
		std::string _windowName;

		Camera _cam;

		Texture2D _texBall, _texFloor;
		Texture2D _chDefault, _chHand, _chGrab;

		PointMassPtr lookingAtPM = nullptr;
		PointMassLinkPtr grabbedPML = nullptr;

        bool _carCamMode = false;
		float _targerCamDist = 2.0f;
		float _camDist = _targerCamDist;

		void _resetPlayerPos();

		void _init();
		void _renderPointMasses();
		void _renderPlanes();
		void _renderBlocks();
		void _render();
	};
}