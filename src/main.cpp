#include "renderer.h"

int main()
{
	areno::WorldConfig config("calib.yaml");

	areno::World w(config);
	w.generate();

	areno::Renderer r(w, {1024.0f, 1024.0f}, "CARSYNTH");

	while (!r.done) {
		w.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}