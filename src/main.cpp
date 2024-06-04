#include "renderer.h"

int main()
{
	areno::World w;
	w.generate();

	areno::Renderer r(w, {1024.0f, 1024.0f}, "ARENO");

	while (!r.done) {
		w.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}