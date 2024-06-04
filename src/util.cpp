#include "util.h"
#include "raylib.h"

float areno::util::randFloat() {
    return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX) + 1.0f);
}

Color areno::util::getUniqueColorById(unsigned int id) {
	return {(unsigned char)((70 + 150 * id) % 255), (unsigned char)((150 + 70 * id) % 255), (unsigned char)((30 + 200 * id) % 255), 255};
}

bool areno::util::valsAreClose(float val1, float val2) {
    return fabs(val1 - val2) < 1e-6;
}