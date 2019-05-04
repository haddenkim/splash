#include "app/App.h"
#include "app/mpmHook.h"
#include "settings/systemStart.h"
#include "state/shape.h"

int main(int argc, char* argv[])
{
	SystemStart start;
	start.shapes.emplace_back(Shape(MODEL_SNOW, 0.5, 0.4, 0.5, 0, 0, 0, 1, 0, 1));
	start.shapes.emplace_back(Shape(MODEL_SNOW, 0.4, 0.6, 0.5, 0, 0, 0, 0, 1, 1));
	start.shapes.emplace_back(Shape(MODEL_SNOW, 0.6, 0.8, 0.5, 0, 0, 0, 0, 1, 0));

	App app(new MpmHook(start));

	app.start();
}
