#include "app/App.h"
#include "app/mpmHook.h"
#include "settings/systemStart.h"
#include "state/shape.h"

int main(int argc, char* argv[])
{
	SystemStart start;
	start.shapes.emplace_back(Shape(MODEL_SNOW, 0.5, 0.7, 0.5, 1, 0, 0, 0, 1, 0));

	App app(new MpmHook(start));

	app.start();
}
