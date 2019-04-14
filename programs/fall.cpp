#include "App.h"
#include "mpmHook.h"
#include "state/shape.h"

int main(int argc, char* argv[])
{

	App app(new MpmHook({ Shape(0.5, 0.4, 0.5, 0, 0, 0, 1, 0, 1),
						  Shape(0.4, 0.6, 0.5, 0, 0, 0, 0, 1, 1),
						  Shape(0.6, 0.8, 0.5, 0, 0, 0, 0, 1, 0)

	}));

	app.start();
}
