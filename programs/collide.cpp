#include "App.h"
#include "mpmHook.h"
#include "state/shape.h"

int main(int argc, char* argv[])
{

	App app(new MpmHook({ Shape(0.2, 0.8, 0.2, 20, 0, 20, 0, 1, 1),
						  Shape(0.8, 0.7, 0.5, -20, 0, 0, 1, 0, 1)

	}));

	app.start();
}
