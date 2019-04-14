#include "App.h"
#include "mpmHook.h"
#include "state/shape.h"

int main(int argc, char* argv[])
{

	App app(new MpmHook({ Shape(0.5, 0.7, 0.5, 30, 0, 0, 0, 1, 0)

	}));

	app.start();
}
