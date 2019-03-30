#include "App.h"
#include "mpmHook.h"

int main(int argc, char* argv[])
{
	App app(new MpmHook());

	app.start();
}
