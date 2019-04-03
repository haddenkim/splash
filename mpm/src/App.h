#include "PhysicsHook.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/unproject.h>
#include <thread>

static PhysicsHook* hook = NULL;

class App {
public:
	App(PhysicsHook* p_hook)
	{
		hook = p_hook;
		hook->reset();

		// libigl viewer settings
		// viewer.core.orthographic = true;
		// viewer.core.camera_zoom = 4.0;

		viewer.data().show_lines = false;
		viewer.data().set_face_based(false);
		viewer.core.is_animating = true;

		// libigl viewer callbacks
		viewer.callback_key_pressed = keyCallback;
		// viewer.callback_mouse_down = mouseCallback;
		// viewer.callback_mouse_scroll = mouseScroll;
		viewer.callback_pre_draw = drawCallback;

		// libigl GUI
		viewer.plugins.push_back(&menu);
		menu.callback_draw_viewer_menu = drawGUI;

		// initial camera position
		viewer.core.camera_eye	= Eigen::Vector3f(0.5, 2, 2);
		viewer.core.camera_center = Eigen::Vector3f(0.5, 0.5, 0.5);
	}

	void start()
	{
		viewer.launch();
	}

	// libigl
	igl::opengl::glfw::Viewer			viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;

	// libigl callbacks
	static bool drawCallback(igl::opengl::glfw::Viewer& viewer)
	{
		hook->render(viewer);
		return false;
	}

	static bool keyCallback(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifiers)
	{
		if (key == ' ') {
			hook->toggleSimulation();
			return true;
		}
		return false;
	}

	static bool mouseCallback(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
	{
		Eigen::Vector3f pos(viewer.down_mouse_x, viewer.down_mouse_y, 0);
		Eigen::Matrix4f model  = viewer.core.view;
		Eigen::Vector3f unproj = igl::unproject(pos, model, viewer.core.proj, viewer.core.viewport);
		hook->mouseClicked(unproj[0], -unproj[1], button);

		return true;
	}

	static bool mouseScroll(igl::opengl::glfw::Viewer& viewer, float delta)
	{
		return true;
	}

	static void drawGUI()
	{
		if (ImGui::CollapsingHeader("Simulation Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			bool isRunning = !hook->isPaused();
			if (isRunning) {
				ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 255, 0, 255)); // green
			}

			if (ImGui::Button("Run/Pause Sim", ImVec2(-1, 0))) {
				hook->toggleSimulation();
			}

			if (isRunning) {
				ImGui::PopStyleColor();
			}

			if (ImGui::Button("Reset Sim", ImVec2(-1, 0))) {
				hook->reset();
			}

			static int numSteps = 1;
			ImGui::SliderInt("n", &numSteps, 1, 100);
			ImGui::SameLine();
			if (ImGui::Button("Sim n Steps", ImVec2(-1, 0))) {
				for (int i = 0; i < numSteps; i++) {
					hook->simulateOneStep();
				}
			}
		}

		// continue drawing hook specific gui
		hook->drawGUI();
	}
};