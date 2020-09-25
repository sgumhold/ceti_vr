#pragma once

#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/arrow_renderer.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/render/shader_program.h>
#include <random>
#include <vector>

#include <rigid_body.h>

/* TODO: check description comments for each member */

/** TODO: Description here. */
class rigid_body_viewer : 
	public cgv::base::node, 
	public cgv::render::drawable, 
	public cgv::gui::provider, 
	public cgv::gui::event_handler
{
protected:
	/**@name scene management*/
	//@{
	/// enumeration of the diffenent available scenes
	enum scene {
		S_PLANE,
		S_SLIDING,
		S_IMPACT,
		S_STACKED
	} current_scene;
	/// random generator used for generating randomized rigid bodies
	std::mt19937 rand_gen;
	/// if the scene should be reset
	bool do_reset;
	/// if the generation of a new body is requested
	bool do_generate_body;
	/// if the generation of a shooting body is requested
	bool do_shoot_body;
	/// whether to aply an explosive force to the bodies
	bool do_detonate_bomb;
	/// clear all bodies and generate the chosen scene
	void generate_scene();
	/// randomly generate a rigid body
	void generate_body();
	/// shoot a random body from the view towards the mouse position
	void shoot_body();
	//@}

	/**@name rendering */
	//@{
	/// pointer reference to the current view
	cgv::render::view* view_ptr;
	/// position in 3d space where the mouse pointer is hovering over
	dvec3 mouse_target;
	//@}

	/// the rigid body phisics system
	physics::rigid_body rb;
	/// whether continuus simulation should be performed
	bool animate;
	/// whether a single simulation step should be performed
	bool single_step;
	/// recurring event to update the simulation
	void timer_event(double, double);

public:
	/**@name main viewer interface implementation*/
	//@{
	/// construct plugin and set default parameters
	rigid_body_viewer();
	/// reflect members of the viewer
	bool self_reflect(cgv::reflect::reflection_handler& _rh);
	/// handle key events
	bool handle(cgv::gui::event& _e);
	/// stream help information to the given output stream
	void stream_help(std::ostream& _os);
	/// stream statistical information to the given stream
	void stream_stats(std::ostream& _os);
	/// triggers necessary events due to a value change in given member, this can be called by gui or config file
	void on_set(void* member_ptr);
	/// return the type name of the viewer
	std::string get_type_name() const { return "rigid_body_viewer"; }
	/// just configures background color
	bool init(cgv::render::context& ctx);
	/// ensure that framebuffer object is created in sampling resolution, that shader programs are built, and perform simulation step if necessary
	void init_frame(cgv::render::context& ctx);
	/// ensure the scene is created with selected number of bodies, and draw either texture in 2D or scene 3D mode
	void draw(cgv::render::context& ctx);
	/// create the gui 
	void create_gui();
	//@}
};
