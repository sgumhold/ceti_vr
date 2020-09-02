#pragma once

#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/frame_buffer.h>
#include <fstream>
#include <memory>
#include <chrono>

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>

class vr_box_stacking :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider {
protected:
	// different interaction states for the controllers
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};

	enum Axis {
		AXIS_X = 0,
		AXIS_Y = 1,
		AXIS_Z = 2
	};

	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;

	// rendering styles
	cgv::render::box_render_style style;
	cgv::render::rounded_cone_render_style cone_style;

	// sample for rendering text labels
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;
	bool label_outofdate; // whether label texture is out of date
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label
	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;
	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;

	// store handle to vr kit of which left deadzone and precision is configured
	void* last_kit_handle;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// record of the vr_events
	std::shared_ptr<std::ofstream> vr_events_stream, box_trajectory_stream, controller_trajectory_stream;
	std::string vr_events_record_path;
	bool log_vr_events;
	int grab_number[4];
	std::chrono::time_point<std::chrono::steady_clock> vrr_t_start; //time since start of recording

	//settings for live editing boxes
	bool box_edit_mode;
	box3 new_box;
	float new_box_distance;
	rgb new_box_color;
	
	float edit_box_step;
	float edit_box_max_size;
	int edit_box_selected_axis;
	float grid_step;
	float rotation_grid_step;
	bool snap_box_to_grid;
	std::string message_t;
	bool show_message_t;

	// store the movable boxes
	std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;

	// store the wireframe boxes
	std::vector<box3> frame_boxes;
	std::vector<rgb> frame_box_colors;
	std::vector<vec3> frame_box_translations;
	std::vector<quat> frame_box_rotations;

	// store highlighting for grabbed box
	std::vector<box3> highlighted_boxes;
	std::vector<rgba> highlighted_box_colors;
	std::vector<vec3> highlighted_box_translations;
	std::vector<quat> highlighted_box_rotations;
	std::vector<int> highlighted_box_controler;
	std::vector<int> highlighted_box_id;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;
	std::vector<vec3> intersection_grab_translations;
	std::vector<quat> intersection_grab_rotations;
	std::vector<char> intersection_grab_initialized;

	// state of current interaction with boxes for each controller
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;
	cgv::render::box_render_style wire_frame_style;
public:
	/// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_table(float tw, float td, float th, float tW);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	/// construct boxes for environment
	void construct_environment(float s, float ew, float ed, float w, float d, float h);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_movable_boxes(float tw, float td, float th, float tW, size_t nr);
	/// construct a scene with a table
	void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW);
public:
	vr_box_stacking();

	std::string get_type_name() { return "vr_box_stacking"; }

	void stream_help(std::ostream& os);

	void on_set(void* member_ptr);

	bool handle(cgv::gui::event& e);
	
	bool init(cgv::render::context& ctx);

	void clear(cgv::render::context& ctx);

	void init_frame(cgv::render::context& ctx);

	void draw(cgv::render::context& ctx);

	void create_gui();

	bool self_reflect(cgv::reflect::reflection_handler& rh);

	void read_configuration(int cfg_index);

	//! stores configuration of the movable boxes inside a file
	/*! stores configuration of the movable boxes inside a file. All passed vectors in the arguments are reqired to have the same number of elements.
		@param fn : file for writing the configuration */
	bool save_boxes(const std::string fn, const std::vector<box3>& boxes, const std::vector<rgb>& box_colors, const std::vector<vec3>& box_translations, const std::vector<quat>& box_rotations);
	
	//! loads boxes stored by the save_boxes method from a file
	/*! loads boxes stored by the save_boxes method. This method will append the box information from the file to the vectors given by the arguments.*/
	bool load_boxes(const std::string fn, std::vector<box3>& boxes, std::vector<rgb>& box_colors, std::vector<vec3>& box_translations, std::vector<quat>& box_rotations);

	bool judge_box_target(int box_id);

protected:

	void resize_box(int box_index, vec3 extends);

	void on_save_movable_boxes_cb();
	void on_load_movable_boxes_cb();
	void on_load_wireframe_boxes_cb();
	void clear_movable_boxes();
	void clear_frame_boxes();
	void on_set_vr_event_streaming_file();

	void change_box_extents(Axis axis,int ci);
	void delete_box(int bi);
	size_t clear_intersections(int ci);
	size_t clear_highlighted_boxes(int ci);
	size_t clear_highlighted_boxes(int ci, int bi);

	static float quadratic_error(const box3& a, const box3& b,const vec3 & translation_a, const vec3 & translation_b, const quat & rotation_a, const quat & rotation_b);
	//! stores the corners of a box in an array
	static void box_corners(const box3& b,const vec3 & translation,const quat & rotation, vec3* points);
	//! approximate nearest box by comparing the distances between p and the center of the given boxes
	static int nearest_box_center(const vec3& p, const box3* boxes, const int boxes_size, const int exclude=-1);
};

///@}
