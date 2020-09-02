#include "vr_tool.h"
#include <cgv/base/node.h>
#include <cgv/render/context.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/gui/event_handler.h>
#include <vr/vr_state.h>
#include <vr/vr_kit.h>
#include <vr/vr_driver.h>
#include <cg_vr/vr_events.h>
#include <vr_view_interactor.h>

class vr_draw : public vr_tool
{
protected:
	// render configuration
	cgv::render::sphere_render_style srs;
	cgv::render::rounded_cone_render_style rcrs;

	/// different drawing modes
	enum DrawMode {
		DM_POINT,    // only draw points
		DM_LINE,     // draw points connected with lines
		DM_COLORIZE  // change color of points
	};
	DrawMode draw_mode[2];
	/// per controller radius used to draw with touch pad center
	float draw_radius[2];
	/// per controller a draw color
	rgb   draw_color[2];
	/// members used for color adjustment
	bool in_color_selection[2];
	vec3 color_selection_ref[2];
	rgb last_color[2];
	/// distance of drawing point from controller origin
	float draw_distance;
	/// threshold for new vertex creation in measured in meters
	float creation_threshold;
	/// parameters to map trigger to radius
	float min_trigger;
	float min_radius;
	float max_radius;
	/// distance of point p to line through l0 and l1 
	static float distance(const vec3& p, const vec3& l0, const vec3& l1)
	{
		vec3 dl = l1 - l0;
		vec3 dp = p  - l0;
		float lambda = dot(dp, dl) / dot(dl, dl);
		if (lambda > 0.0f && lambda < 1.0f)
			return length(dp + lambda * dl);
		return std::min(length(dp), length(p - l1));
	}
private:
	uint32_t li_help[2];
	// per controller whether we are drawing 
	bool   drawing[2];
	// per controller last used radius
	float  last_radius[2];
	// per controller cache of previously drawn vertices
	int32_t prev[2];
	int32_t prev_prev[2];
	int32_t prev_prev_prev[2];
	
	bool in_radius_adjustment[2];
	float initial_radius[2];
	float initial_y[2];

	/// transform point with pose to lab coordinate system 
	vec3 compute_lab_draw_position(const float* pose, const vec3& p)
	{
		return mat34(3, 4, pose) * vec4(p, 1.0f);
	}
	/// transform default draw point with pose to lab coordinate system 
	vec3 compute_lab_draw_position(const float* pose)
	{
		return compute_lab_draw_position(pose, vec3(0.0f, 0.0f, -draw_distance));
	}
	/// check newly tracked position and add new vertex if necessary
	void consider_vertex(int ci, const vec3& p, double time, float radius)
	{
		if (!scene_ptr)
			return;
		// manage radius
		if (radius == -1.0f)
			radius = last_radius[ci];
		else
			last_radius[ci] = radius;

		// when we start drawing, just add new vertex
		if (prev[ci] == -1) {			
			prev[ci] = scene_ptr->add_vertex({ p, radius, draw_color[ci] });
			// std::cout << " starting" << std::endl;
		}
		else {
			// otherwise check if we can update prev vertex
			auto& v_prev = scene_ptr->ref_vertex(prev[ci]);
			float dist = length(v_prev.position - p);
			// first check if new ball encloses previous or previous encloses new ball
			if (dist + v_prev.radius < radius ||
				dist + radius < v_prev.radius) {
				v_prev.position = p;
				v_prev.radius = radius;
				// std::cout << " inout" << std::endl;
			}
			else {
				// otherwise compute prediction
				bool no_update = true;
				vec3  p_pred = v_prev.position;
				float r_pred = v_prev.radius;
				if (prev_prev[ci] != -1) {
					const auto& v_prev_prev = scene_ptr->get_vertex(prev_prev[ci]);
					// check for direction reversal
					vec3 d_pred = v_prev.position - v_prev_prev.position;
					vec3 d = p - v_prev.position;
					if (dot(d_pred, d) >= 0.0f) {
						no_update = false;
						p_pred = v_prev_prev.position;
						r_pred = v_prev_prev.radius;
						if (prev_prev_prev[ci] != -1) {
							const auto& v_prev_prev_prev = scene_ptr->get_vertex(prev_prev_prev[ci]);
							vec3 d_pred = v_prev_prev.position - v_prev_prev_prev.position;
							float l_pred_sqr = dot(d_pred, d_pred);
							if (l_pred_sqr > 1e-8f) {
								vec3 d = p - v_prev_prev.position;
								float lambda = dot(d_pred, d) / l_pred_sqr;
								if (lambda < 0)
									lambda = 0;
								p_pred = v_prev_prev.position + lambda * d_pred;
								r_pred = v_prev_prev.radius + lambda * (v_prev_prev.radius - v_prev_prev_prev.radius);
							}
						}
					}
				}
				// and check whether this is not good enough
				if (length(p - p_pred) > creation_threshold ||
					abs(radius - r_pred) > creation_threshold) {

					prev_prev_prev[ci] = prev_prev[ci];
					prev_prev[ci] = prev[ci];
					prev[ci] = scene_ptr->add_vertex({ p, radius, draw_color[ci] });
					// std::cout << " new" << std::endl;

					if (draw_mode[ci] == DM_LINE)
						scene_ptr->add_edge({ uint32_t(prev_prev[ci]), uint32_t(prev[ci]) });

				}
				else {
					if (!no_update) {
						v_prev.position = p;
						v_prev.radius = radius;
					}
				}
			}
		}
		post_redraw();
	}
	/// simplest approach to colorize scene vertices
	void colorize_vertex(int ci, const vec3& p, float radius)
	{
		if (!scene_ptr)
			return;
		for (uint32_t vi = 0; vi < scene_ptr->get_nr_vertices(); ++vi) {
			if ((scene_ptr->get_vertex(vi).position - p).length() < radius)
				scene_ptr->ref_vertex(vi).color = draw_color[ci];
		}
	}
	/// helper function called when we start drawing
	void start_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		drawing[ci] = true;
		if (draw_mode[ci] != DM_COLORIZE) {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose);
			consider_vertex(ci, p, time, radius);
		}
		else {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose, vec3(0,0,-50*draw_radius[ci]));
			colorize_vertex(ci, p, 10*draw_radius[ci]);
		}
	}
	/// helper function called when we continue drawing
	void continue_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		if (draw_mode[ci] != DM_COLORIZE) {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose);
			consider_vertex(ci, p, time, radius);
		}
		else {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose, vec3(0, 0, -50 * draw_radius[ci]));
			colorize_vertex(ci, p, 10*draw_radius[ci]);
		}
	}
	/// helper function called when we stop drawing
	void stop_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		if (draw_mode[ci] != DM_COLORIZE) {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose);
			consider_vertex(ci, p, time, radius);
			prev[ci] = prev_prev[ci] = prev_prev_prev[ci] = -1;
		}
		drawing[ci] = false;
	}
public:
	vr_draw() : vr_tool("vr_draw")
	{
		draw_mode[0] = draw_mode[1] = DM_LINE;
		in_color_selection[0] = in_color_selection[1] = false;
		in_radius_adjustment[0] = in_radius_adjustment[1] = false;
		draw_radius[0] = draw_radius[1] = 0.01f;
		draw_color[0] = rgb(1.0f, 0.3f, 0.7f);
		draw_color[1] = rgb(0.7f, 0.3f, 1.0f);
		draw_distance = 0.1f;
		creation_threshold = 0.002f;
		min_trigger = 0.03f;
		min_radius = 0.001f;
		max_radius = 0.03f;
		li_help[0] = li_help[1] = -1;

		drawing[0] = drawing[1] = false;
		prev[0] = prev_prev[0] = prev_prev_prev[0] = prev[1] = prev_prev[1] = prev_prev_prev[1] = -1;
	}
	std::string get_type_name() 
	{ 
		return "vr_draw"; 
	}
	void on_set(void* member_ptr)
	{
		update_member(member_ptr);
		post_redraw();
	}
	bool init(cgv::render::context& ctx)
	{
		cgv::render::ref_sphere_renderer(ctx, 1);
		cgv::render::ref_rounded_cone_renderer(ctx, 1);
		return true;
	}
	void init_frame(cgv::render::context& ctx)
	{
		vr_tool::init_frame(ctx);
		if (scene_ptr && li_help[0] == -1) {
			for (int ci = 0; ci < 2; ++ci) {
				li_help[ci] = scene_ptr->add_label("DPAD_Up .. toggle draw mode\nTPAD_Touch&Up/Dn .. change radius\nTPAD_Touch&Move .. change color\ncolorize (0.000)\nRGB(0.00,0.00,0.00)\nHLS(0.00,0.00,0.00)",
					rgba(ci == 0 ? 0.8f : 0.4f, 0.4f, ci == 1 ? 0.8f : 0.4f, 1.0f));
				scene_ptr->fix_label_size(li_help[ci]);
				scene_ptr->place_label(li_help[ci], vec3(ci == 1 ? -0.05f : 0.05f, 0.0f, 0.0f), quat(vec3(1, 0, 0), -1.5f),
					ci == 0 ? vr_scene::CS_LEFT_CONTROLLER : vr_scene::CS_RIGHT_CONTROLLER, ci == 1 ? vr_scene::LA_RIGHT : vr_scene::LA_LEFT, 0.2f);
				scene_ptr->hide_label(li_help[ci]);
			}
		}
		static const char* draw_mode_str[] = { "point","line","colorize" };
		for (int ci = 0; ci < 2; ++ci) {
			if (li_help[ci] == -1)
				continue;
			// update help text
			cgv::media::color<float, cgv::media::HLS> hls = draw_color[ci];
			std::stringstream ss;
			ss << "DPAD_Up .. toggle draw mode\nTPAD_Touch&Up/Dn .. change radius\nTPAD_Touch&Move .. change color\n"
				<< draw_mode_str[draw_mode[ci]] << " (" << std::setw(4) << std::setprecision(2) << draw_radius[ci] << ")"
				<< "\nRGB(" << std::setw(4) << std::setprecision(2) << draw_color[ci][0] << "," << std::setw(4) << std::setprecision(2) << draw_color[ci][1] << "," << std::setw(4) << std::setprecision(2) << draw_color[ci][2] << ")"
				<< "\nHLS(" << std::setw(4) << std::setprecision(2) << hls[0] << "," << std::setw(4) << std::setprecision(2) << hls[1] << "," << std::setw(4) << std::setprecision(2) << hls[2] << ")";
			ss.flush();
			scene_ptr->update_label_text(li_help[ci], ss.str());
			// update visibility of labels
			if (in_color_selection[ci])
				scene_ptr->show_label(li_help[ci]);
			else
				scene_ptr->hide_label(li_help[ci]);
		}
	}
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_sphere_renderer(ctx, -1);
		cgv::render::ref_rounded_cone_renderer(ctx, -1);
	}
	void draw(cgv::render::context& ctx)
	{
		// draw tool in case it is active and we have access to state 
		if (!(tool_is_active && kit_ptr && vr_view_ptr))
			return;
		const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
		if (!state_ptr)
			return;

		// draw spheres that represent the pen
		std::vector<vec3> P;
		std::vector<float> R;
		std::vector<rgb> C;
		for (int ci = 0; ci < 2; ++ci)
			if (draw_mode[ci] != DM_COLORIZE && state_ptr->controller[ci].status == vr::VRS_TRACKED) {
				P.push_back(compute_lab_draw_position(state_ptr->controller[ci].pose));
				R.push_back(drawing[ci] ? last_radius[ci] : draw_radius[ci]);
				C.push_back(draw_color[ci]);
			}
		if (!P.empty()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(srs);
			sr.set_position_array(ctx, P);
			sr.set_radius_array(ctx, R);
			sr.set_color_array(ctx, C);
			sr.render(ctx, 0, (GLsizei)P.size());
		}
	}
	void finish_frame(cgv::render::context& ctx)
	{
		// draw tool in case it is active
		if (!(tool_is_active && kit_ptr && vr_view_ptr))
			return;
		const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
		if (!state_ptr)
			return;

		std::vector<vec3> P;
		std::vector<float> R;
		std::vector<rgba> C;
		for (int ci = 0; ci < 2; ++ci)
			if (draw_mode[ci] == DM_COLORIZE && state_ptr->controller[ci].status == vr::VRS_TRACKED) {
				P.push_back(compute_lab_draw_position(state_ptr->controller[ci].pose, vec3(0.0f)));
				R.push_back(0.01f);
				C.push_back(draw_color[ci]); C.back().alpha() = 0.5f;
				P.push_back(compute_lab_draw_position(state_ptr->controller[ci].pose, vec3(0.0f, 0.0f, -50*draw_radius[ci])));
				R.push_back(10*draw_radius[ci]);
				C.push_back(draw_color[ci]); C.back().alpha() = 0.5f;
			}
		if (P.empty())
			return;

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);

		auto& rcr = cgv::render::ref_rounded_cone_renderer(ctx);
		rcr.set_render_style(rcrs);
		rcr.set_position_array(ctx, P);
		rcr.set_radius_array(ctx, R);
		rcr.set_color_array(ctx, C);
		if (rcr.validate_and_enable(ctx)) {
			if (P.size() == 2)
				rcr.draw(ctx, 0, (GLsizei)P.size());
			else {
				dvec3 e = vr_view_ptr->get_eye();
				if (distance(e, P[0], P[1]) > distance(e, P[2], P[3]))
					rcr.draw(ctx, 0, (GLsizei)P.size());
				else {
					rcr.draw(ctx, 2, 2);
					rcr.draw(ctx, 0, 2);
				}
			}
			rcr.disable(ctx);
		}
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
	}
	void stream_help(std::ostream& os)
	{
		os << "vr_draw: select draw <M>ode, press vr pad to draw" << std::endl;
	}
	bool handle(cgv::gui::event& e)
	{
		if ((e.get_flags() && cgv::gui::EF_VR) == 0) {
			if (e.get_kind() != cgv::gui::EID_KEY)
				return false;
			auto& ke = static_cast<cgv::gui::key_event&>(e);
			if (ke.get_action() == cgv::gui::KA_RELEASE)
				return false;
			switch (ke.get_key()) {
			case 'M': 
				if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					draw_mode[1] = draw_mode[1] == DM_COLORIZE ? DM_POINT : DrawMode(draw_mode[1] + 1);
					on_set(&draw_mode[1]);
				}
				else {
					draw_mode[0] = draw_mode[0] == DM_COLORIZE ? DM_POINT : DrawMode(draw_mode[0] + 1);
					on_set(&draw_mode[0]);
				}
				return true;
			}
			return false;
		}
		if (e.get_kind() == cgv::gui::EID_KEY) {
			auto& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			int ci = vrke.get_controller_index();
			switch (vrke.get_key()) {
			case vr::VR_DPAD_UP:
				if (vrke.get_action() == cgv::gui::KA_PRESS) {
					draw_mode[ci] = draw_mode[ci] == DM_COLORIZE ? DM_POINT : DrawMode(draw_mode[ci] + 1);
					on_set(&draw_mode[ci]);
				}
				return true;
			case vr::VR_GRIP:
				if (vrke.get_action() == cgv::gui::KA_PRESS) {
					in_color_selection[ci] = true;
					color_selection_ref[ci] = reinterpret_cast<const vec3&>(vrke.get_state().controller[ci].pose[9]);
					last_color[ci] = draw_color[ci];
				}
				else {
					in_color_selection[ci] = false;
				}
				return true;
			case vr::VR_INPUT0:
				if (vrke.get_action() == cgv::gui::KA_PRESS)
					start_drawing(vrke.get_controller_index(), vrke.get_state(), vrke.get_time(), draw_radius[vrke.get_controller_index()]);
				else
					stop_drawing(vrke.get_controller_index(), vrke.get_state(), vrke.get_time());
				return true;
			}
			return false;
		}
		else if (e.get_kind() == cgv::gui::EID_THROTTLE) {
			auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
			int ci = te.get_controller_index();
			float v = te.get_value();
			bool d = v >= min_trigger;
			if (d) {
				if (drawing[ci]) {
					last_radius[ci] = min_radius + (max_radius - min_radius) * (v - min_trigger) / (1.0f - min_trigger);
					continue_drawing(ci, te.get_state(), te.get_time(), last_radius[ci]);
				}
				else
					start_drawing(ci, te.get_state(), te.get_time(), min_radius);
			}
			else if (drawing[ci])
				stop_drawing(ci, te.get_state(), te.get_time(), min_radius);
		}
		else if (e.get_kind() == cgv::gui::EID_STICK) {
			auto& se = static_cast<cgv::gui::stick_event&>(e);
			int ci = se.get_controller_index();
			switch (se.get_action()) {
			case cgv::gui::SA_TOUCH :
				in_radius_adjustment[ci] = true;
				initial_radius[ci] = draw_radius[ci];
				initial_y[ci] = se.get_y();
				return true;
			case cgv::gui::SA_RELEASE:
				in_radius_adjustment[ci] = false;
				return true;
			case cgv::gui::SA_MOVE:
				if (in_radius_adjustment[ci]) {
					draw_radius[ci] = initial_radius[ci] * exp(se.get_y() - initial_y[ci]);
					update_member(&draw_radius[ci]);
				}
				return true;
			}
			return false;
		}
		else if (e.get_kind() == cgv::gui::EID_POSE) {
			auto& pe = static_cast<cgv::gui::vr_pose_event&>(e);
			int ci = pe.get_trackable_index();
			if (ci >= 0 && ci < 2) {
				if (in_color_selection[ci]) {					
					vec3 dp = reinterpret_cast<const vec3&>(pe.get_state().controller[ci].pose[9]) - color_selection_ref[ci];
					cgv::media::color<float, cgv::media::HLS> hls = last_color[ci];
					for (int k = 0; k < 3; ++k)
						hls[k] = std::max(0.0f,std::min(1.0f,hls[k] + 4.0f * dp[k]));
					draw_color[ci] = hls;
					update_member(&draw_color[ci]);
					return true;
				}
				else if (drawing[ci]) {
					continue_drawing(ci, pe.get_state(), pe.get_time());
					return true;
				}
			}
		}
		return false;
	}
	void create_gui()
	{
		add_decorator("vr_draw", "heading");
		if (begin_tree_node("rendering", srs.material)) {
			align("\a");
			if (begin_tree_node("spheres", srs)) {
				align("\a");
				add_gui("spheres", srs);
				align("\b");
				end_tree_node(srs);
			}
			if (begin_tree_node("cones", rcrs)) {
				align("\a");
				add_gui("cones", rcrs);
				align("\b");
				end_tree_node(rcrs);
			}
			align("\b");
			end_tree_node(srs.material);
		}
		if (begin_tree_node("interaction", draw_mode)) {
			align("\a");
			add_member_control(this, "left_draw_mode", draw_mode[0], "dropdown", "enums='point,line,colorize'");
			add_member_control(this, "right_draw_mode", draw_mode[1], "dropdown", "enums='point,line,colorize'");
			add_member_control(this, "left_draw_radius", draw_radius[0], "value_slider", "min=0.001;max=0.2;step=0.00001;log=true;ticks=true");
			add_member_control(this, "right_draw_radius", draw_radius[1], "value_slider", "min=0.001;max=0.2;step=0.00001;log=true;ticks=true");
			add_member_control(this, "left_draw_color", draw_color[0]);
			add_member_control(this, "right_draw_color", draw_color[1]);
			add_member_control(this, "draw_distance", draw_distance, "value_slider", "min=0.01;max=0.5;log=true;step=0.00001;ticks=true");
			add_member_control(this, "creation_threshold", creation_threshold, "value_slider", "min=0.001;max=0.1;log=true;step=0.00001;ticks=true");
			add_member_control(this, "min_trigger", min_trigger, "value_slider", "min=0.01;max=0.5;log=true;step=0.00001;ticks=true");
			add_member_control(this, "min_radius", min_radius, "value_slider", "min=0.001;max=0.1;log=true;step=0.00001;ticks=true");
			add_member_control(this, "max_radius", max_radius, "value_slider", "min=0.1;max=2;log=true;step=0.00001;ticks=true");
			align("\b");
			end_tree_node(srs);
		}
	}
};


#include <cgv/base/register.h>
cgv::base::object_registration<vr_draw> vr_draw_reg("vr_draw");
