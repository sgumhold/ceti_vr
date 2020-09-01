#include "vr_tool.h"
#include <cgv/base/node.h>
#include <cgv/render/context.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/box_renderer.h>
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

	enum DrawMode {
		DM_POINT,
		DM_LINE
	};
	DrawMode draw_mode;
	float draw_radius;
	rgb   draw_color;
	float draw_distance;
	float creation_threshold;
	float min_throttle;
	float min_radius;
	float max_radius;
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
	bool   drawing[2];
	float  last_radius[2];
	int32_t prev[2];
	int32_t prev_prev[2];
	int32_t prev_prev_prev[2];
	vec3 compute_world_draw_position(const float* pose, const vec3& p)
	{
		return mat34(3, 4, pose) * vec4(p, 1.0f);
	}
	vec3 compute_world_draw_position(const float* pose)
	{
		return compute_world_draw_position(pose, vec3(0.0f, 0.0f, -draw_distance));
	}
	void consider_vertex(int ci, const vec3& p, double time, float radius)
	{
		if (!scene_ptr)
			return;
		// manage radius
		if (radius == -1.0f)
			radius = last_radius[ci];
		else
			last_radius[ci] = radius;

		// std::cout << "cv " << p << ":" << radius << " [" << prev_prev_prev[ci] << "," << prev_prev[ci] << "," << prev[ci] << "]";
		// when we start drawing, just add new vertex
		if (prev[ci] == -1) {			
			prev[ci] = scene_ptr->add_vertex({ p, radius, draw_color });
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
					prev[ci] = scene_ptr->add_vertex({ p, radius, draw_color });
					// std::cout << " new" << std::endl;

					if (draw_mode == DM_LINE)
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
	void start_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		drawing[ci] = true;
		vec3 p = compute_world_draw_position(state.controller[ci].pose);
		consider_vertex(ci, p, time, radius);
	}
	void continue_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		vec3 p = compute_world_draw_position(state.controller[ci].pose);
		consider_vertex(ci, p, time, radius);
	}
	void stop_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		vec3 p = compute_world_draw_position(state.controller[ci].pose);
		consider_vertex(ci, p, time, radius);
		drawing[ci] = false;
		prev[ci] = prev_prev[ci] = prev_prev_prev[ci] = -1;
	}
public:
	vr_draw() : vr_tool("vr_draw")
	{
		draw_mode = DM_LINE;
		draw_radius = 0.01f;
		draw_color = rgb(1.0f, 0.3f, 0.7f);
		draw_distance = 0.1f;
		creation_threshold = 0.002f;
		min_throttle = 0.03f;
		min_radius = 0.001f;
		max_radius = 0.03f;

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
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_rounded_cone_renderer(ctx, 1);
		return true;
	}
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_sphere_renderer(ctx, -1);
		cgv::render::ref_box_renderer(ctx, -1);
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

		std::vector<vec3> P;
		std::vector<float> R;
		std::vector<rgb> C;
		for (int ci = 0; ci < 2; ++ci)
			if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
				P.push_back(compute_world_draw_position(state_ptr->controller[ci].pose));
				R.push_back(draw_radius);
				C.push_back(draw_color);
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
			if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
				P.push_back(compute_world_draw_position(state_ptr->controller[ci].pose, vec3(0.0f)));
				R.push_back(0.01f);
				C.push_back(rgba(1.0f - ci, 0, float(ci), 0.5f));
				P.push_back(compute_world_draw_position(state_ptr->controller[ci].pose, vec3(0.0f, 0.0f, -0.5f)));
				R.push_back(0.1f);
				C.push_back(rgba(1.0f - ci, 0, float(ci), 0.5f));
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
			case 'M': draw_mode = draw_mode == DM_POINT ? DM_LINE : DM_POINT; on_set(&draw_mode); return true;
			}
			return false;
		}
		if (e.get_kind() == cgv::gui::EID_KEY) {
			auto& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			switch (vrke.get_key()) {
			case vr::VR_INPUT0 :
				if (vrke.get_action() == cgv::gui::KA_PRESS)
					start_drawing(vrke.get_controller_index(), vrke.get_state(), vrke.get_time(), draw_radius);
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
			bool d = v >= min_throttle;
			if (d) {
				if (drawing[ci]) {
					last_radius[ci] = min_radius + (max_radius - min_radius) * (v - min_throttle) / (1.0f - min_throttle);
					continue_drawing(ci, te.get_state(), te.get_time(), last_radius[ci]);
				}
				else
					start_drawing(ci, te.get_state(), te.get_time(), min_radius);
			}
			else if (drawing[ci])
				stop_drawing(ci, te.get_state(), te.get_time(), min_radius);
		}
		else if (e.get_kind() == cgv::gui::EID_POSE) {
			auto& pe = static_cast<cgv::gui::vr_pose_event&>(e);
			int ci = pe.get_trackable_index();
			if (ci >= 0 && ci < 2 && drawing[ci]) {
				continue_drawing(ci, pe.get_state(), pe.get_time());
				return true;
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
			add_member_control(this, "draw_mode", draw_mode, "dropdown", "enums='point,line'");
			add_member_control(this, "draw_radius", draw_radius, "value_slider", "min=0.001;max=0.2;step=0.00001;log=true;ticks=true");
			add_member_control(this, "draw_color", draw_color);
			add_member_control(this, "draw_distance", draw_distance, "value_slider", "min=0.01;max=0.5;log=true;step=0.00001;ticks=true");
			add_member_control(this, "creation_threshold", creation_threshold, "value_slider", "min=0.001;max=0.1;log=true;step=0.00001;ticks=true");
			add_member_control(this, "min_throttle", min_throttle, "value_slider", "min=0.01;max=0.5;log=true;step=0.00001;ticks=true");
			add_member_control(this, "min_radius", min_radius, "value_slider", "min=0.001;max=0.1;log=true;step=0.00001;ticks=true");
			add_member_control(this, "max_radius", max_radius, "value_slider", "min=0.1;max=2;log=true;step=0.00001;ticks=true");
			align("\b");
			end_tree_node(srs);
		}
	}
};


#include <cgv/base/register.h>
cgv::base::object_registration<vr_draw> vr_draw_reg("vr_draw");
