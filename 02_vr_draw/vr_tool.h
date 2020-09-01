#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/base/find_action.h>
#include <cg_vr/vr_server.h>
#include "vr_scene.h"

class vr_tool : 
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
protected:
	// keep pointer to other objects
	vr_scene* scene_ptr;
	vr::vr_kit* kit_ptr;
	vr_view_interactor* vr_view_ptr;

	void on_device_change(void* device_handle, bool connect)
	{
		if (connect)
			kit_ptr = vr::get_vr_kit(device_handle);
		else
			kit_ptr = 0;
	}
	// interaction parameters
	bool tool_is_active;
	//
	vr_scene* find_scene(size_t scene_idx = 0) const
	{
		cgv::base::base* base_ptr = const_cast<cgv::base::base*>(static_cast<const cgv::base::base*>(this));
		std::vector<vr_scene*> scenes;
		cgv::base::find_interface<vr_scene>(base_ptr, scenes);
		if (scenes.empty() || scene_idx > scenes.size())
			return 0;
		return scenes[scene_idx];
	}
public:
	vr_tool(const std::string& _name = "vr_tool") : cgv::base::node(_name)
	{
		tool_is_active = true;

		scene_ptr = 0;
		kit_ptr = 0;
		vr_view_ptr = 0;
	
		connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_tool::on_device_change);
	}
	void init_frame(cgv::render::context& ctx)
	{
		if (!scene_ptr)
			scene_ptr = find_scene();
		if (!vr_view_ptr) {
			auto view_ptr = find_view_as_node();
			if (view_ptr)
				vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		}
	}

//	std::string get_default_options() const {
//		return "parents=none;views=none";
//	}

};
