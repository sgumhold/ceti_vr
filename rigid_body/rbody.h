#pragma once

#include <util.h>
#include <collision.h>

struct rbody {
	enum test_feature {
		FACE_A,
		FACE_B,
		EDGE
	};

	struct test_case {
		vec3 axis;
		unsigned index0;
		unsigned index1;
		test_feature feature;
	};

	struct edge {
		vec3 a;
		vec3 b;

		vec3 get_center() {
			return 0.5f * (a + b);
		}

		std::vector<vec3> clostest_points(edge& other) {
			std::vector<vec3> result;

			vec3 u = this->b - this->a;
			vec3 v = other.b - other.a;

			vec3 w0 = this->a - other.a;

			float a = dot(u, u);
			float b = dot(u, v);
			float c = dot(v, v);
			float d = dot(u, w0);
			float e = dot(v, w0);

			float denom = a * c - b * b;

			float sc = 0.0f;
			float sn = 0.0f;
			float sd = denom;

			float tc = 0.0f;
			float tn = 0.0f;
			float td = denom;

			if(fabsf(denom) < std::numeric_limits<float>::epsilon()) {
				sn = 0.0f;
				sd = 1.0f;
				tn = e;
				td = c;
			} else {
				sn = b * e - c * d;
				tn = a * e - b * d;

				if(sn < 0.0f) {
					sn = 0.0f;
					tn = e;
					td = c;
				} else if(sn > sd) {
					sn = sd;
					tn = e + b;
					td = c;
				}
			}

			if(tn < 0.0f) {
				tn = 0.0f;

				if(-d < 0.0f) {
					sn = 0.0f;
				} else if(-d > a) {
					sn = sd;
				} else {
					sn = -d;
					sd = a;
				}
			} else if(tn > td) {
				tn = td;

				if((-d + b) < 0.0) {
					sn = 0.0f;
				} else if((-d + b) > a) {
					sn = sd;
				} else {
					sn = -d + b;
					sd = a;
				}
			}

			sc = fabsf(sn) < std::numeric_limits<float>::epsilon() ? 0.0f : sn / sd;
			tc = fabsf(tn) < std::numeric_limits<float>::epsilon() ? 0.0f : tn / td;

			result.push_back(this->a + sc * u);
			result.push_back(other.a + tc * v);

			return result;
		}
	};

	struct face {
		vec3 origin;
		vec3 edge1;
		vec3 edge2;
		vec3 normal;
	};

	box3 box;
	vec3 position;
	quat orientation;
	rgb color;
	vec3 velocity;
	vec3 angular_velocity;
	float friction;
	float inv_mass;
	mat3 inv_inertia;
	vec3 force;
	vec3 torque;

	rbody() : rbody(box3(-1.0f, 1.0f)) {}

	rbody(box3 b) {
		box = b;
		position = vec3(0.0f);
		orientation = quat(1.0f, 0.0f, 0.0f, 0.0f);
		orientation.normalize();
		velocity = vec3(0.0f);
		angular_velocity = vec3(0.0f);
		friction = 0.5f;

		force = vec3(0.0f);
		torque = vec3(0.0f);

		// use approximate density of steel
		calculate_mass_and_inertia(7.87f);
	}

	vec3 get_center() {
		return box.get_center() + position;
	}

	void set_orientation(vec3 axis, float degrees) {

		orientation = quat(axis, cgv::math::deg2rad(degrees));
		orientation.normalize();
	}

	void make_static() {

		// set inverse mass and inertia to zero to prevent movement of this body
		inv_mass = 0.0f;
		inv_inertia = mat3(0.0f);
	}

	void calculate_mass_and_inertia(float density) {

		// calculate inverse mass and inertia using volume and density
		vec3 size = box.get_extent();
		float volume = size[0] * size[1] * size[2];
		volume *= 1000000.0f;

		float mass = volume * density; // volume in cm³ times density g/cm³
		mass /= 1000.0f; // use mass in kg

		inv_mass = 1.0f / mass;

		inv_inertia = mat3(0.0f);

		float ix = mass / 12.0f * (size[1] * size[1] + size[2] * size[2]);
		float iy = mass / 12.0f * (size[0] * size[0] + size[2] * size[2]);
		float iz = mass / 12.0f * (size[0] * size[0] + size[1] * size[1]);

		inv_inertia(0, 0) = 1.0f / ix;
		inv_inertia(1, 1) = 1.0f / iy;
		inv_inertia(2, 2) = 1.0f / iz;
	}

	std::vector<float> project_corners(vec3 axis) {
		std::vector<float> projected(8);
		axis.normalize();

		for(unsigned i = 0; i < 8; ++i) {
			vec3 corner = box.get_corner(i);

			orientation.rotate(corner);
			corner += position;

			float length = dot(corner, axis);

			projected[i] = length;
		}

		return projected;
	}

	std::vector<edge> get_edges(unsigned axis) {
		std::vector<edge> edges(4);

		vec3 min = box.get_min_pnt();
		vec3 max = box.get_max_pnt();

		switch(axis) {
		case 0:
			edges[0] = { vec3(min[0], min[1], min[2]), vec3(max[0], min[1], min[2]) };
			edges[1] = { vec3(min[0], min[1], max[2]), vec3(max[0], min[1], max[2]) };
			edges[2] = { vec3(min[0], max[1], min[2]), vec3(max[0], max[1], min[2]) };
			edges[3] = { vec3(min[0], max[1], max[2]), vec3(max[0], max[1], max[2]) };
			break;
		case 1:
			edges[0] = { vec3(min[0], min[1], min[2]), vec3(min[0], max[1], min[2]) };
			edges[1] = { vec3(min[0], min[1], max[2]), vec3(min[0], max[1], max[2]) };
			edges[2] = { vec3(max[0], min[1], min[2]), vec3(max[0], max[1], min[2]) };
			edges[3] = { vec3(max[0], min[1], max[2]), vec3(max[0], max[1], max[2]) };
			break;
		case 2:
			edges[0] = { vec3(min[0], min[1], min[2]), vec3(min[0], min[1], max[2]) };
			edges[1] = { vec3(min[0], max[1], min[2]), vec3(min[0], max[1], max[2]) };
			edges[2] = { vec3(max[0], min[1], min[2]), vec3(max[0], min[1], max[2]) };
			edges[3] = { vec3(max[0], max[1], min[2]), vec3(max[0], max[1], max[2]) };
			break;
		}

		for(unsigned i = 0; i < 4; ++i) {
			vec3 a = edges[i].a;
			vec3 b = edges[i].b;

			orientation.rotate(a);
			edges[i].a = a + position;

			orientation.rotate(b);
			edges[i].b = b + position;
		}

		return edges;
	}

	std::vector<face> get_faces(unsigned axis) {
		std::vector<face> faces(2);

		vec3 min = box.get_min_pnt();
		vec3 max = box.get_max_pnt();

		vec3 extend = max - min;

		switch(axis) {
		case 0:
			faces[0] = { min, vec3(0.0f, 1.0f, 0.0f) * extend, vec3(0.0f, 0.0f, 1.0f) * extend, vec3(-1.0f, 0.0f, 0.0f) };
			faces[1] = { max, vec3(0.0f, -1.0f, 0.0f) * extend, vec3(0.0f, 0.0f, -1.0f) * extend, vec3(1.0f, 0.0f, 0.0f) };
			break;
		case 1:
			faces[0] = { min, vec3(1.0f, 0.0f, 0.0f) * extend, vec3(0.0f, 0.0f, 1.0f) * extend, vec3(0.0f, -1.0f, 0.0f) };
			faces[1] = { max, vec3(-1.0f, 0.0f, 0.0f) * extend, vec3(0.0f, 0.0f, -1.0f) * extend, vec3(0.0f, 1.0f, 0.0f) };
			break;
		case 2:
			faces[0] = { min, vec3(1.0f, 0.0f, 0.0f) * extend, vec3(0.0f, 1.0f, 0.0f) * extend, vec3(0.0f, 0.0f, -1.0f) };
			faces[1] = { max, vec3(-1.0f, 0.0f, 0.0f) * extend, vec3(0.0f, -1.0f, 0.0f) * extend, vec3(0.0f, 0.0f, 1.0f) };
			break;
		}

		for(unsigned i = 0; i < 2; ++i) {
			vec3 o = faces[i].origin;
			vec3 a = faces[i].edge1;
			vec3 b = faces[i].edge2;
			vec3 n = faces[i].normal;

			orientation.rotate(o);
			faces[i].origin = o + position;

			orientation.rotate(a);
			faces[i].edge1 = a;

			orientation.rotate(b);
			faces[i].edge2 = b;

			orientation.rotate(n);
			faces[i].normal = n;
		}

		return faces;
	}

	face get_incident_face(vec3 normal, rbody* other) {
		float max = -std::numeric_limits<float>::max();
		face f;

		std::vector<face> faces;

		for(unsigned i = 0; i < 3; ++i) {
			std::vector<face> axis_faces = other->get_faces(i);
			faces.push_back(axis_faces[0]);
			faces.push_back(axis_faces[1]);
		}

		for(unsigned i = 0; i < faces.size(); ++i) {
			float cost = dot(-normal, faces[i].normal);

			if(cost >= 0.0f && cost > max) {
				max = cost;
				f = faces[i];
			}
		}

		return f;
	}

	std::vector<test_case> get_test_cases(rbody& other) {
		// there are a total of 15 axes which need to be tested
		std::vector<test_case> axes;

		vec3 Ax = vec3(1.0f, 0.0f, 0.0f);
		vec3 Ay = vec3(0.0f, 1.0f, 0.0f);
		vec3 Az = vec3(0.0f, 0.0f, 1.0f);

		orientation.rotate(Ax);
		orientation.rotate(Ay);
		orientation.rotate(Az);

		axes.push_back({ Ax, 0, 0, FACE_A });
		axes.push_back({ Ay, 1, 1, FACE_A });
		axes.push_back({ Az, 2, 2, FACE_A });

		vec3 Bx = vec3(1.0f, 0.0f, 0.0f);
		vec3 By = vec3(0.0f, 1.0f, 0.0f);
		vec3 Bz = vec3(0.0f, 0.0f, 1.0f);

		other.orientation.rotate(Bx);
		other.orientation.rotate(By);
		other.orientation.rotate(Bz);

		axes.push_back({ Bx, 0, 0, FACE_B });
		axes.push_back({ By, 1, 1, FACE_B });
		axes.push_back({ Bz, 2, 2, FACE_B });

		axes.push_back({ cross(Ax, Bx), 0, 0, EDGE });
		axes.push_back({ cross(Ax, By), 0, 1, EDGE });
		axes.push_back({ cross(Ax, Bz), 0, 2, EDGE });

		axes.push_back({ cross(Ay, Bx), 1, 0, EDGE });
		axes.push_back({ cross(Ay, By), 1, 1, EDGE });
		axes.push_back({ cross(Ay, Bz), 1, 2, EDGE });

		axes.push_back({ cross(Az, Bx), 2, 0, EDGE });
		axes.push_back({ cross(Az, By), 2, 1, EDGE });
		axes.push_back({ cross(Az, Bz), 2, 2, EDGE });

		return axes;
	}

	std::vector<collision> get_collisions(rbody* other, test_case min_case, float separation) {
		std::vector<collision> collisions;

		vec3 center = 0.5f*(get_center() + other->get_center());

		if(min_case.feature == EDGE) {
			std::vector<edge> edges0 = get_edges(min_case.index0);
			std::vector<edge> edges1 = other->get_edges(min_case.index1);

			float min_dist0 = (center - edges0[0].get_center()).sqr_length();
			float min_dist1 = (center - edges1[0].get_center()).sqr_length();

			edge min0 = edges0[0];
			edge min1 = edges1[0];

			for(unsigned i = 1; i < 4; ++i) {
				float dist0 = (center - edges0[i].get_center()).sqr_length();
				float dist1 = (center - edges1[i].get_center()).sqr_length();

				if(dist0 < min_dist0) {
					min_dist0 = dist0;
					min0 = edges0[i];
				}

				if(dist1 < min_dist1) {
					min_dist1 = dist1;
					min1 = edges1[i];
				}
			}

			float distance = 0.0f;
			std::vector<vec3> points = min0.clostest_points(min1);

			if(points.size() == 2) {
				float dist = (points[0] - points[1]).length();

				/*collisions.push_back({ points[0], min_case.axis, -dist, *this, other });
				collisions.push_back({ points[1], min_case.axis, -dist, *this, other });*/
				collisions.push_back({ 0.5f * (points[0] + points[1]), min_case.axis, separation, this, other });
			}
		} else {
			rbody* a = this;
			rbody* b = other;

			if(min_case.feature == FACE_B) {
				std::swap(a, b);
			}

			// find most incident face to the collision normal on the other body
			std::vector<face> faces = a->get_faces(min_case.index0);

			face clip_face;

			vec3 b_to_center = normalize(center - b->position);

			if(dot(b_to_center, faces[0].normal) < dot(b_to_center, faces[1].normal)) {
				clip_face = faces[0];
			} else {
				clip_face = faces[1];
			}

			face incident_face = a->get_incident_face(clip_face.normal, b);

			if(dot(faces[1].normal, clip_face.normal) > 0.9f) {
				clip_face = faces[1];
			}

			std::vector<vec3> corners;

			corners.push_back(incident_face.origin);
			corners.push_back(incident_face.origin + incident_face.edge1);
			corners.push_back(incident_face.origin + incident_face.edge2);
			corners.push_back(incident_face.origin + incident_face.edge1 + incident_face.edge2);

			// project corners of incident_face onto clip_face
			for(int i = 0; i < int(corners.size()); ++i) {
				float p_dot_n = dot(corners[i] - clip_face.origin, clip_face.normal);

				vec3 projected = corners[i] - p_dot_n * clip_face.normal;

				if(p_dot_n < 0.0f) {
					vec3 ea = clip_face.edge1;
					float eal = static_cast<float>(ea.normalize());

					vec3 eb = clip_face.edge2;
					float ebl = static_cast<float>(eb.normalize());

					float proj_dot_ea = dot(projected - clip_face.origin, ea);

					if(proj_dot_ea < 0.0f)
						projected -= proj_dot_ea * ea;
					else if(proj_dot_ea > eal) {
						projected -= (proj_dot_ea - eal) * ea;
					}

					float proj_dot_eb = dot(projected - clip_face.origin, eb);

					if(proj_dot_eb < 0.0f)
						projected -= proj_dot_eb * eb;
					else if(proj_dot_eb > ebl) {
						projected -= (proj_dot_eb - ebl) * eb;
					}

					collisions.push_back({ projected, clip_face.normal, p_dot_n, a, b });
				}
			}
		}

		return collisions;
	}
};
