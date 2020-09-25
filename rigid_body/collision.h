#pragma once

#include <util.h>

struct rbody;

struct collision {
	vec3 position;
	vec3 normal;
	float separation;
	rbody* body1;
	rbody* body2;

	// calculated attributes
	float Pn, Pt;

	vec3 r1, r2;
	float normal_mass;
	float bias;
	float friction;
	vec3 tangent;
	float tangent_mass;
};
