#pragma once
#include <Eigen/Dense>

/*
    If shape is the superclass, then classes such as "ShapeCuboid" can inherit from it
*/

/*

classdef Shape < handle
	%%
	properties
	end

	%%
	methods
		%%
		function this = Shape()
		end
	end
	
	%%
	methods (Abstract)
		%%
		I = computeInertia(this)

		%%
		flag = broadphaseGround(this,E,Eg)

		%%
		cdata = narrowphaseGround(this,E,Eg)

		%%
		flag = broadphaseShape(this,Ethis,that,Ethat)

		%%
		cdata = narrowphaseShape(this,Ethis,that,Ethat)

		%%
		[F,V] = draw(this,E,color,axisSize)
	end
end

*/

class Shape {
    public:

        virtual ~Shape() = default;
        Shape();
};

class ShapeCuboid : Shape {
    public:
        Eigen::Vector3f sides;

    ShapeCuboid(const Eigen::Vector3f& sides);
    
    void computeInertia(float density);
    void broadphaseGround(const Eigen::Matrix4f& E, const Eigen::Matrix4f& Eg);
    void narrowphaseGround(const Eigen::Matrix4f& E, const Eigen::Matrix4f& Eg);
    void broadphaseShape(const Eigen::Matrix4f& E1, const ShapeCuboid& that, const Eigen::Matrix4f& E2);
    void narrowphaseShape(const Eigen::Matrix4f& E1, const ShapeCuboid& that, const Eigen::Matrix4f& E2);
    void raycast(const Eigen::Vector3f& x, const Eigen::Vector3f& n);
    void draw(const Eigen::Matrix4f& E, const Eigen::Vector3f& color, float axisSize);

    static void smits_mul(const Eigen::Vector3f& rx, const Eigen::Vector3f& rv, const Eigen::Vector3f& bmin, const Eigen::Vector3f& bmax);
    static void make_ray(float x, float y, float z, float i, float j, float k);
    static void make_aabox(float x0, float y0, float z0, float x1, float y1, float z1);
};