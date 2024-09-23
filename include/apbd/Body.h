#pragma once
#include <Eigen/Dense>
#include <vector>
#include "Shape.h"

// All these special characters aren't good for my carpal tunnel
typedef Eigen::Matrix<float, 3, 1> vec3;
typedef Eigen::Matrix<float, 7, 1> vec7;
typedef Eigen::Matrix<float, 12, 1> vec12;
typedef Eigen::Matrix<float, 3, 3> mat3;
typedef Eigen::Matrix<float, 4, 4> mat4;

/*
    Typically, the body class would parent BodyAffine and BodyRigid, because they share similar "properties"

    However, they don't really share the same principles; i.e. one has 7 dof, the other 12.

    To this extent, I will treat them separately, and use dynamic dispatch for a superclass "Body" to handle a
    variate number of bodies.

    To start, there is much overlap.
*/

class Body {
    public:
        bool collide;
        float mu;

        // I believe index is the current count of bodies in the world, we should use a static ctr
        static size_t indexCount;

        size_t index;
        size_t layer;
        std::vector<size_t> neighbors;

        Shape shape;
        float density;

        Eigen::Matrix<float, 3, 1> color;
        float axisSize;

        virtual ~Body() = default;
        Body();
        Body(bool collide, float mu, Shape shape, float density);
};


/*

%%
		function E = computeTransform(this)
			E = eye(4);
			E(1:3,1:3) = reshape(this.x(1:9),3,3)';
            E(1:3,4) = this.x(10:12);
		end

		%%
		function xw = transformPoint(this,xl)
			xw = zeros(3,1);
			xw(1) = this.x(1:3)'*xl + this.x(10);
			xw(2) = this.x(4:6)'*xl + this.x(11);
			xw(3) = this.x(7:9)'*xl + this.x(12);
		end

		%%
		function vw = transformVector(this,vl)
			vw = zeros(3,1);
			vw(1) = this.x(1:3)'*vl;
			vw(2) = this.x(4:6)'*vl;
			vw(3) = this.x(7:9)'*vl;
		end

		%%
		function xl = invTransformPoint(this,xw)
			A = reshape(this.x(1:9),3,3)';
			p = x(10:12);
			xl = A\(xw - p);
		end

		%%
		function vl = invTransformVector(this,vw)
			A = reshape(this.x(1:9),3,3)';
			vl = A\vw;
		end
		
		%%
		function computeInertiaConst(this)
			% Computes inertia for the shape
			I = this.shape.computeInertia(this.density);
            this.Wp = 1/I(4);
			R2A = 0.5*[
				-1  1  1
				 1 -1  1
				 1  1 -1
				];
			this.Wa = 1./(R2A*I(1:3));
		end

		%%
		function v = computePointVel(this,xl,k,ks,hs)
			xdot = this.computeVelocity(k,ks,hs);
			J = apbd.BodyAffine.jacobian(xl);
			v = J*xdot;
		end
		
        %%
		function stepBDF1(this,k,ks,hs,grav)
			xdot = this.computeVelocity(k,ks,hs);
            this.x0 = this.x;
			axdot = xdot(1:3);
			aydot = xdot(4:6);
			azdot = xdot(7:9);
			pdot = xdot(10:12);
			w = this.Wp;
			W = this.Wa;
			f = zeros(3,1);
			t = zeros(9,1); % affine torque
			tx = t(1:3);
			ty = t(4:6);
			tz = t(7:9);
			f = f + grav/w; % Gravity
			% Integrate velocities
			axdot = axdot + hs*(W.*tx);
			aydot = aydot + hs*(W.*ty);
			azdot = azdot + hs*(W.*tz);
			pdot  = pdot  + hs*(w *f);
			% Integrate positions
			this.x(1:3) = this.x(1:3) + hs*axdot;
			this.x(4:6) = this.x(4:6) + hs*aydot;
			this.x(7:9) = this.x(7:9) + hs*azdot;
			this.x(10:12) = this.x(10:12) + hs*pdot;
		end

		%%
		function [T,V] = computeEnergies(this,k,ks,hs,grav)
			xdot = this.computeVelocity(k,ks,hs);
			axdot = xdot(1:3);
			aydot = xdot(4:6);
			azdot = xdot(7:9);
			pdot = xdot(10:12);
			p = this.x(10:12);
			m = 1/this.Wp;
			I = 1./this.Wa;
			% Energy
			T = 0.5*(axdot'*(I.*axdot) + aydot'*(I.*aydot) + azdot'*(I.*azdot) + pdot'*m*pdot);
			V = -m*grav'*p;
		end

		%%
		function flag = broadphaseGround(this,Eg)
			E = this.computeTransform();
			flag = this.shape.broadphaseGround(E,Eg);
		end

		%%
		function cdata = narrowphaseGround(this,Eg)
			E = this.computeTransform();
			cdata = this.shape.narrowphaseGround(E,Eg);
		end

		%%
		function flag = broadphaseRigid(this,that)
			E1 = this.computeTransform();
			E2 = that.computeTransform();
			flag = this.shape.broadphaseShape(E1,that.shape,E2); % bad syntax
		end

		%%
		function cdata = narrowphaseRigid(this,that)
			E1 = this.computeTransform();
			E2 = that.computeTransform();
			cdata = this.shape.narrowphaseShape(E1,that.shape,E2); % bad syntax
		end

		%%
		function [F,V] = draw(this)
			E = this.computeTransform();
			[F,V] = this.shape.draw(E,this.color,this.axisSize);
		end
	end

	methods (Static)
		%%
		function J = jacobian(xl)
			% Not used -- too expensive to form
			J = zeros(3,12);
			J(1,1:3) = xl';
			J(2,4:6) = xl';
			J(3,7:9) = xl';
			J(1:3,10:12) = eye(3);
		end

		%%
		function J = jacobianRot(xl)
			% Not used -- too expensive to form
			J = zeros(3,12);
			J(1,1:3) = xl';
			J(2,4:6) = xl';
			J(3,7:9) = xl';
		end

*/
class BodyAffine : public Body {
    public:
        const static size_t num_dof = 12;

        vec12 xInit;
        vec12 xdotInit;
        vec12 x;
        vec12 x0;
        vec12 dxJacobi;
        vec12 dphiJacobi;

        mat3 Wa;
        float Wp;

        virtual ~BodyAffine() = default;
        BodyAffine(Shape shape, float density);
        void setInitTransform(const Eigen::Matrix4f& E);
        void computeInitTransform(); /* TODO */
        void setInitVelocity(const Eigen::Matrix<float, 12, 1>& phi); /* TODO */
        Eigen::Matrix<float, 4, 4> computeTransform(); 
        Eigen::Matrix<float, 3, 1> transformPoint(const Eigen::Matrix<float, 3, 1>& xl);
        Eigen::Matrix<float, 3, 1> transformVector(const Eigen::Matrix<float, 3, 1>& vl);
        Eigen::Matrix<float, 3, 1> invTransformPoint(const Eigen::Matrix<float, 3, 1>& xw);
        Eigen::Matrix<float, 3, 1> invTransformVector(const Eigen::Matrix<float, 3, 1>& vw);

        void computeInertiaConst();
        Eigen::Matrix<float, 3, 1> computePointVel(const Eigen::Matrix<float, 3, 1>& xl, float k, float ks, float hs);

        void stepBDF1(float k, float ks, float hs, const Eigen::Matrix<float, 3, 1>& grav);
        std::pair<float, float> computeEnergies(float k, float ks, float hs, const Eigen::Matrix<float, 3, 1>& grav);

        bool broadphaseGround(const Eigen::Matrix4f& Eg);
        void narrowphaseGround(const Eigen::Matrix4f& Eg);
        bool broadphaseRigid(const BodyAffine& that);
        void narrowphaseRigid(const BodyAffine& that);

        void draw();

        static void jacobian(const Eigen::Matrix<float, 3, 1>& xl);
        static void jacobianRot(const Eigen::Matrix<float, 3, 1>& xl);
};

class BodyRigid : public Body {
    const static size_t num_dof = 7;

    vec7 xInit;  
    vec7 xdotInit;
    vec7 x;
    vec7 x0;
    vec7 dxJacobi;
    vec7 dphiJacobi;

    Eigen::Matrix<float, 3, 1> Mr;
    float Mp;
};