#include "../../include/apbd/Body.h"
#include "se3/lib.h"

/*
    NOTES:

    1)
        eye(n) = I --> identity matrix of size n x n
    
    2)
        function name(this, param1) { ... } --> void
        function VAR = name(this, param1) { ... } --> returns VAR

    3)
        broadphase, narrowphase are collision detection as methods in Shape

        cdata = collision data, in MATLAB it is a struct

    4)
        draw can return std::pair<F, V> as a method of again, Shape
        
        F = faces, V = vertices
*/

//////////////////////////////////////////////////////////////////////// Body CLASS ////////////////////////////////////////////////////////////////////////

size_t Body::indexCount = 0;

Body::Body() : collide(false), mu(0.0f), layer(99), density(0.0f) {
    index = ++indexCount;
    neighbors = std::vector<size_t>();

    shape = Shape();
    color << 1.0f, 0.0f, 0.0f;
    axisSize = 1.0f;
}

Body::Body(bool collide, float mu, Shape shape, float density) {
    this->collide = collide;
    this->mu = mu;
    this->shape = shape;
    this->density = density;
    this->layer = 99;
    this->index = ++indexCount;
    this->neighbors = std::vector<size_t>();

    // While these could be parametrized, I assume it will only end up more troublesome having to add them for each call.
    color << 1.0f, 0.0f, 0.0f;
    axisSize = 1.0f;
}

//////////////////////////////////////////////////////////////////////// Body::Affine SUBCLASS ////////////////////////////////////////////////////////////////////////
BodyAffine::BodyAffine(Shape shape, float density) : Body() {
    // BODY ATTRIBUTES //
    this->shape = shape;
    this->density = density;

    // AFFINE UNIQUE ATTRIBUTES //
    Wa = vec3::Zero();
    Wp = 0.0f;

    xInit.setZero();
    x.setZero();
    x0.setZero();
    dxJacobi.setZero();
    dphiJacobi.setZero();
}

void BodyAffine::setInitTransform(const Eigen::Matrix4f& E) {
    xInit.block<9, 1>(0, 0) = Eigen::Map<const Eigen::Matrix<float, 9, 1>>(E.block<3, 3>(0, 0).data()).transpose();
    xInit.block<3, 1>(9, 0) = E.block<3, 1>(0, 3);
}

/* TODO */
void BodyAffine::computeInitTransform() {
    /*
        Einit = eye(4);
        Einit(1:3,1:3) = reshape(this.xInit(1:9),3,3)';
        Einit(1:3,4) = this.xInit(10:12);
    */

    return;
}

/* TODO */
void BodyAffine::setInitVelocity(const Eigen::Matrix<float, 12, 1>& phi) {
    /*
        E = this.computeInitTransform();
        Edot = E*se3.brac(phi);
        this.xdotInit(1:9) = reshape(Edot(1:3,1:3)',9,1);
        this.xdotInit(10:12) = Edot(1:3,4);
    */

    return;
}

/* TODO */
Eigen::Matrix<float, 4, 4> BodyAffine::computeTransform() {
    auto E = Eigen::Matrix<float, 4, 4>::Identity();

    /*
        E(1:3,1:3) = reshape(this.x(1:9), 3, 3)';
        E(1:3,4) = this.x(10:12);
    */

   return;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyAffine::transformPoint(const Eigen::Matrix<float, 3, 1>& xl) {
    Eigen::Matrix<float, 3, 1> xw;

    /*
        xw = zeros(3,1);
        xw(1) = this.x(1:3)'*xl + this.x(10);
        xw(2) = this.x(4:6)'*xl + this.x(11);
        xw(3) = this.x(7:9)'*xl + this.x(12);
    */

    return xw;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyAffine::transformVector(const Eigen::Matrix<float, 3, 1>& vl) {
    Eigen::Matrix<float, 3, 1> vw;

    /*
        vw = zeros(3,1);
        vw(1) = this.x(1:3)'*vl;
        vw(2) = this.x(4:6)'*vl;
        vw(3) = this.x(7:9)'*vl;
    */

    return vw;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyAffine::invTransformPoint(const Eigen::Matrix<float, 3, 1>& xw) {
    Eigen::Matrix<float, 3, 1> xl;

    /*
        A = reshape(this.x(1:9),3,3)';
        p = x(10:12);
        xl = A\(xw - p);
    */

    return xl;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyAffine::invTransformVector(const Eigen::Matrix<float, 3, 1>& vw) {
    Eigen::Matrix<float, 3, 1> vl;

    /*
        A = reshape(this.x(1:9),3,3)';
        vl = A\vw;
    */

    return vl;
}

/* TODO */
void BodyAffine::computeInertiaConst() {
    /*
        I = this.shape.computeInertia(this.density);
        this.Wp = 1/I(4);
        R2A = 0.5*[
            -1  1  1
             1 -1  1
             1  1 -1
        ];
        this.Wa = 1./(R2A*I(1:3));
    */

    return;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyAffine::computePointVel(const Eigen::Matrix<float, 3, 1>& xl, float k, float ks, float hs) {
    Eigen::Matrix<float, 3, 1> xdot;

    /*
        xdot = this.computeVelocity(k,ks,hs);
        J = apbd.BodyAffine.jacobian(xl);
        v = J*xdot;
    */

    return xdot;
}

/* TODO */
void BodyAffine::stepBDF1(float k, float ks, float hs, const Eigen::Matrix<float, 3, 1>& grav) {
    /*
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
    */

    return;
}

/* TODO */
std::pair<float, float> BodyAffine::computeEnergies(float k, float ks, float hs, const Eigen::Matrix<float, 3, 1>& grav) {
    float T, V;

    /*
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
    */

    return std::make_pair(T, V);
}

/* TODO */
bool BodyAffine::broadphaseGround(const Eigen::Matrix4f& Eg) {
    bool flag;

    /*
        E = this.computeTransform();
        flag = this.shape.broadphaseGround(E,Eg);
    */

    return flag;
}

/* TODO */
void BodyAffine::narrowphaseGround(const Eigen::Matrix4f& Eg) {
    /*
        E = this.computeTransform();
        cdata = this.shape.narrowphaseGround(E,Eg);
    */

    return;
}

/* TODO */
bool BodyAffine::broadphaseRigid(const BodyAffine& that) {
    bool flag;

    /*
        E1 = this.computeTransform();
        E2 = that.computeTransform();
        flag = this.shape.broadphaseShape(E1,that.shape,E2); % bad syntax
    */

    return flag;
}

/* TODO */
void BodyAffine::narrowphaseRigid(const BodyAffine& that) {
    /*
        E1 = this.computeTransform();
        E2 = that.computeTransform();
        cdata = this.shape.narrowphaseShape(E1,that.shape,E2); % bad syntax
    */

    return;
}

/* TODO */
void BodyAffine::draw() {
    /*
        E = this.computeTransform();
        [F,V] = this.shape.draw(E,this.color,this.axisSize);
    */

    return;
}