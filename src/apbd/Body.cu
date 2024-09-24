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

//////////////////////////////////////// Body::Affine SUBCLASS ////////////////////////////////////////
BodyAffine::BodyAffine(Shape shape, float density) : Body() {
    // BODY ATTRIBUTES //
    this->shape = shape;
    this->density = density;

    // AFFINE UNIQUE ATTRIBUTES //
    Wa = Eigen::Matrix<float, 3, 1>::Zero();
    Wp = 0.0f;

    xInit.setZero();
    x.setZero();
    x0.setZero();
    dxJacobi.setZero();
    dphiJacobi.setZero();
}

/* TODO */
void BodyAffine::setInitTransform(const Eigen::Matrix4f& E) {
    // this.xInit(1:9) = reshape(E(1:3,1:3)',9,1);
    // this.xInit(10:12) = E(1:3,4);

    return;
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

//////////////////////////////////////// Body::Rigid SUBCLASS ////////////////////////////////////////

/* TODO */
BodyRigid::BodyRigid(Shape shape, float density) : Body() {
    // BODY ATTRIBUTES //
    this->shape = shape;
    this->density = density;

    // RIGID UNIQUE ATTRIBUTES //
    Mr = Eigen::Matrix<float, 3, 1>::Zero();
    Mp = 0.0f;
    w = Eigen::Matrix<float, 3, 1>::Zero();
    v = Eigen::Matrix<float, 3, 1>::Zero();
    deltaBody2Worldp = Eigen::Matrix<float, 3, 1>::Zero();
    deltaBody2Worldq = Eigen::Matrix<float, 3, 1>::Zero();
    deltaBody2Worldq[4] = 1.0f; /* COULD BE SYNTACTICALLY WRONG, NOT SURE */
    deltaLinDt = Eigen::Matrix<float, 3, 1>::Zero();
    deltaAngDt = Eigen::Matrix<float, 3, 1>::Zero();

    xInit.setZero();
    x.setZero();
    x0.setZero();
    dxJacobi.setZero();
    dphiJacobi.setZero();
}

void BodyRigid::setInitTransform(const Eigen::Matrix4f& E) {
    xInit.block<9, 1>(0, 0) = Eigen::Map<const Eigen::Matrix<float, 9, 1>>(E.block<3, 3>(0, 0).data()).transpose();
    xInit.block<3, 1>(9, 0) = E.block<3, 1>(0, 3);
}

/* TODO */
Eigen::Matrix4f BodyRigid::computeInitTransform() {
    auto Einit = Eigen::Matrix4f::Identity();

    /*
        Einit(1:3,1:3) = se3.qToMat(this.xInit(1:4));
        Einit(1:3,4) = this.xInit(5:7);
    */

    return Einit;
}

/* TODO */
void BodyRigid::setInitVelocity(const Eigen::Matrix<float, 7, 1>& phi) {
    /*
        q = this.xInit(1:4);
        this.xdotInit(5:7) = se3.qRot(q,phi(4:6));
        this.xdotInit(1:4) = se3.wToQdot(q,phi(1:3));
        this.v = phi(4:6);
        this.w = phi(1:3);
    */

    return;
}

/* TODO */
void BodyRigid::updateStates(float hs) {
    /*
        q = this.x0(1:4);
        R = se3.qToMat(q);
        invsqrtI = R * diag(sqrt(1./this.Mr)) * R';
        angularMotionVel = invsqrtI * this.w;
        wNorm =  norm(angularMotionVel);
        if(wNorm>1e-9)
            halfWDt = 0.5 * wNorm * hs;
            dq = [angularMotionVel * sin(halfWDt)/ wNorm; 0];
            result = se3.qMul(dq, this.deltaBody2Worldq);
            result = result + this.deltaBody2Worldq * cos(halfWDt);
            this.deltaBody2Worldq = result / norm(result);
        end
        this.deltaBody2Worldp = this. deltaBody2Worldp + this.v * hs;

        this.deltaAngDt = this.deltaAngDt + this.w * hs;
        this.deltaLinDt = this.deltaLinDt + this.v * hs;

        this.x(1:4) = se3.qMul(this.deltaBody2Worldq, this.x0(1:4));
        this.x(5:7) = this.x0(5:7) + this.deltaBody2Worldp;
    */

    return;
}

/* TODO */
void BodyRigid::integrateStates() {
    /*
        q = this.x0(1:4);
        R = se3.qToMat(q);
        invsqrtI = R * diag(sqrt(1./this.Mr)) * R';
        this.w = invsqrtI * this.w;
        this.x(1:4) = se3.qMul(this.deltaBody2Worldq, this.x0(1:4));
        this.x(5:7) = this.x0(5:7) + this.deltaBody2Worldp;
        this.deltaBody2Worldp = zeros(3,1);
        this.deltaBody2Worldq = zeros(4,1);
        this.deltaBody2Worldq(4) = 1;
        
        this.deltaLinDt = zeros(3,1);
        this.deltaAngDt = zeros(3,1);

        %Clear contact information
        this.layer = 99;
        this.neighbors = [];
    */

    return;
}

/* TODO */
Eigen::Matrix4f BodyRigid::computeTransform() {
    auto E = Eigen::Matrix4f::Identity();

    /*
        E(1:3,1:3) = se3.qToMat(this.x(1:4));
        E(1:3,4) = this.x(5:7);
    */

    return E;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyRigid::transformPoint(const Eigen::Matrix<float, 3, 1>& xl) {
    Eigen::Matrix<float, 3, 1> xw;

    /*
        q = this.x(1:4);
        p = this.x(5:7);
        xw = se3.qRot(q,xl) + p;
    */

    return xw;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyRigid::transformPointPrev(const Eigen::Matrix<float, 3, 1>& xl) {
    Eigen::Matrix<float, 3, 1> xw;

    /*
        q = this.x0(1:4);
        p = this.x0(5:7);
        xw = se3.qRot(q,xl) + p;
    */

    return xw;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyRigid::transformVector(const Eigen::Matrix<float, 3, 1>& vl) {
    Eigen::Matrix<float, 3, 1> vw;

    /*
        q = this.x(1:4);
        vw = se3.qRot(q,vl);
    */

    return vw;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyRigid::invTransformPoint(const Eigen::Matrix<float, 3, 1>& xw) {
    Eigen::Matrix<float, 3, 1> xl;

    /*
        q = this.x(1:4);
        p = this.x(5:7);
        xl = se3.qRotInv(q,xw - p);
    */

    return xl;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyRigid::invTransformVector(const Eigen::Matrix<float, 3, 1>& vw) {
    Eigen::Matrix<float, 3, 1> vl;

    /*
        q = this.x(1:4);
        vl = se3.qRotInv(q,vw);
    */

    return vl;
}

/* TODO */
void BodyRigid::computeInertiaConst() {
    /*
        I = this.shape.computeInertia(this.density);
        this.Mr = I(1:3);
        this.Mp = I(4);
    */

    return;
}

/* TODO */
Eigen::Matrix<float, 3, 1> BodyRigid::computePointVel(const Eigen::Matrix<float, 3, 1>& xl) {
    Eigen::Matrix<float, 3, 1> v;

    /*
        v = this.computeVelocity(k,ks,hs);
        q = this.x(1:4);
        w = se3.qdotToW(q,qdot);
        rw = se3.qRot(q,xl);
        v = se3.cross(w,rw) + pdot;
    */

    return v;
}

/* TODO */
void BodyRigid::stepBDF1(float h, const Eigen::Matrix<float, 3, 1>& grav) {
    /*
        this.x0 = this.x;
        v = this.v; % pdot
        q = this.x(1:4);
        p = this.x(5:7);
        R = se3.qToMat(q);
        w = this.w; % angular velocity in body coords
        f = zeros(3,1); % translational force in world space
        t = zeros(3,1); % angular torque in body space
        m = this.Mp; % scalar mass
        I = R * diag(this.Mr) * R'; % inertia in world space;
        Iw = I*w; % angular momentum in body space
        f = f + m*grav; % Gravity
        t = t + se3.cross(Iw,w); % Coriolis
        % Integrate velocities
        w = w + h*(I\t);
        v = v + h*(m \f);
        %qdot = se3.wToQdot(q,w);
        % Integrate positions
        %q = q + hs*qdot;
        %p = p + hs*v;
        %q = q/norm(q);
        %this.x(1:4) = q;
        %this.x(5:7) = p;
        sqrtInertia = R * diag(sqrt(this.Mr)) * R';
        this.w = sqrtInertia * w;
        this.v = v;
    */

    return;
}

/* TODO */
std::pair<float, float> BodyRigid::computeEnergies(float k, float ks, float hs, const Eigen::Matrix<float, 3, 1>& grav) {
    float T, V;

    /*
        v = this.v; % pdot
        w = this.w; % angular velocity in body coords
        q = this.x(1:4);
        p = this.x(5:7);
        m = this.Mp; % scalar mass
        I = this.Mr; % inertia in body space
        Iw = I.*w; % angular momentum in body space
        % Energy
        T = 0.5*w'*Iw + 0.5*m*(v'*v);
        V = -m*grav'*p;
    */

    return std::make_pair(T, V);
}

/* TODO */
bool BodyRigid::broadphaseGround(const Eigen::Matrix4f& Eg) {
    bool flag;

    /*
        E = this.computeTransform();
        flag = this.shape.broadphaseGround(E,Eg);
    */

    return flag;
}

/* TODO */
void BodyRigid::narrowphaseGround(const Eigen::Matrix4f& Eg) {
    /*
        E = this.computeTransform();
        cdata = this.shape.narrowphaseGround(E,Eg);
    */

    return;
}

/* TODO */
bool BodyRigid::broadphaseRigid(const BodyRigid& that) {
    bool flag;

    /*
        E1 = this.computeTransform();
        E2 = that.computeTransform();
        flag = this.shape.broadphaseShape(E1,that.shape,E2); % bad syntax
    */

    return flag;
}

/* TODO */
void BodyRigid::narrowphaseRigid(const BodyRigid& that) {
    /*
        E1 = this.computeTransform();
        E2 = that.computeTransform();
        cdata = this.shape.narrowphaseShape(E1,that.shape,E2); % bad syntax
    */

    return;
}

/* TODO */
void BodyRigid::draw() {
    /*
        E = this.computeTransform();
        [F,V] = this.shape.draw(E,this.color,this.axisSize);
    */

    return;
}

/* TODO */
void BodyRigid::jacobian(const Eigen::Matrix<float, 3, 1>& xl, const Eigen::Matrix<float, 3, 1>& w) {
    /*
        wx = w(1);
        wy = w(2);
        wz = w(3);
        w0 = w(4);
        Jaw = 2*[
                 wx, -wy, -wz,  w0
                 wy,  wx, -w0, -wz
                 wz,  w0,  wx,  wy
                 wy,  wx,  w0,  wz
                -wx,  wy, -wz,  w0
                -w0,  wz,  wy, -wx
                 wz, -w0,  wx, -wy
                 w0,  wz,  wy,  wx
                -wx, -wy,  wz,  w0
                ];
        Jxa = apbd.BodyAffine.jacobian(xl);
        J = [Jxa(:,1:9)*Jaw, eye(3)];
    */

    return;
}