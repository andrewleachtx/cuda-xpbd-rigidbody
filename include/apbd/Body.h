#pragma once
#include <Eigen/Dense>
#include <vector>
#include "Shape.h"

/*
    Typically, the body class would parent BodyAffine and BodyRigid, because they share similar "properties"

    However, they don't really share the same principles; i.e. one has 7 dof, the other 12.

    To this extent, I will treat them separately, and use dynamic dispatch for a superclass "Body" to handle a
    variate number of bodies.

    To start, there is much overlap.

    Eigen::Matrix<type, rows, cols> i.e. vec3 = Eigen::Matrix<float, 3, 1>
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

class BodyAffine : public Body {
    public:
        const static size_t dof = 12;

        Eigen::Matrix<float, 12, 1> xInit;
        Eigen::Matrix<float, 12, 1> xdotInit;
        Eigen::Matrix<float, 12, 1> x;
        Eigen::Matrix<float, 12, 1> x0;
        Eigen::Matrix<float, 12, 1> dxJacobi;
        Eigen::Matrix<float, 12, 1> dphiJacobi;

        Eigen::Matrix<float, 3, 3> Wa;
        float Wp;

        virtual ~BodyAffine() = default;
        BodyAffine(Shape shape, float density);

        void setInitTransform(const Eigen::Matrix4f& E);
        void computeInitTransform();
        void setInitVelocity(const Eigen::Matrix<float, 12, 1>& phi);
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
    const static size_t dof = 7;

    Eigen::Matrix<float, 7, 1> xInit;  
    Eigen::Matrix<float, 7, 1> xdotInit;
    Eigen::Matrix<float, 7, 1> x;
    Eigen::Matrix<float, 7, 1> x0;
    Eigen::Matrix<float, 7, 1> dxJacobi;
    Eigen::Matrix<float, 7, 1> dphiJacobi;

    Eigen::Matrix<float, 3, 1> Mr;
    float Mp;
    Eigen::Matrix<float, 3, 1> w;
    Eigen::Matrix<float, 3, 1> v;
    Eigen::Matrix<float, 3, 1> deltaBody2Worldp;
    Eigen::Matrix<float, 4, 1> deltaBody2Worldq;
    Eigen::Matrix<float, 3, 1> deltaLinDt;
    Eigen::Matrix<float, 3, 1> deltaAngDt;

    virtual ~BodyRigid() = default;
    BodyRigid(Shape shape, float density);

    void setInitTransform(const Eigen::Matrix4f& E);
    Eigen::Matrix4f computeInitTransform();
    void setInitVelocity(const Eigen::Matrix<float, 7, 1>& phi);
    void updateStates(float hs);
    void integrateStates();
    Eigen::Matrix4f computeTransform();
    Eigen::Matrix<float, 3, 1> transformPoint(const Eigen::Matrix<float, 3, 1>& xl);
    Eigen::Matrix<float, 3, 1> transformPointPrev(const Eigen::Matrix<float, 3, 1>& xl);
    Eigen::Matrix<float, 3, 1> transformVector(const Eigen::Matrix<float, 3, 1>& vl);

    Eigen::Matrix<float, 3, 1> invTransformPoint(const Eigen::Matrix<float, 3, 1>& xw);
    Eigen::Matrix<float, 3, 1> invTransformVector(const Eigen::Matrix<float, 3, 1>& vw);

    void computeInertiaConst();
    Eigen::Matrix<float, 3, 1> computePointVel(const Eigen::Matrix<float, 3, 1>& xl);
    void stepBDF1(float h, const Eigen::Matrix<float, 3, 1>& grav);
    std::pair<float, float> computeEnergies(float k, float ks, float hs, const Eigen::Matrix<float, 3, 1>& grav);
    
    bool broadphaseGround(const Eigen::Matrix4f& Eg);
    void narrowphaseGround(const Eigen::Matrix4f& Eg);
    bool broadphaseRigid(const BodyRigid& that);
    void narrowphaseRigid(const BodyRigid& that);

    void draw();

    static void jacobian(const Eigen::Matrix<float, 3, 1>& xl, const Eigen::Matrix<float, 3, 1>& w);
};