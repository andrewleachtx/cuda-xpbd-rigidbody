#pragma once
#include <Eigen/Dense>
#include <vector>
#include "Shape.h"

typedef Eigen::Matrix<float, 3, 1> vec3;
typedef Eigen::Matrix<float, 7, 1> vec7;
typedef Eigen::Matrix<float, 12, 1> vec12;
typedef Eigen::Matrix<float, 3, 3> mat3;

/*
    Typically, the body class would parent BodyAffine and BodyRigid, because they share similar "properties"

    However, they don't really share the same principles; i.e. one has 7 dof, the other 12.

    To this extent, I will treat them separately, and use dynamic dispatch for a superclass "Body" to handle a
    variate number of bodies.

    To start, there is much overlap.
*/

class Body {
    public:
        virtual ~Body() = default;
};

class BodyAffine : public Body {
    public:
        const static size_t num_dof = 12;

        vec12 xdotInit;
        vec12 x;
        vec12 x0;
        vec12 dxJacobi;
        vec12 dphiJacobi;

        bool collide;
        float mu;

        size_t index;
        size_t layer;
        std::vector<size_t> neighbors;
        
        Shape shape;
        float density;
        mat3 Wa;
        float Wp;

        vec3 color;
        float axisSize;
};

class BodyRigid : public Body {
    const static size_t num_dof = 7;
    
    vec7 xdotInit;
    vec7 x;
    vec7 x0;
    vec7 dxJacobi;
    vec7 dphiJacobi;

    bool collide;
    float mu;

    size_t index;
    size_t layer;
    std::vector<size_t> neighbors;

    Shape shape;
    float density;
    vec3 Mr;
    float Mp;

    vec3 color;
    float axisSize;
}