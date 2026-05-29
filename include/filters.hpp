#pragma once 
#include <Eigen/Dense>
#include <functional>
#include <camera.hpp>

/* 
    variable filtering 

    plane's state: x = [x, y, z, vx, vy, vz]  in ENU, modeled as a point

    state transition (dt):
    x_n+1 = [x + vx*dt, y + vy*dt, z + vz*dt, vx, vy, vz]

    measurement:
    [u, v] = CameraModel * R * [x, y, z]  (transformCoords())

    */

using Eigen::Matrix;
using Eigen::Vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

template<int stateDim> 
class EKF {
    using StateVector = Vector<double, stateDim>;
    using MatrixSS = Matrix<double, stateDim, stateDim>;

    // state 
    StateVector x_;
    MatrixSS P_;
    // predict
    MatrixSS F, Ft, Q;
    MatrixSS I = MatrixSS::Identity(); 
    
    double K;

public:
    EKF(const MatrixSS& F0, const MatrixSS& Q0, const StateVector& x0, const MatrixSS& P0);

    void predict();

    template <typename MM, int measDim>
    void update(MM& model, const Vector<double, measDim>& z);

    Vector<double, stateDim> getState();
};


template<int measDim, int stateDim> 
struct MeasurementModel {
    using MatrixMS = Matrix<double, measDim, stateDim>;
    using MatrixSM = Matrix<double, stateDim, measDim>;
    using MatrixMM = Matrix<double, measDim, measDim>;
    using MatrixSS = Matrix<double, stateDim, stateDim>;    // TODO: lookup traits 

    MatrixMS H; MatrixSM Ht;
    MatrixMM R, Q;

    MeasurementModel(const MatrixMS& H0, const MatrixMM& Q0, const MatrixMM& R0):  H(H0), Ht(H0.transpose()), R(R0), Q(Q0) {}

    virtual Matrix<double, stateDim, measDim>  gain(const Matrix<double, stateDim, stateDim>& P) const
    {
        MatrixMM inv = (H * P * Ht + R).inverse();
        MatrixSM K = P * Ht * inv;
        return K;
    }

    virtual Vector<double, measDim> measure(const Vector<double, stateDim>& x) const = 0; //h()

    virtual ~MeasurementModel() = default;
};

namespace plane_spotter {

    // define plane-spotter specific models 

template<int measDim, int stateDim> 
struct ADSBModel: public MeasurementModel<measDim, stateDim> {
    using Base = MeasurementModel<measDim, stateDim>;
    using typename Base::MatrixMS;
    using typename Base::MatrixMM;

    ADSBModel(const MatrixMS& H0, const MatrixMM& Q0, const MatrixMM& R0): Base(H0, Q0, R0) {}
    void linearize(Vector<double, stateDim>& x) {}
    Vector<double, measDim> measure(const Vector<double, stateDim>& x) const override; 
};

template<int measDim, int stateDim> 
struct CVModel: public MeasurementModel<measDim, stateDim> {
    using Base = MeasurementModel<measDim, stateDim>;
    using typename Base::MatrixMS;
    using typename Base::MatrixMM;

    Camera* cam;
    Matrix<double, 3, 6> Hconst; 
    
    CVModel(Camera* cam_params, const Matrix<double, 3, 6>& H0, const MatrixMM& Q0, const MatrixMM& R0): cam(cam_params), Base(MatrixMS::Zero(), Q0, R0), Hconst(H0) {}
    void linearize(Vector<double, stateDim>& x);
    Vector<double, measDim> measure(const Vector<double, stateDim>& x) const override;
};

} // namespace plane_spotter


// -------- EKF --------

template <int stateDim>
EKF<stateDim>::EKF(const MatrixSS& F0, const MatrixSS& Q0,
                   const StateVector& x0, const MatrixSS& P0)
    : F(F0), Q(Q0), x_(x0), P_(P0)
{
    Ft = F.transpose();
}

template <int stateDim>
void EKF<stateDim>::predict()
{
    x_ = F * x_;
    P_ = F * P_ * Ft + Q;
}

template <int stateDim>
Vector<double, stateDim> EKF<stateDim>::getState()
{
    return x_;
}

template <int stateDim>
template <typename MM, int measDim>
inline void EKF<stateDim>::update(MM& model, const Vector<double, measDim>& z)
{
    model.linearize(x_);
    auto K = model.gain(P_);
    x_ = x_ + K * (z - model.measure(x_));
    P_ = (I - K * model.H) * P_;
}


// -------- plane_spotter models --------

namespace plane_spotter {

template <int measDim, int stateDim>
inline Vector<double, measDim> ADSBModel<measDim, stateDim>::measure(const Vector<double, stateDim>& x) const
{
    return this->H * x;
}

template <int measDim, int stateDim>
inline Vector<double, measDim> CVModel<measDim, stateDim>::measure(const Vector<double, stateDim>& x) const
{
    Vector<double, 3> rotated = this->Hconst * x;
    cv::Point3d cv_rotated(rotated[0], rotated[1], rotated[2]);
    cv::Point2i cv_pix = cam->projectWorldToPixel(cv_rotated);

    Vector<double, measDim> pixel;
    pixel[0] = cv_pix.x; pixel[1] = cv_pix.y;
    return pixel;
}

template <int measDim, int stateDim>
void CVModel<measDim, stateDim>::linearize(Vector<double, stateDim>& x)
{
    Vector<double, 3> rotated = Hconst * x;

    Matrix<double, 2, 3> projJacobian;
    projJacobian << cam->xFov / rotated[2],  0.0,           -cam->xFov * rotated[0] / (rotated[2] * rotated[2]),
                             0.0,   cam->yFov / rotated[2], -cam->yFov * rotated[1] / (rotated[2] * rotated[2]);

    this->H  = projJacobian * Hconst;
    this->Ht = this->H.transpose();
}

} // namespace plane_spotter