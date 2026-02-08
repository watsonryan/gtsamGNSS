/**
 *  @file   GNSSDCSFactor.h
 *  @author Watson
 *  @brief  Header file for GNSS Factor with multimodal uncert. model
 **/

#pragma once
#include <atomic>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/gnssNavigation/GnssTypes.h>





namespace gtsam {


class GTSAM_EXPORT GNSSDCSFactor : public NonlinearFactor {

private:

typedef gtsam::NonlinearFactor Base;
typedef GNSSDCSFactor This;

mutable std::atomic<int> iter_count_{0};
Key k1_,k2_;
Point3 satXYZ_;
Point3 nomXYZ_;
nonBiasStates h_;
Vector2 measured_, k_;
Eigen::MatrixXd model_;

public:

typedef std::shared_ptr<GNSSDCSFactor> shared_ptr;

GNSSDCSFactor() : measured_() {
        h_=Matrix(2,5);
}

GNSSDCSFactor(Key deltaStates, Key bias, const Vector2 measurement,
              const Point3 satXYZ, const Point3 nomXYZ, const Eigen::MatrixXd& model, const Vector2 k) :
        Base(KeyVector{deltaStates, bias}), k1_(deltaStates), k2_(bias), k_(k), measured_(measurement), satXYZ_(satXYZ), nomXYZ_(nomXYZ), model_(model) {
}

virtual ~GNSSDCSFactor() {
}

/** print */
virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
        const This *t = dynamic_cast<const This*>(&f);

        if (t && Base::equals(f)) { return k1_ == t->k1_ && k2_ == t->k2_; }
        else{ return false; }
}

virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return gtsam::NonlinearFactor::shared_ptr(
            new GNSSDCSFactor(k1_, k2_, measured_, satXYZ_, nomXYZ_, model_, k_));
}

virtual double error(const gtsam::Values& x) const {
        return whitenedError(x).squaredNorm();
}

Vector unwhitenedError(const gtsam::Values& x,
                       std::vector<Matrix>* H = nullptr) const;

Vector whitenedError(const gtsam::Values& x,
                     std::vector<Matrix>* H = nullptr) const;

virtual Vector residual(const gtsam::Values& x) const {
        const Vector b = unwhitenedError(x);
        return b;
}

virtual size_t dim() const {
        return 5;
}

std::size_t size() const {
        return 2;
}

bool active(const gtsam::Values& x) const {
        return true;
}



/* ************************************************************************* */
/**
 * Linearize a non-linearFactorN to get a gtsam::GaussianFactor,
 * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
 * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
 */
/* This version of linearize recalculates the noise model each time */
virtual gtsam::GaussianFactor::shared_ptr linearize(
        const gtsam::Values& x) const {

        const int iter_count = iter_count_.fetch_add(1, std::memory_order_relaxed) + 1;

        if (!active(x))
                return gtsam::GaussianFactor::shared_ptr();

        // Call evaluate error to get Jacobians and RHS vector b
        std::vector<Matrix> A(this->size());
        Vector b = unwhitenedError(x, &A);

        // Fill in terms, needed to create JacobianFactor below
        std::vector<std::pair<Key, Matrix> > terms(size());
        for (size_t j = 0; j < size(); ++j) {
                terms[j].first = keys()[j];
                terms[j].second.swap(A[j]);
        }

        // DCS Base scaling of cov.
        double v_range, v_phase;

        // check range residaul
        if (std::pow(b(0),2) < k_(0)  || iter_count < 2)
        {
                v_range = model_(0,0);
        }
        else
        {
                double scale = (4*std::pow(k_(0),2)) / (std::pow( std::pow(b(0),2) + k_(0),2));
                v_range = model_(0,0)/scale;
        }

        // check phase residaul
        if (std::pow(b(1),2) < k_(1) || iter_count < 2)
        {
                v_phase = model_(1,1);
        }
        else
        {
                double scale = (4*std::pow(k_(1),2)) / (std::pow( std::pow(b(1),2) + k_(1),2));
                v_phase = model_(1,1)/scale;
        }

        auto jacobianFactor = GaussianFactor::shared_ptr( new JacobianFactor(terms, -b, noiseModel::Diagonal::Variances((gtsam::Vector(2) << v_range, v_phase).finished()) ));

        return jacobianFactor;
}


private:

}; // GNSSDCSFactor Factor
} // namespace
