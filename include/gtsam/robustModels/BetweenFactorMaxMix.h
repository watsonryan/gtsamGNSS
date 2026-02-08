/*
 *  @file   betweenFactorMaxMix.h
 *  @author Watson
 *  @brief  Header file for Pseudorange Switchable factor
 */

#include <Eigen/Eigen>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <limits>

namespace gtsam {

  template<class VALUE>
  class BetweenFactorMaxMix : public NoiseModelFactor2<VALUE, VALUE>
  {
    public:
      using Base = NoiseModelFactor2<VALUE, VALUE>;
      using HValue = typename Base::template OptionalMatrix<VALUE>;

      BetweenFactorMaxMix() {};
      BetweenFactorMaxMix(Key key1, Key key2, const VALUE& measured, const SharedNoiseModel& model, const SharedNoiseModel& model2, const Vector& hypVec, double w)
      : Base(model, key1, key2), weight(w), nullHypothesisModel(model2), hyp(hypVec),
        betweenFactor(key1, key2, measured, model)  {   };

      Vector evaluateError(const VALUE& p1, const VALUE& p2,
          HValue H1 = boost::none,
          HValue H2 =  boost::none) const
        {

          // calculate error
          Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);

          // which hypothesis is more likely
          auto g1 = noiseModel::Gaussian::Covariance(hyp.asDiagonal());
          auto g2 = noiseModel::Gaussian::Covariance((hyp/weight).asDiagonal());

          double m1 = this->noiseModel_->squaredMahalanobisDistance(error);
          Matrix info1(g1->information());
          const double info_det1 = std::max(info1.determinant(), std::numeric_limits<double>::min());
          double nu1 = sqrt(info_det1);
          double l1 = nu1 * exp(-0.5*m1);

          double m2 = nullHypothesisModel->squaredMahalanobisDistance(error);
          Matrix info2(g2->information());
          const double info_det2 = std::max(info2.determinant(), std::numeric_limits<double>::min());
          double nu2 = sqrt(info_det2);
          double l2 = nu2 * exp(-0.5*m2);

          if (l2>l1) {
            if (H1) *H1 = *H1 * weight;
            if (H2) *H2 = *H2 * weight;
            error *= sqrt(weight);
          }

          return error;
        };

    private:
      BetweenFactor<VALUE> betweenFactor;
      SharedNoiseModel nullHypothesisModel;
      double weight;
      Vector hyp;

  };
}
