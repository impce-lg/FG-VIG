#pragma once

#include <gtsam/navigation/ImuFactor.h>

using namespace gtsam;

namespace VIG {

class jPreintegratedIMUs : public PreintegratedImuMeasurements {
 public:
  jPreintegratedIMUs() { preintMeasCov_.setZero(); };
  jPreintegratedIMUs(
      const boost::shared_ptr<PreintegrationParams> &p,
      const imuBias::ConstantBias &biasHat = imuBias::ConstantBias())
      : PreintegratedImuMeasurements(p, biasHat){};
  void BiasCorrectedDelta(const imuBias::ConstantBias &bias_i,
                          OptionalJacobian<9, 6> H = boost::none);

  //  public:
  //   NavState mDeltaXij;
};

}  // namespace VIG
