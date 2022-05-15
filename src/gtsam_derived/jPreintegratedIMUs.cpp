#include "jPreintegratedIMUs.h"

using namespace std;

namespace VIG {

void jPreintegratedIMUs::BiasCorrectedDelta(
    const imuBias::ConstantBias &bias_i, OptionalJacobian<9, 6> H) {
  // Correct deltaRij, derivative is delRdelBiasOmega_
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_correctedRij_bias;
  const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasIncr.gyroscope();
  const Rot3 correctedRij = deltaRij().expmap(biasInducedOmega, boost::none,
                                              H ? &D_correctedRij_bias : 0);
  if (H) D_correctedRij_bias *= delRdelBiasOmega_;

  Matrix3 D_dR_correctedRij;
  // TODO(frank): could line below be simplified? It is equivalent to
  //   LogMap(deltaRij_.compose(Expmap(biasInducedOmega)))
  Vector3 correctedPij = deltaPij() +
                         delPdelBiasAcc_ * biasIncr.accelerometer() +
                         delPdelBiasOmega_ * biasIncr.gyroscope();
  Vector3 correctedVij = deltaVij() +
                         delVdelBiasAcc_ * biasIncr.accelerometer() +
                         delVdelBiasOmega_ * biasIncr.gyroscope();
  deltaXij_ = NavState(correctedRij, correctedPij, correctedVij);

  if (H) {
    Matrix36 D_dR_bias, D_dP_bias, D_dV_bias;
    D_dR_bias << Z_3x3, D_dR_correctedRij * D_correctedRij_bias;
    D_dP_bias << delPdelBiasAcc_, delPdelBiasOmega_;
    D_dV_bias << delVdelBiasAcc_, delVdelBiasOmega_;
    (*H) << D_dR_bias, D_dP_bias, D_dV_bias;
  }
}

}  // namespace VIG
