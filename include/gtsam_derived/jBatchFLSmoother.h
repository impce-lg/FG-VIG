/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    jBatchFLSmoother.h
 * @brief   An LM-based fixed-lag smoother.
 *
 * @author  Jin
 * @date    Oct 14, 2012
 */

// \callgraph
#pragma once

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

namespace gtsam {

class GTSAM_UNSTABLE_EXPORT jBatchFLSmoother : public BatchFixedLagSmoother {

public:

  /// Typedef for a shared pointer to an Incremental Fixed-Lag Smoother
  typedef boost::shared_ptr<jBatchFLSmoother> shared_ptr;

  /** default constructor */
  jBatchFLSmoother(
      double smootherLag = 0.0,
      const LevenbergMarquardtParams& parameters = LevenbergMarquardtParams(),
      bool enforceConsistency = true)
      : BatchFixedLagSmoother(smootherLag){};

  /** destructor */
  virtual ~jBatchFLSmoother() { };

  /** Test(Jin): Add new factors, updating the solution and relinearizing as needed. */
  Result Update(const NonlinearFactorGraph& newFactors, const Values& newTheta,
                const KeyVector& marginalizableKeys);

  /** Test(Jin): Update the current linearization point */
  template <typename ValueType>
  void updateLinearizationPoint(Key j, const ValueType& val) {
    theta_.update(j, val);
  }

  /** Test(Jin): Marginalize out selected variables */
  double GetCurrentTimestamp() const;
  KeyVector FindKeysBefore(double timestamp) const;
  void Reorder(const KeyVector& marginalizeKeys = KeyVector());
  NonlinearFactorGraph CalMarginalFactors(const KeyVector& marginalizableKeys);
  const NonlinearFactorGraph& GetMarginalFactors() const {
    return marginalFactors_;
  }

protected:

  NonlinearFactorGraph marginalFactors_;

}; // jBatchFLSmoother

} /// namespace gtsam
