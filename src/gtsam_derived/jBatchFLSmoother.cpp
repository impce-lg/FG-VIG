/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    jBatchFLSmoother.cpp
 * @brief   An LM-based fixed-lag smoother.
 *
 * @author  Jin
 * @date    Oct 14, 2012
 */

#include "jBatchFLSmoother.h"

using namespace std;

namespace gtsam {

/* ************************************************************************* */
FixedLagSmoother::Result jBatchFLSmoother::Update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const KeyVector& marginalizableKeys) {
  // Update all of the internal variables with the new information
  // Add the new variables to theta
  theta_.insert(newTheta);
  // Add new variables to the end of the ordering
  for (const auto& key_value : newTheta) {
    ordering_.push_back(key_value.key);
  }
  // Augment Delta
  delta_.insert(newTheta.zeroVectors());

  // Add the new factors to the graph, updating the variable index
  insertFactors(newFactors);

  // Reorder
  reorder(marginalizableKeys);

  // Optimize
  Result result;
  if (factors_.size() > 0) {
    result = optimize();
  }

  // Marginalize out old variables.
  if (marginalizableKeys.size() > 0) {
    marginalFactors_ = CalMarginalFactors(marginalizableKeys);
  }

  return result;
}

double jBatchFLSmoother::GetCurrentTimestamp() const {
  if(timestampKeyMap_.size() > 0) {
    return timestampKeyMap_.rbegin()->first;
  } else {
    return -std::numeric_limits<double>::max();
  }
}

/* ************************************************************************* */
KeyVector jBatchFLSmoother::FindKeysBefore(double timestamp) const {
  KeyVector keys;
  TimestampKeyMap::const_iterator end = timestampKeyMap_.lower_bound(timestamp);
  for(TimestampKeyMap::const_iterator iter = timestampKeyMap_.begin(); iter != end; ++iter) {
    keys.push_back(iter->second);
  }
  return keys;
}

/* ************************************************************************* */
void jBatchFLSmoother::Reorder(const KeyVector& marginalizeKeys) {
  // COLAMD groups will be used to place marginalize keys in Group 0, and everything else in Group 1
  ordering_ = Ordering::ColamdConstrainedFirst(factors_, marginalizeKeys);
}

/** Test(Jin): Marginalize out selected variables */
NonlinearFactorGraph jBatchFLSmoother::CalMarginalFactors(
    const KeyVector& marginalizeKeys) {
  // In order to marginalize out the selected variables, the factors involved in
  // those variables must be identified and removed. Also, the effect of those
  // removed factors on the remaining variables needs to be accounted for. This
  // will be done with linear container factors from the result of a partial
  // elimination. This function removes the marginalized factors and adds the
  // linearized factors back in.

  // Identify all of the factors involving any marginalized variable. These must
  // be removed.
  set<size_t> removedFactorSlots;
  const VariableIndex variableIndex(factors_);
  for (Key key : marginalizeKeys) {
    const auto& slots = variableIndex[key];
    removedFactorSlots.insert(slots.begin(), slots.end());
  }

  // Add the removed factors to a factor graph
  NonlinearFactorGraph removedFactors;
  for (size_t slot : removedFactorSlots) {
    if (factors_.at(slot)) {
      removedFactors.push_back(factors_.at(slot));
    }
  }

  // Calculate marginal factors on the remaining keys
  NonlinearFactorGraph marginalFactors =
      CalculateMarginalFactors(removedFactors, theta_, marginalizeKeys,
                               parameters_.getEliminationFunction());
                            
  return marginalFactors;
}

/* ************************************************************************* */
} /// namespace gtsam
