#pragma once
#include <Eigen/Core>

namespace math {
template <typename Container, typename VectorType, typename Getter>
void compute_cov_and_mean(const Container &data, VectorType &mean,
                          VectorType &cov, Getter &&getter) {
    const size_t len = data.size();
    assert(len > 1);
    mean = std::accumulate(data.begin(), data.end(), VectorType::Zero().eval(),
                           [&getter](const VectorType &sum, const auto &item)
                               -> VectorType { return sum + getter(item); }) /
           static_cast<double>(len);
    // Compute covariance diagonal
    cov = std::accumulate(data.begin(), data.end(), VectorType::Zero().eval(),
                          [&mean, &getter](const VectorType &sum,
                                           const auto &item) -> VectorType {
                              // eigen uses lazy evaluation. That could be wrong
                              // when doing getter(item) - mean?? THIS DOESN'T
                              // WORK: auto diff =  getter(item).eval() - mean;
                              auto value = getter(item).eval();
                              auto diff  = value - mean;
                              return sum + diff.cwiseProduct(diff);
                          }) /
          static_cast<double>(len - 1);
}

//////////////////////////////////////////////////////////////////////////////
// Search in Tree
//////////////////////////////////////////////////////////////////////////////

template <typename EigenVectorType>
double get_squared_distance(const EigenVectorType &p1,
                            const EigenVectorType &query) {
    return (p1 - query).squaredNorm();
}

}   // namespace math