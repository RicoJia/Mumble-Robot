#pragma once
#include <Eigen/Core>

namespace math {
template <typename Container, typename VectorType, typename Getter>
void compute_cov_and_mean(const Container &data, VectorType &mean,
                          VectorType &cov, Getter &&getter) {
  const size_t len = data.size();
  assert(len > 1);
  mean = std::accumulate(
             data.begin(), data.end(), VectorType::Zero().eval(),
             [&getter](const VectorType &sum, const auto &item) -> VectorType {
               return sum + getter(item);
             }) /
         static_cast<double>(len);
  // Compute covariance diagonal
  cov = std::accumulate(data.begin(), data.end(), VectorType::Zero().eval(),
                        [&mean, &getter](const VectorType &sum,
                                         const auto &item) -> VectorType {
                          auto diff = getter(item) - mean;
                          return sum + diff.cwiseProduct(diff);
                        }) /
        static_cast<double>(len - 1);
}
} // namespace math