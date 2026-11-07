#include "ft_tilt_control/allocator.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace ft_tilt
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTol = 1e-9;

// Bounds on the working motors as G·z ≥ h, for f = f0 + K·z.
struct Constraints
{
  Eigen::MatrixXd g;
  Eigen::VectorXd h;

  bool feasible(const Eigen::VectorXd & z) const
  {
    return ((g * z - h).array() >= -kTol).all();
  }
};

Constraints bounds_on_z(
  const Vector6d & f0, const Eigen::MatrixXd & k, const Vector6d & lower,
  const Vector6d & upper, const std::vector<int> & active)
{
  const int m = 2 * static_cast<int>(active.size());
  Constraints c{Eigen::MatrixXd(m, k.cols()), Eigen::VectorXd(m)};
  for (size_t j = 0; j < active.size(); ++j) {
    const int i = active[j];
    c.g.row(2 * j) = k.row(i);
    c.h(2 * j) = lower(i) - f0(i);
    c.g.row(2 * j + 1) = -k.row(i);
    c.h(2 * j + 1) = f0(i) - upper(i);
  }
  return c;
}

// Calls fn with the row indices of every subset of size s of {0..m-1}.
void for_each_subset(int m, int s, const std::function<void(const std::vector<int> &)> & fn)
{
  std::vector<bool> pick(m, false);
  std::fill(pick.begin(), pick.begin() + s, true);
  std::vector<int> rows;
  do {
    rows.clear();
    for (int j = 0; j < m; ++j) {
      if (pick[j]) {
        rows.push_back(j);
      }
    }
    fn(rows);
  } while (std::prev_permutation(pick.begin(), pick.end()));
}

// Smallest ‖z‖ with G·z ≥ h. The optimum is the least-norm point of some set of at most
// dim(z) active constraints, so we try them all (≤ 79 small solves) and keep the best
// feasible one. With K orthonormal this is the minimum-norm f within the bounds.
std::optional<Eigen::VectorXd> min_norm_within_bounds(const Constraints & c)
{
  const int n = static_cast<int>(c.g.cols());
  const int m = static_cast<int>(c.g.rows());
  std::optional<Eigen::VectorXd> best;
  double best_norm = kInf;
  for (int s = 0; s <= std::min(n, m); ++s) {
    for_each_subset(m, s, [&](const std::vector<int> & rows) {
        Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
        if (!rows.empty()) {
          const Eigen::MatrixXd gs = c.g(rows, Eigen::placeholders::all);
          const Eigen::FullPivLU<Eigen::MatrixXd> lu(gs * gs.transpose());
          if (!lu.isInvertible()) {
            return;
          }
          z = gs.transpose() * lu.solve(c.h(rows));
        }
        if (z.norm() < best_norm && c.feasible(z)) {
          best = z;
          best_norm = z.norm();
        }
      });
  }
  return best;
}

// No z satisfies the bounds: z maximising t in G·z − t ≥ h (least violation), by
// checking every vertex of that linear program in (z, t).
Eigen::VectorXd least_violation(const Constraints & c)
{
  const int n = static_cast<int>(c.g.cols()) + 1;
  Eigen::MatrixXd g(c.g.rows(), n);
  g << c.g, -Eigen::VectorXd::Ones(c.g.rows());
  Eigen::VectorXd best = Eigen::VectorXd::Zero(n);
  double best_t = -kInf;
  for_each_subset(static_cast<int>(g.rows()), n, [&](const std::vector<int> & rows) {
      const Eigen::FullPivLU<Eigen::MatrixXd> lu(g(rows, Eigen::placeholders::all));
      if (!lu.isInvertible()) {
        return;
      }
      const Eigen::VectorXd x = lu.solve(c.h(rows));
      if (x(n - 1) > best_t && ((g * x - c.h).array() >= -kTol).all()) {
        best = x;
        best_t = x(n - 1);
      }
    });
  return best.head(n - 1);
}

double margin_of(
  const Vector6d & f, const Vector6d & lower, const Vector6d & upper,
  const std::vector<int> & active)
{
  double m = kInf;
  for (int i : active) {
    m = std::min({m, f(i) - lower(i), upper(i) - f(i)});
  }
  return m;
}

}  // namespace

Allocation allocate(
  const Matrix46d & a, const Eigen::Vector4d & wrench,
  const Vector6d & lower, const Vector6d & upper, bool use_null_space)
{
  Matrix46d af = a;
  std::vector<int> active;
  for (int i = 0; i < kNumRotors; ++i) {
    if (upper(i) <= 0.0) {
      af.col(i).setZero();
    } else {
      active.push_back(i);
    }
  }

  Vector6d f = af.completeOrthogonalDecomposition().pseudoInverse() * wrench;

  if (use_null_space && active.size() > 4 && margin_of(f, lower, upper, active) < 0.0) {
    const Eigen::MatrixXd act = af(Eigen::placeholders::all, active);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(act, Eigen::ComputeFullV);
    const int dim = static_cast<int>(active.size()) - static_cast<int>(svd.rank());
    if (dim > 0) {
      Eigen::MatrixXd k = Eigen::MatrixXd::Zero(kNumRotors, dim);  // orthonormal null basis
      for (size_t j = 0; j < active.size(); ++j) {
        k.row(active[j]) = svd.matrixV().row(j).tail(dim);
      }
      const Constraints c = bounds_on_z(f, k, lower, upper, active);
      const auto z = min_norm_within_bounds(c);
      f += k * (z ? *z : least_violation(c));
    }
  }

  Allocation out;
  out.margin = margin_of(f, lower, upper, active);
  for (int i = 0; i < kNumRotors; ++i) {
    f(i) = upper(i) <= 0.0 ? 0.0 : std::clamp(f(i), lower(i), upper(i));
  }
  out.force = f;
  return out;
}

}  // namespace ft_tilt
