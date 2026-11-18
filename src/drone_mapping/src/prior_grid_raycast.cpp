#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using Point = std::array<double, 3>;
using Index = std::array<int, 3>;
constexpr double kResolution = 0.05;
constexpr double kRange = 4.5;
constexpr double kReturnMargin = 0.05;
constexpr double kEpsilon = 1e-9;

struct Evidence {
  Point origin;
  Index size;
  double blind;
  std::vector<unsigned char> states;
  std::vector<std::int32_t> first_second;

  std::size_t address(const Index& id) const {
    return (std::size_t(id[0]) * size[1] + id[1]) * size[2] + id[2];
  }

  bool inside(const Index& id) const {
    for (int i = 0; i < 3; ++i)
      if (id[i] < 0 || id[i] >= size[i]) return false;
    return true;
  }

  void trace(const std::array<double, 7>& ray) {
    Point start, direction;
    double range = 0.0;
    for (int i = 0; i < 3; ++i) {
      start[i] = (ray[i] - origin[i]) / kResolution;
      direction[i] = ray[i+3] - ray[i];
      range += direction[i] * direction[i];
    }
    range = std::sqrt(range);
    if (!std::isfinite(range) || range <= blind) return;

    Index end;
    for (int i = 0; i < 3; ++i)
      end[i] = std::floor(start[i] + direction[i] / kResolution);
    // Keep raw ends out of the free class, including returns removed from the PCD.
    if (inside(end)) states[address(end)] = 2;

    double enter = blind, leave = std::min(kRange, range - kReturnMargin);
    for (int i = 0; i < 3; ++i) {
      direction[i] /= range * kResolution;
      if (std::abs(direction[i]) < kEpsilon) {
        if (start[i] < 0 || start[i] >= size[i]) return;
        continue;
      }
      const double a = -start[i] / direction[i];
      const double b = (size[i] - start[i]) / direction[i];
      enter = std::max(enter, std::min(a, b));
      leave = std::min(leave, std::max(a, b));
    }
    if (leave - enter <= kEpsilon) return;

    Index id, step;
    Point crossing, interval;
    for (int i = 0; i < 3; ++i) {
      id[i] = std::floor(start[i] + (enter + kEpsilon) * direction[i]);
      if (std::abs(direction[i]) < kEpsilon) {
        crossing[i] = interval[i] = std::numeric_limits<double>::infinity();
        step[i] = 0;
      } else {
        step[i] = direction[i] > 0 ? 1 : -1;
        crossing[i] = (id[i] + (step[i] > 0) - start[i]) / direction[i];
        interval[i] = 1.0 / std::abs(direction[i]);
      }
    }
    const auto second = static_cast<std::int32_t>(ray[6]);
    while (inside(id)) {
      const double next = std::min({crossing[0], crossing[1], crossing[2], leave});
      if (next - enter > kEpsilon) {
        const auto adr = address(id);
        if (states[adr] == 0) {
          if (first_second[adr] == -1) first_second[adr] = second;
          else if (first_second[adr] != second) states[adr] = 1;
        }
      }
      if (next >= leave - kEpsilon) break;
      for (int i = 0; i < 3; ++i) {
        if (crossing[i] <= next + kEpsilon) {
          id[i] += step[i];
          crossing[i] += interval[i];
        }
      }
      enter = next;
    }
  }
};
}  // namespace

int main(int argc, const char* const* argv) {
  try {
    if (argc != 8) throw std::runtime_error("expected origin xyz, dimensions xyz and blind range");
    Evidence grid;
    std::size_t cells = 1;
    for (int i = 0; i < 3; ++i) {
      grid.origin[i] = std::stod(argv[1+i]);
      grid.size[i] = std::stoi(argv[4+i]);
      if (!std::isfinite(grid.origin[i]) || grid.size[i] < 1 || grid.size[i] > 1000)
        throw std::runtime_error("invalid grid dimensions");
      cells *= grid.size[i];
    }
    grid.blind = std::stod(argv[7]);
    if (!std::isfinite(grid.blind) || grid.blind < 0 || grid.blind >= kRange || cells > 25000000)
      throw std::runtime_error("invalid blind range or grid size");
    grid.states.assign(cells, 0);
    grid.first_second.assign(cells, -1);

    std::ios::sync_with_stdio(false);
    std::array<double, 7> ray;
    while (std::cin.read(reinterpret_cast<char*>(ray.data()), sizeof(ray))) grid.trace(ray);
    if (!std::cin.eof() || std::cin.gcount()) throw std::runtime_error("truncated ray stream");
    for (auto& state : grid.states)
      if (state == 2) state = 0;
    std::cout.write(reinterpret_cast<const char*>(grid.states.data()), grid.states.size());
    return std::cout ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "prior_grid_raycast: " << error.what() << '\n';
    return 1;
  }
}
