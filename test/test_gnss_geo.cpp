#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "gnss_geo.hpp"

namespace
{

constexpr double kLat0 = 36.08288044139598;
constexpr double kLon0 = 140.07654638977903;
constexpr double kAlt0 = 41.1224;

std::pair<double, double> localizationFormula(double lat, double lon, double lat0, double lon0)
{
  const double a = 6378137.0, f = 1.0 / 298.257222101;
  const double e2 = f * (2.0 - f);
  const double s = std::sin(lat0 * M_PI / 180.0);
  const double n = a / std::sqrt(1.0 - e2 * s * s) * std::cos(lat0 * M_PI / 180.0);
  const double m = a * (1.0 - e2) / std::pow(1.0 - e2 * s * s, 1.5);
  return {(lon - lon0) * M_PI / 180.0 * n, (lat - lat0) * M_PI / 180.0 * m};
}

}  // namespace

TEST(GnssGeo, OriginIsDatum)
{
  const auto [lat, lon] = rsf_simulator::xyToLatLon(0.0, 0.0, kLat0, kLon0);
  EXPECT_NEAR(lat, kLat0, 1e-15);
  EXPECT_NEAR(lon, kLon0, 1e-15);
}

TEST(GnssGeo, RoundtripWithLocalizationFormulaWithin1mm)
{
  std::mt19937_64 rng(0);
  std::uniform_real_distribution<double> dist_x(-450.0, 600.0), dist_y(-500.0, 300.0);
  for (int i = 0; i < 200; ++i) {
    const double x = dist_x(rng), y = dist_y(rng);
    const auto [lat, lon] = rsf_simulator::xyToLatLon(x, y, kLat0, kLon0);
    const auto [bx, by] = localizationFormula(lat, lon, kLat0, kLon0);
    EXPECT_LT(std::abs(bx - x), 1e-3);
    EXPECT_LT(std::abs(by - y), 1e-3);
  }
}

TEST(GnssGeo, LatLonToXyInverse)
{
  const auto [lat, lon] = rsf_simulator::xyToLatLon(123.4, -56.7, kLat0, kLon0);
  const auto [x, y] = rsf_simulator::latLonToXy(lat, lon, kLat0, kLon0);
  EXPECT_LT(std::abs(x - 123.4), 1e-6);
  EXPECT_LT(std::abs(y + 56.7), 1e-6);
}

TEST(GnssGeo, AltitudeAddsDatumAltitude)
{
  EXPECT_NEAR(rsf_simulator::altitudeFromZ(0.5, kAlt0), 41.6224, 1e-9);
}
