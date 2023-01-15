#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <set>

class ColorSensor{
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor{i2cPort};

  double minMatchConfidence = 0.95;

  rev::ColorMatch coneColorMatcher;
  rev::ColorMatch cubeColorMatcher;

  std::set<frc::Color> coneColorTargets = {};
  std::set<frc::Color> cubeColorTargets = {};

  public:
  ColorSensor();
  frc::Color getColor();
  uint32_t getProximity();
  bool matchesCubeColor(frc::Color);
  bool matchesConeColor(frc::Color);
};