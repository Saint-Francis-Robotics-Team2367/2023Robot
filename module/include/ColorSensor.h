#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <array>
#include <cmath>

class ColorSensor{
  public: 
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor{i2cPort};
  struct colorTarget{
    double minH;
    double maxH;
    double minS;
    double minV;
    double minTotal;
    colorTarget(double minH, double maxH, double minS, double minV, double minTotal):
    minH(minH), maxH(maxH), minS(minS), minV(minV), minTotal(minTotal)
    {
    }
  };
  double minIR = 100;
  colorTarget coneColorTarget = colorTarget(6,7,8,8,8);
  colorTarget cubeColorTarget = colorTarget(1,1,1,1,7);
  ColorSensor();
  rev::ColorSensorV3::RawColor getColor();
  uint32_t getProximity();
  double getIR();
  bool alignedIR(double);
  std::array<int, 3> rgbTohsv(frc::Color);
  bool matchesCubeColor(rev::ColorSensorV3::RawColor);
  bool matchesConeColor(rev::ColorSensorV3::RawColor);
  bool matchesTarget(rev::ColorSensorV3::RawColor, colorTarget);
  private:
};
