#include <cmath>
#include <vector>
#include <iostream>
#include <tuple>
#include "DriveBaseModule.h"

using namespace std;

void DriveBaseModule::ArmInit() {
  kp = 0.5;
  ki = 0.5;
  kd = 0.2;
  lArmPID.SetP(kp);
  lArmPID.SetI(ki);
  lArmPID.SetD(kd);
  rArmPID.SetP(kp);
  rArmPID.SetI(ki);
  rArmPID.SetD(kd);
  frc::SmartDashboard::PutNumber("P Gain", kp);
  frc::SmartDashboard::PutNumber("I Gain", ki);
  frc::SmartDashboard::PutNumber("D Gain", kd);
}

void DriveBaseModule::ArmPeriodic() {
  double sensor_position = lArmEncoder.GetPosition();
  double theta = plot_point({43,25}, 30, 30);
  cout << theta;
  if (sensor_position>theta) {
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    if((p != kp)) {
      lArmPID.SetP(p);
      rArmPID.SetP(p);
      kp = p;
    }
    if((i != ki)) {
      lArmPID.SetI(i);
      rArmPID.SetP(i);
      ki = i;
    }
    if((d != kd)) {
      lArmPID.SetD(d);
      rArmPID.SetP(d);
      kd = d;
    }
  }
}

vector<tuple<double, double>, tuple<double, double>> circle_line_segment_intersection(std::pair<double, double> center, double radius, std::pair<double, double> pt1, std::pair<double, double> pt2, bool full_line, double tangent_tol) {
  // Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.
  double p1x, p1y, p2x, p2y, cx, cy;
  tie(p1x, p1y) = pt1;
  tie(p2x, p2y) = pt2;
  tie(cx, cy) = center;
  double x1, y1, x2, y2;
  x1 = p1x - cx;
  y1 = p1y - cy;
  x2 = p2x - cx;
  y2 = p2y - cy;
  double dx, dy;
  dx = x2 - x1;
  dy = y2 - y1;
  double dr = sqrt(dx*dx + dy*dy);
  double big_d = x1 * y2 - x2 * y1;
  double discriminant = radius * radius * dr*dr - big_d*big_d;
  vector<tuple<double, double>, tuple<double, double>> intersections;
  if (discriminant < 0) {  // No intersection between circle and line
    return;
  }
  else {  // There may be 0, 1, or 2 intersections with the segment
    for (int sign : {1, -1}) {
        intersections.push_back(make_tuple(cx + (big_d * dy + sign * (dy < 0 ? -1 : 1) * dx * std::sqrt(discriminant)) / dr*dr, cy + (-big_d * dx + sign * std::abs(dy) * std::sqrt(discriminant)) / dr*dr));
    }
    if (!full_line) {  // If only considering the segment, filter out intersections that do not fall within the segment
        for (int i = 0; i < intersections.size(); i++) {
            double xi, yi;
            tie(xi, yi) = intersections[i];
            double fraction;
        if (abs(dx) > abs(dy)) {
          fraction = (xi - p1x) / dx;
        }
        else {
          fraction = (yi - p1y) / dy;
        }
        if (fraction < 0 || fraction > 1) {
          intersections.erase(intersections.begin() + i);
          i--;
        }
      }
    }
        if (intersections.size() == 2 && abs(discriminant) <= tangent_tol) {
            return intersections[0];
        }
        else {
            return intersections;
        }
  }
}

int zeroClamp(int x, int bound = 0.1) {
    if ((-bound < x) && (x<bound)) {
        return 0.01;
    }
    else {
        return x;
    }
}

double plot_point(std::pair<double, double> point, double length1, double length2) {
  double a = point.first;
  double b = point.second;
  double t1x = -60;
  double t1y = ((-1 * a / b) * (t1x - (a / 2))) + (b / 2);
  double t2x = 60;
  double t2y = ((-1 * a / b) * (t2x - (a / 2))) + (b / 2);
  vector<tuple<double, double>, tuple<double, double>> b_point = circle_line_segment_intersection({0, 0}, 30, {t1x, t1y}, {t2x, t2y}, true, 1e-9);
  tuple<double, double> b_point1 = b_point[0];
  tuple<double, double> b_point2 = b_point[1];
  double bx = get<0>(b_point1);
  double by = get<1>(b_point1);
  bx = zeroClamp(bx);
  by = zeroClamp(by);
  double c2y = zeroClamp(b - by);
  double c2x = zeroClamp(a - bx);
  double theta1 = (180 / M_PI) * atan2(by, bx);
  double theta2 = (180 / M_PI) * (atan2(c2y, c2x) - theta1);
  return theta1;
}