#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "../laser.h"
#include "../ranger.h"
#include "../radar.h"

using namespace std;

//==================== HELPER FUNCTIONS ====================//
void testLimits(vector<double> stream, double min, double max) {
  for(auto measurement: stream) {
    ASSERT_TRUE(measurement <= max);
    ASSERT_TRUE(measurement >= min);
  }
}
//==================== HELPER FUNCTIONS ====================//

//==================== UNIT TEST START ====================//
//Test that data measurements are within limits
TEST (DataDistanceTest, Laser) {
  Laser l;

  //We arbitarily test the data 100 times as its randomly generated
  for(int i = 0; i < 100; i++) {
    testLimits(l.generateData(), 0.2, 8.0);
  }
}

TEST (DataDistanceTest, Radar) {
  Radar r;

  //We arbitarily test the data 100 times as its randomly generated
  for(int i = 0; i < 100; i++) {
    testLimits(r.generateData(), 0.2, 16.0);
  }
}

//Test for correct number of measurements produced
TEST (NumMeasurementsTest, LaserAngRes10) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=10, offset=0}
  //
  // Should produce 1 vector with 19 measurements. (no overlap)
  Laser l;
  l.setAngularResolution(10);
  l.setOffset(0);

  std::vector<RangerInterface *> sensors = { &l };
  auto rawdata = sensors.front()->generateData();

  ASSERT_EQ(19, rawdata.size());
}

TEST (NumMeasurementsTest, LaserAngRes30) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=30, offset=0}
  //
  // Should produce 1 vector with 7 measurements. (no overlap)
  Laser l;
  l.setAngularResolution(30);
  l.setOffset(0);

  std::vector<RangerInterface *> sensors = { &l };
  auto rawdata = sensors.front()->generateData();

  ASSERT_EQ(7, rawdata.size());
}

TEST (NumMeasurementsTest, Radar) {
  //Sensors:
  //  - (1) Radar {fov=60, angular_res=20, offset=0}
  //
  // Should produce 1 vector with 3 measurements. (no overlap)
  Radar r;
  r.setOffset(0);

  std::vector<RangerInterface *> sensors = { &r };
  auto rawdata = sensors.front()->generateData();

  ASSERT_EQ(3, rawdata.size());
}

TEST (validInputs, Laser)
{
    Laser l;
    EXPECT_TRUE(l.setAngularResolution(10));
    EXPECT_TRUE(l.setAngularResolution(30));
}

TEST (InvalidInputs, Laser)
{
    Laser l;
    int prev_angular_res = l.getAngularResolution();
    EXPECT_FALSE(l.setAngularResolution(34298));
    EXPECT_EQ(l.getAngularResolution(), prev_angular_res);


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
