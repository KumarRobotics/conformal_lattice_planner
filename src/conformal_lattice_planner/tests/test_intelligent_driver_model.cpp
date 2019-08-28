/*
 * Copyright [2019] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>
#include <conformal_lattice_planner/intelligent_driver_model.h>

using namespace planner;

TEST(IntelligentDriverModel, accessors) {
  IntelligentDriverModel idm;

  idm.timeGap() = 2.0;
  EXPECT_DOUBLE_EQ(idm.timeGap(), 2.0);

  idm.distanceGap() = 10.0;
  EXPECT_DOUBLE_EQ(idm.distanceGap(), 10.0);

  idm.accelExp() = 5.0;
  EXPECT_DOUBLE_EQ(idm.accelExp(), 5.0);

  idm.comfortAccel() = 2.0;
  EXPECT_DOUBLE_EQ(idm.comfortAccel(), 2.0);

  idm.comfortDecel() = 2.0;
  EXPECT_DOUBLE_EQ(idm.comfortDecel(), 2.0);

  return;
}

TEST(IntelligentDriverModel, idm) {
  IntelligentDriverModel idm;

  const double ego_v = 28.0;
  const double ego_v0 = 30.0;
  const double lead_v = 25.0;
  const double s = 20;

  const double free_accel = idm.idm(ego_v, ego_v0);
  EXPECT_NEAR(free_accel, 0.3617, 1e-3);

  const double block_accel = idm.idm(ego_v, ego_v0, lead_v, s);
  EXPECT_NEAR(block_accel, -11.2679, 1e-3);

  return;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
