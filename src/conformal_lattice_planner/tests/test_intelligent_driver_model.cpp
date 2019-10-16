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

TEST(BaseIntelligentDriverModel, accessors) {
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

}

TEST(BasicIntelligentDriverModel, idm) {
  BasicIntelligentDriverModel idm;

  {
    const double ego_v = 10.0;
    const double ego_v0 = 29.0;
    const double accel = idm.idm(ego_v, ego_v0);
    EXPECT_NEAR(accel, 1.47879202184, 1e-3);
  }

  {
    const double ego_v = 40.0;
    const double ego_v0 = 29.0;
    const double accel = idm.idm(ego_v, ego_v0);
    EXPECT_NEAR(accel, -3.9292424086, 1e-3);
  }

  {
    const double ego_v = 20.0;
    const double ego_v0 = 29.0;
    const double lead_v = 29.0;
    const double s = 50.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, 1.13907234946, 1e-3);
  }

  {
    const double ego_v = 25.0;
    const double ego_v0 = 29.0;
    const double lead_v = 15.0;
    const double s = 50.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, -4.80628632147, 1e-3);
  }

  {
    const double ego_v = 29.0;
    const double ego_v0 = 29.0;
    const double lead_v = 0.0;
    const double s = 100.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, -8.0, 1e-3);
  }
}

TEST(ImprovedIntelligentDriverModel, idm) {
  ImprovedIntelligentDriverModel idm;

  {
    const double ego_v = 10.0;
    const double ego_v0 = 29.0;
    const double accel = idm.idm(ego_v, ego_v0);
    EXPECT_NEAR(accel, 1.47879202184, 1e-3);
  }

  {
    const double ego_v = 40.0;
    const double ego_v0 = 29.0;
    const double accel = idm.idm(ego_v, ego_v0);
    EXPECT_NEAR(accel, -1.34454982035, 1e-3);
  }

  {
    const double ego_v = 20.0;
    const double ego_v0 = 29.0;
    const double lead_v = 29.0;
    const double s = 50.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, 1.15583439997, 1e-3);
  }

  {
    const double ego_v = 25.0;
    const double ego_v0 = 29.0;
    const double lead_v = 15.0;
    const double s = 50.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, -3.97784967465, 1e-3);
  }

  {
    const double ego_v = 29.0;
    const double ego_v0 = 29.0;
    const double lead_v = 0.0;
    const double s = 100.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, -8.0, 1e-3);
  }
}

TEST(AdaptiveCruiseControl, idm) {
  AdaptiveCruiseControl idm;

  {
    const double ego_v = 10.0;
    const double ego_v0 = 29.0;
    const double accel = idm.idm(ego_v, ego_v0);
    EXPECT_NEAR(accel, 1.47879202184, 1e-3);
  }

  {
    const double ego_v = 40.0;
    const double ego_v0 = 29.0;
    const double accel = idm.idm(ego_v, ego_v0);
    EXPECT_NEAR(accel, -1.34454982035, 1e-3);
  }

  {
    const double ego_v = 20.0;
    const double ego_v0 = 29.0;
    const double lead_v = 29.0;
    const double s = 50.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, 1.15583439997, 1e-3);
  }

  {
    const double ego_v = 25.0;
    const double ego_v0 = 29.0;
    const double lead_v = 15.0;
    const double s = 50.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, -3.16738208405, 1e-3);
  }

  {
    const double ego_v = 29.0;
    const double ego_v0 = 29.0;
    const double lead_v = 0.0;
    const double s = 100.0;
    const double accel = idm.idm(ego_v, ego_v0, lead_v, s);
    EXPECT_NEAR(accel, -6.62828409616, 1e-3);
  }
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
