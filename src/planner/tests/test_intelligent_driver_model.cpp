/*
 * Copyright 2020 Ke Sun
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <gtest/gtest.h>
#include <planner/common/intelligent_driver_model.h>

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
