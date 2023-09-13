#include <gtest/gtest.h>

#include "fastcat/config.h"
#include "fastcat/jsd/gold_actuator_offline.h"
#include "fastcat/jsd/jsd_device_base.h"

namespace
{
class JsdDeviceBaseTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    // FASTCAT_UNIT_TEST_DIR contains path to .
    sdo_response_queue_ = std::make_shared<std::queue<fastcat::SdoResponse>>();
    device_.SetOffline(true);
  }

  fastcat::GoldActuatorOffline                      device_;
  std::shared_ptr<std::queue<fastcat::SdoResponse>> sdo_response_queue_;
};

TEST_F(JsdDeviceBaseTest, TesterQueueOps)
{
  EXPECT_TRUE(sdo_response_queue_->empty());

  {
    fastcat::SdoResponse resp;
    resp.device_name = "jose_cuervo";
    sdo_response_queue_->push(resp);
    EXPECT_FALSE(sdo_response_queue_->empty());
  }

  {
    fastcat::SdoResponse resp;
    resp = sdo_response_queue_->front();
    sdo_response_queue_->pop();
    EXPECT_EQ(0, resp.device_name.compare("jose_cuervo"));
    EXPECT_TRUE(sdo_response_queue_->empty());
  }
}

TEST_F(JsdDeviceBaseTest, RegisterAndUseSdo)
{
  device_.RegisterSdoResponseQueue(sdo_response_queue_);

  fastcat::DeviceCmd cmd;

  cmd.name                             = "patron";
  cmd.type                             = fastcat::ASYNC_SDO_WRITE_CMD;
  cmd.async_sdo_write_cmd.sdo_index    = 0xDEAD;
  cmd.async_sdo_write_cmd.sdo_subindex = 3;
  cmd.async_sdo_write_cmd.data_type    = JSD_SDO_DATA_U16;
  cmd.async_sdo_write_cmd.data.as_u16  = 0xBEEF;
  cmd.async_sdo_write_cmd.app_id       = 123;

  // Calling this for offline EtherCAT devices should
  // push SDO responses immediate onto the sdo response queue
  EXPECT_TRUE(device_.Write(cmd));
  EXPECT_FALSE(sdo_response_queue_->empty());

  {
    fastcat::SdoResponse resp;
    resp = sdo_response_queue_->front();
    sdo_response_queue_->pop();

    EXPECT_EQ(0, resp.device_name.compare("patron"));
    EXPECT_EQ(resp.response.sdo_index, 0xDEAD);
    EXPECT_EQ(resp.response.sdo_subindex, 3);
    EXPECT_EQ(resp.response.data.as_u16, 0xBEEF);
    EXPECT_EQ(resp.response.app_id, 123);

    EXPECT_TRUE(resp.response.success);
  }
}

}  // namespace
