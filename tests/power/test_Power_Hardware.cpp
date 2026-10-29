#include "test_power_common.h"

extern "C" void usb_reconnect_work_handler(struct k_work *work);

TEST_F(PowerManagementTestSuite, UsbReconnectCoverage) {
    // Explicit call to anonymous namespace bound work handler
    extern struct k_work_delayable usb_reconnect_work;
    if (usb_reconnect_work.work.handler) {
        usb_reconnect_work.work.handler(&usb_reconnect_work.work);
    }
}

TEST_F(PowerManagementTestSuite, TolerantWrappersDirectCoverage) {
    // 1. Cover nullptr short-circuits
    EXPECT_TRUE(pm_device_suspend_tolerant(nullptr, "NULL_DEV"));
    EXPECT_TRUE(pm_device_resume_tolerant(nullptr, "NULL_DEV"));
    
    // 2. Cover the LOG_ERR branch on a non-ignored error (-EIO)
    mocks.pm_adc_resume_ret = -EIO;
    EXPECT_FALSE(pm_device_resume_tolerant(dummy_adc, "ADC"));
    mocks.pm_adc_resume_ret = 0;
}
