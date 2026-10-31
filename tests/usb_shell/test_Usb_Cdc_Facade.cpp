// test_Usb_Cdc_Facade.cpp
// Tests for UsbCdcFacade (USB_CDC_Virtual_COM_Shell_Interface.cpp): init, DTR/cable detection, transmit,
// the UART RX interrupt + ring buffer, and readLine() (line endings, backspace, overflow).

#include "test_usb_shell_common.h"

TEST_F(UsbShellTestSuite,InitSuccess){
    UsbCdcFacade fresh; testing::internal::CaptureStdout();
    EXPECT_TRUE(fresh.init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("[INF] USB CDC Facade Initialized"),std::string_view::npos);
}

TEST_F(UsbShellTestSuite,InitHardwareNotReady){
    force_device_not_ready=true; UsbCdcFacade fresh; testing::internal::CaptureStdout();
    EXPECT_FALSE(fresh.init());
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[ERR] Failed to enable USB"),std::string_view::npos);
    force_device_not_ready=false;
}

TEST_F(UsbShellTestSuite,InitCallbackFails){
    mock_uart_callback_fail=true; UsbCdcFacade fresh;
    EXPECT_FALSE(fresh.init());
    mock_uart_callback_fail=false;
}

TEST_F(UsbShellTestSuite,InitTwice){
    ASSERT_TRUE(facade.init());
    mock_uart_callback_fail=true; EXPECT_TRUE(facade.init());
    mock_uart_callback_fail=false;
}

TEST_F(UsbShellTestSuite,DtrConnectDisconnect){
    // DTR low while attached: stays closed (no edge)
    mock_dtr_state=0; EXPECT_FALSE(facade.isConnected());
    EXPECT_FALSE(facade.isConnected());

    // DTR rises: open edge, logged once
    mock_dtr_state=1; testing::internal::CaptureStdout();
    EXPECT_TRUE(facade.isConnected());
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[INF] USB Terminal Opened (DTR High)"),std::string_view::npos);

    // DTR still high: steady state, no further log
    testing::internal::CaptureStdout();
    EXPECT_TRUE(facade.isConnected());
    EXPECT_EQ(testing::internal::GetCapturedStdout().find("USB Terminal"),std::string_view::npos);

    // DTR falls: close edge
    mock_dtr_state=0; testing::internal::CaptureStdout();
    EXPECT_FALSE(facade.isConnected());
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[WRN] USB Terminal Closed (DTR Low)"),std::string_view::npos);
}

TEST_F(UsbShellTestSuite,UsbCableDetached){
    UsbCdcFacade f; ASSERT_TRUE(f.init());

    // Cable never attached and terminal never opened -> dtr_ready already false
    g_usb_connected=false;
    EXPECT_FALSE(f.isConnected());
    EXPECT_FALSE(f.dtr_ready);

    // Attach + open terminal
    g_usb_connected=true; mock_dtr_state=1;
    EXPECT_TRUE(f.isConnected());
    EXPECT_TRUE(f.dtr_ready);

    // Cable pulled while the terminal was open -> RX disabled, dtr_ready cleared
    g_usb_connected=false;
    EXPECT_FALSE(f.isConnected());
    EXPECT_FALSE(f.dtr_ready);

    // Re-attach: terminal re-opens
    g_usb_connected=true;
    EXPECT_TRUE(f.isConnected());
}

TEST_F(UsbShellTestSuite,LineCtrlErrorHandling){
    mock_dtr_state=1;

    // Hard error (-1): neither 0 nor ENOTSUP/ENOSYS -> treated as "no terminal"
    mock_uart_line_ctrl_fail=true;
    EXPECT_FALSE(facade.isConnected());
    mock_uart_line_ctrl_fail=false;

    // Driver without DTR support (-ENOTSUP): assume terminal present
    mock_uart_line_ctrl_ret=-ENOTSUP;
    EXPECT_TRUE(facade.isConnected());

    // Hard error while open -> closes again
    mock_uart_line_ctrl_fail=true;
    EXPECT_FALSE(facade.isConnected());
    mock_uart_line_ctrl_fail=false;

    // Driver without line-ctrl at all (-ENOSYS): assume terminal present
    mock_uart_line_ctrl_ret=-ENOSYS;
    EXPECT_TRUE(facade.isConnected());

    // Back to a working driver with DTR low
    mock_uart_line_ctrl_ret=0; mock_dtr_state=0;
    EXPECT_FALSE(facade.isConnected());
}

TEST_F(UsbShellTestSuite,Transmit){
    mock_dtr_state=1; mock_tx_index=0; facade.transmit("AB");
    ASSERT_EQ(mock_tx_index,2u); EXPECT_EQ(mock_tx_buffer[0],'A'); EXPECT_EQ(mock_tx_buffer[1],'B');
    mock_dtr_state=0; mock_tx_index=0; facade.transmit("Data"); EXPECT_EQ(mock_tx_index,0u);
    mock_dtr_state=1; mock_tx_index=0; facade.transmit(""); EXPECT_EQ(mock_tx_index,0u);
    g_usb_connected=false; mock_tx_index=0; facade.transmit("Data"); EXPECT_EQ(mock_tx_index,0u);
}

TEST_F(UsbShellTestSuite,IrqUpdateFails){
    mock_uart_irq_update_fail=true; ASSERT_TRUE(facade.init());
    if(mock_uart_irq_cb) mock_uart_irq_cb(dummy_uart_dev,mock_uart_cb_data);
    mock_uart_irq_update_fail=false;
}

TEST_F(UsbShellTestSuite,IrqNoRxReady){
    UsbCdcFacade local; ASSERT_TRUE(local.init());
    mock_rx_head=mock_rx_tail=0;
    mock_uart_irq_cb(dummy_uart_dev,mock_uart_cb_data);
}

TEST_F(UsbShellTestSuite,RingBufferAndOverflow){
    UsbCdcFacade local; ASSERT_TRUE(local.init());

    inject_mock_uart_data("hello\n");
    std::array<char,MAX_CMD_LEN> cmd;
    ASSERT_TRUE(local.readLine(cmd)); EXPECT_STREQ(cmd.data(),"hello");

    std::array<char, 500> arr_X; arr_X.fill('X');
    inject_mock_uart_data(std::string_view(arr_X.data(), arr_X.size()));
    EXPECT_GT(local.overflow_count.load(),0u);
    EXPECT_GT(local.dropped_bytes.load(),0u);

    inject_mock_uart_data("B");

    local.overflow_logged.store(false);
    testing::internal::CaptureStdout();
    std::array<char, 300> arr_Y; arr_Y.fill('Y');
    inject_mock_uart_data(std::string_view(arr_Y.data(), arr_Y.size()));
    EXPECT_NE(testing::internal::GetCapturedStdout().find("USB RX overflow"),std::string_view::npos);

    local.overflow_logged.store(true);
    local.rx_tail.store(100); local.rx_head.store(99);
    testing::internal::CaptureStdout();
    inject_mock_uart_data("Z");
    EXPECT_EQ(testing::internal::GetCapturedStdout().find("USB RX overflow"),std::string_view::npos);
}

TEST_F(UsbShellTestSuite, BackspaceHandling) {
    UsbCdcFacade local;
    ASSERT_TRUE(local.init());
    mock_rx_head = mock_rx_tail = 0;
    
    inject_mock_uart_data("a\b");
    inject_mock_uart_data("b\n");
    
    std::array<char, MAX_CMD_LEN> cmd;
    ASSERT_TRUE(local.readLine(cmd));
    EXPECT_STREQ(cmd.data(), "b");

    inject_mock_uart_data("\b\n");
    ASSERT_TRUE(local.readLine(cmd));
    EXPECT_STREQ(cmd.data(), "");

    inject_mock_uart_data("x\x7Fy\n");
    ASSERT_TRUE(local.readLine(cmd));
    EXPECT_STREQ(cmd.data(), "y");
}

TEST_F(UsbShellTestSuite, ReadLineVariants) {
    std::array<char, MAX_CMD_LEN> cmd;

    EXPECT_FALSE(facade.readLine(cmd));

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("partial");
        EXPECT_FALSE(f.readLine(cmd));
        inject_mock_uart_data("\n");
        ASSERT_TRUE(f.readLine(cmd));
        EXPECT_STREQ(cmd.data(), "partial");
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN - 1> arr_A; arr_A.fill('A');
        inject_mock_uart_data(std::string_view(arr_A.data(), arr_A.size()));
        EXPECT_FALSE(f.readLine(cmd));
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        std::array<char, 150> arr_B; arr_B.fill('B');
        inject_mock_uart_data(std::string_view(arr_B.data(), arr_B.size()));
        EXPECT_FALSE(f.readLine(cmd));
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("\r\nstatus\r\n");
        // "\r\n" is absorbed as a single terminator -> one empty line, then "status"
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "");
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "status");
        EXPECT_FALSE(f.readLine(cmd));
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("reboot\r");
        ASSERT_TRUE(f.readLine(cmd));
        EXPECT_STREQ(cmd.data(), "reboot");
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("a\rb\n");
        ASSERT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "a");
        ASSERT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "b");
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN> arr_C; arr_C.fill('C'); arr_C.back() = '\n';
        inject_mock_uart_data(std::string_view(arr_C.data(), arr_C.size()));
        EXPECT_TRUE(f.readLine(cmd));
        EXPECT_EQ(strlen(cmd.data()), MAX_CMD_LEN - 1);
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN> wcmd;

        std::array<char, 101> a; a.fill('D'); a.back() = '\n';
        std::array<char, 101> b; b.fill('E'); b.back() = '\n';
        std::array<char, 101> c; c.fill('F'); c.back() = '\n';

        std::array<char, 101> a_cmp; a_cmp.fill('D'); a_cmp.back() = '\0';
        std::array<char, 101> b_cmp; b_cmp.fill('E'); b_cmp.back() = '\0';
        std::array<char, 101> c_cmp; c_cmp.fill('F'); c_cmp.back() = '\0';

        inject_mock_uart_data(std::string_view(a.data(), a.size()));
        EXPECT_TRUE(f.readLine(wcmd));
        EXPECT_STREQ(wcmd.data(), a_cmp.data());

        inject_mock_uart_data(std::string_view(b.data(), b.size()));
        EXPECT_TRUE(f.readLine(wcmd));
        EXPECT_STREQ(wcmd.data(), b_cmp.data());

        inject_mock_uart_data(std::string_view(c.data(), c.size()));
        EXPECT_TRUE(f.readLine(wcmd));
        EXPECT_STREQ(wcmd.data(), c_cmp.data());
    }
}

TEST_F(UsbShellTestSuite, OverlongCommandWithEol) {
    UsbCdcFacade local;
    ASSERT_TRUE(local.init());
    mock_rx_head = mock_rx_tail = 0;
    std::array<char, MAX_CMD_LEN> cmd;
    std::array<char, 151> arr_X; arr_X.fill('X'); arr_X.back() = '\n';
    inject_mock_uart_data(std::string_view(arr_X.data(), arr_X.size()));
    EXPECT_FALSE(local.readLine(cmd));
}
