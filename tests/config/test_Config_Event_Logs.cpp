// test_Config_Event_Logs.cpp
// Tests for the ConfigStore event log (Persistent_Configuration_System.cpp): getStoredLogCount() / getLogEntry() on an empty or
// uninitialized store, flash read errors, skipped keys and bad lengths, valid entries, and FAULT handling at entry and mid-traversal.

#include "test_config_common.h"

// Helper: append one valid FULL_CHARGE_LOG record to the mock flash.
static void injectValidLogEntry() {
    FcbRecordHeader header{};
    header.key = ConfigKey::FULL_CHARGE_LOG;
    header.length = sizeof(ConfigStore::LogEntry);
    ConfigStore::LogEntry entry{};

    fcb_entry loc{};
    loc.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(header) + sizeof(ConfigStore::LogEntry), &loc);
    flash_area_write(nullptr, loc.fe_elem_off, &header, sizeof(header));
    flash_area_write(nullptr, loc.fe_elem_off + sizeof(header), &entry, sizeof(ConfigStore::LogEntry));
}

TEST_F(ConfigStoreTestSuite, LogFunctionsFailWhenUninitialized) {
    ConfigStore& config = ConfigStore::getInstance();
    EXPECT_EQ(config.getStoredLogCount(), 0);
    ConfigStore::LogEntry entry{};
    EXPECT_FALSE(config.getLogEntry(0, entry));
}

// NEW TEST: Covers the read-failure branch during FCB transversal 
TEST_F(ConfigStoreTestSuite, LogRetrievalHandlesFlashReadErrors) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    FcbRecordHeader header{};
    header.key = ConfigKey::FULL_CHARGE_LOG;
    header.length = sizeof(ConfigStore::LogEntry);
    ConfigStore::LogEntry entry1{};

    fcb_entry loc1{};
    loc1.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(header) + sizeof(ConfigStore::LogEntry), &loc1);
    flash_area_write(nullptr, loc1.fe_elem_off, &header, sizeof(header));
    flash_area_write(nullptr, loc1.fe_elem_off + sizeof(header), &entry1, sizeof(ConfigStore::LogEntry));

    // Inject read failure
    mock_flash_read_fail = true; 

    EXPECT_EQ(config.getStoredLogCount(), 0);
    ConfigStore::LogEntry read_entry{};
    EXPECT_FALSE(config.getLogEntry(0, read_entry));
}

// NEW TEST: Validates the branch that skips corrupted lengths and unrecognized keys
TEST_F(ConfigStoreTestSuite, LogRetrievalIgnoresOtherKeysAndBadLengths) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    // Entry 1: Wrong Key
    FcbRecordHeader h1{};
    h1.key = ConfigKey::ALARM_THRESHOLD_BASE;
    h1.length = sizeof(ConfigStore::LogEntry);
    fcb_entry loc1{};
    loc1.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(h1) + sizeof(ConfigStore::LogEntry), &loc1);
    flash_area_write(nullptr, loc1.fe_elem_off, &h1, sizeof(h1));

    // Entry 2: Correct Key, Wrong Length
    FcbRecordHeader h2{};
    h2.key = ConfigKey::FULL_CHARGE_LOG;
    h2.length = 9999;
    fcb_entry loc2{};
    loc2.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(h2) + 4, &loc2);
    flash_area_write(nullptr, loc2.fe_elem_off, &h2, sizeof(h2));

    EXPECT_EQ(config.getStoredLogCount(), 0);
    ConfigStore::LogEntry read_entry{};
    EXPECT_FALSE(config.getLogEntry(0, read_entry));
}

TEST_F(ConfigStoreTestSuite, LogRetrievalWorks) {
    ConfigStore& config = ConfigStore::getInstance();
    config.init();

    // Manually inject 2 valid log entries directly into the FCB byte structure
    FcbRecordHeader header{};
    header.key = ConfigKey::FULL_CHARGE_LOG;
    header.length = sizeof(ConfigStore::LogEntry);
    
    ConfigStore::LogEntry entry1{};
    ConfigStore::LogEntry entry2{};

    // Mocking Entry 1
    fcb_entry loc1{};
    loc1.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(header) + sizeof(ConfigStore::LogEntry), &loc1);
    flash_area_write(nullptr, loc1.fe_elem_off, &header, sizeof(header));
    flash_area_write(nullptr, loc1.fe_elem_off + sizeof(header), &entry1, sizeof(ConfigStore::LogEntry));

    // Mocking Entry 2
    fcb_entry loc2{};
    loc2.fe_sector = (void*)1;
    fcb_append(nullptr, sizeof(header) + sizeof(ConfigStore::LogEntry), &loc2);
    flash_area_write(nullptr, loc2.fe_elem_off, &header, sizeof(header));
    flash_area_write(nullptr, loc2.fe_elem_off + sizeof(header), &entry2, sizeof(ConfigStore::LogEntry));

    EXPECT_EQ(config.getStoredLogCount(), 2);

    ConfigStore::LogEntry read_entry{};
    EXPECT_TRUE(config.getLogEntry(0, read_entry));
    EXPECT_TRUE(config.getLogEntry(1, read_entry));
    
    // Out-of-bounds check
    EXPECT_FALSE(config.getLogEntry(2, read_entry));
}

// getStoredLogCount()/getLogEntry(): early exit when initialized but FAULT.
TEST_F(ConfigStoreTestSuite, LogFunctionsReturnEarlyWhenFaultedAtEntry) {
    ConfigStore& config = ConfigStore::getInstance();
    ASSERT_TRUE(config.init());

    sys_context.current_state = SystemState::FAULT;

    EXPECT_EQ(config.getStoredLogCount(), 0U);
    ConfigStore::LogEntry entry{};
    EXPECT_FALSE(config.getLogEntry(0, entry));
}

// FAULT appearing *during* the traversal (checked inside the while loops).
TEST_F(ConfigStoreTestSuite, LogCountAbortsWhenFaultOccursMidTraversal) {
    ConfigStore& config = ConfigStore::getInstance();
    ASSERT_TRUE(config.init());
    injectValidLogEntry();
    ASSERT_EQ(config.getStoredLogCount(), 1U);   // sanity: works while healthy

    mock_trip_fault_on_getnext = true;           // FAULT is raised by the first fcb_getnext()
    EXPECT_EQ(config.getStoredLogCount(), 0U);
}

TEST_F(ConfigStoreTestSuite, GetLogEntryAbortsWhenFaultOccursMidTraversal) {
    ConfigStore& config = ConfigStore::getInstance();
    ASSERT_TRUE(config.init());
    injectValidLogEntry();

    ConfigStore::LogEntry entry{};
    ASSERT_TRUE(config.getLogEntry(0, entry));   // sanity: works while healthy

    mock_trip_fault_on_getnext = true;
    EXPECT_FALSE(config.getLogEntry(0, entry));
}
