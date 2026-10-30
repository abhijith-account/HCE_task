# HCE_task — Zephyr RTOS Safety-Critical Medical Device Firmware (`medical_firmware`)

**Repository:** [abhijith-account/HCE_task](https://github.com/abhijith-account/HCE_task)
**Target RTOS:** [Zephyr RTOS v4.4.0](https://www.google.com/search?q=https://github.com/zephyrproject-rtos/zephyr)
**Language Standard:** C++20 (`CONFIG_STD_CPP20=y`) & C11
**Target Platforms:** STMicroelectronics **STM32 Nucleo-F446RE** (ARM Cortex-M4F @ 96 MHz) & **ARM MPS2+ AN386** (QEMU Cortex-M4 Emulation)

## 1. Executive Overview

`HCE_task` (`medical_firmware`) is a deterministic, fault-tolerant C++20 embedded firmware application built on [Zephyr RTOS v4.4.0](https://www.google.com/search?q=https://github.com/zephyrproject-rtos/zephyr) for a multi-channel medical infusion pump and patient telemetry controller.

The firmware is engineered around five strict safety-critical and low-power design pillars:
- **Zero Dynamic Heap Allocation (`CONFIG_HEAP_MEM_POOL_SIZE=0`)**: All runtime objects, sensor pipeline commands (`ICommand`), and I2C register cache entries (`CacheEntry`) are allocated deterministically from static, mutex-guarded memory pools (`StaticPool<T, N>`) using placement `new`.
- **Hardware Memory & Stack Isolation**: Enforces ARM MPU protection (`CONFIG_ARM_MPU=y`), hardware stack overflow guards (`CONFIG_HW_STACK_PROTECTION=y`), compiler stack canaries (`CONFIG_STACK_CANARIES=y`), and zero-heap C/C++ standard library configuration (`CONFIG_NEWLIB_LIBC_MIN_REQUIRED_HEAP_SIZE=0`).
- **3-Tier Hardware Power Profile (`PM_STATE_ACTIVE` -> `PM_STATE_SUSPEND_TO_IDLE` -> `PM_STATE_SUSPEND_TO_RAM`)**: Autonomously transitions from **Active Mode** (`96 MHz` PLL, `> 50 mA` load) after `30 s` of inactivity to **Idle Mode** (`16 MHz` HSI, `VOS3`, `~5 mA` load) for `5 s`, and then into **Deep Sleep / STOP Mode** (`~234 µA` typical on main regulator, `60 s` total duration with `1 MHz` mid-STOP RTC wakeups every `25 s` to service the hardware watchdog).
- **Multi-Tier Fault Recovery & Safe Halt**: Combines a 3-stage I2C fault recovery pipeline (exponential retry, SCL bus reset, and a 64-entry reliability-scored fallback cache) with a spinlock-guarded system state machine (`INIT`, `RUNNING`, `FAULT`, `SAFE_HALT`) and a 9-thread heartbeat watchdog supervisor (`IWDG`).
- **100% Host Unit Test Coverage Gate**: Includes a 9-binary GoogleTest (`GTest`) suite organized into domain-specific test modules (`tests/power/`, `tests/battery/`, `tests/rtos/`, and root test suites) backed by checked-in Zephyr OS stubs (`tests/mocks/`), deterministic per-call fault injection, linker symbol wrapping (`--wrap`), and a strict `gcovr` gate requiring **100% line and 100% branch coverage** across `src/` (with pre-generated results in `coverage_report.html`).

## 2. Core Subsystems & Architecture

The codebase is organized into nine modular subsystems under `include/` and `src/`:

| Subsystem | Header & Implementation Paths | Key Architectural Mechanisms |
| :--- | :--- | :--- |
| **Static Memory & MISRA Compliance Layer** | `include/Static_Memory+MISRA_Compliance_Layer.h`<br>`src/Static_Memory+MISRA_Compliance_Layer.cpp` | Provides the `StaticPool<T, N>` template (`alignas(4)` byte array + boolean occupancy bitmap + `k_mutex`) with pointer-bounds validation on deallocation. Runs `memory_monitor_thread` to log unused stack watermarks across 11 RTOS threads every `30 s`. |
| **RTOS Command-Based Thread System** | `include/RTOS_Command_based_thread_system/`<br>`src/RTOS_Command_based_thread_system/` | Implements a priority-preemptive Producer -> Processor -> Logger pipeline (`Prio 6 -> 5 -> 4`) backed by a 16-slot `MaxCommandSize` static pool (`48 B`, `alignas(8)`) and two 10-deep `k_msgq` queues (`processor_queue`, `logger_queue`). |
| **Smart Battery System (BMS)** | `include/Smart_Battery_System/`<br>`src/Smart_Battery_System/` | Monitors a 3S Li-ion pack (`8700-12600 mV`, `2000 mAh`) via a TI **INA226** (`0x40`, `100 mΩ` shunt, `25 µA` current LSB, `819.2 mA` max measurable range) and 8x oversampled 12-bit ADC NTC thermistor (`PA0`, `-40 °C` to `125 °C` LUT). Fuses coulomb counting + 3S OCV curve interpolation using a 1D **Kalman Filter** and an EMA/hysteresis **FSM Classifier** (`IDLE`, `CHARGING`, `DISCHARGING`, `CUTOFF`). |
| **Power Management System (PMS)** | `include/Power_Management_System/`<br>`src/Power_Management_System/` | Singleton `PowerManager` notifying up to 8 `IPowerObserver` listeners (`beforeSleep`, `afterWakeup`, `sleepAborted`). Controls STM32 RCC clock scaling (`96 MHz` PLL -> `16 MHz` HSI in Idle -> `4 MHz` pre-STOP -> `1 MHz` mid-STOP service), USB OTG FS PHY power-down, GPIO `ANALOG` sleep pinmuxing, and RTC Alarm A wakeups. Escalates to system fault after 3 consecutive PM failures (`PM_FAILURE_FAULT_THRESHOLD = 3`). |
| **Device State Machine & Watchdog** | `include/Device_State_Machine+Watchdog.h`<br>`src/Device_State_Machine+Watchdog.cpp` | Spinlock-synchronized `DeviceContext` enforcing legal transitions between `INIT`, `RUNNING`, `FAULT`, and `SAFE_HALT`. Integrates `WatchdogTimer` (`&iwdg`) and `daly_watchdog_feed_hook`, verifying atomic heartbeats from 9 threads every `1000 ms`. |
| **Fault-Tolerant I2C Communication Layer** | `include/Fault_Tolerant_I2C_Communication_Layer.h`<br>`src/Fault_Tolerant_I2C_Communication_Layer.cpp` | Wraps 8/16/24/64-bit I2C register operations in `Result<T>`. Implements `RetryStrategy`, `BusResetStrategy`, and `FailSafeStrategy` backed by a 64-entry `StaticPool<CacheEntry, 64>` with `3000 ms` max age and reliability score decay (`100 -> 50`, `-10` per fallback read). |
| **Persistent Configuration System** | `include/Persistent_Configuration_System.h`<br>`src/Persistent_Configuration_System.cpp` | Manages append-only records in internal Flash via Zephyr's Flash Circular Buffer (`FCB`, magic `0x12345678`) across 2 sectors (`storage_partition`, `256 KB`). Guards writes with `FlashPowerGuard`, rotates sectors on `-ENOSPC`, provisions 5 infusion device slots (`Slots 1-5`), and runs a one-time 1,000-cycle Flash endurance test. |
| **USB CDC Virtual COM Shell Interface** | `include/USB_CDC_Virtual_COM_Shell_Interface.h`<br>`src/USB_CDC_Virtual_COM_Shell_Interface.cpp` | Interrupt-driven UART/CDC ACM facade (`256 B` lock-free atomic RX ring buffer, `32 B` chunked TX with `2 ms` inter-chunk gap) exposing the `med-device:~$ ` CLI (`status`, `set_rate`, `log_dump`, `reboot`) with compile-time slot bounds checking and hardware USB cable disconnect fault detection. |
| **RTOS Synchronization Layer** | `include/RTOS_Synchronization_Layer.h`<br>`src/RTOS_Synchronization_Layer.cpp` | C++ wrappers around `k_sem` (`ZephyrSemaphore`), `k_mutex` (`ZephyrMutex`), and `k_work_delayable` (`ZephyrWorkQueue`). Coordinates a 10-element `SharedHeartRateBuffer` between `heart_rate_producer_thread` and `display_consumer_thread`, plus a 1 Hz system status workqueue (`status_work`). |

## 3. RTOS Thread & Concurrency Architecture

All 12 threads are statically allocated at compile time via `K_THREAD_DEFINE` with explicit stack sizes, numeric priorities (`4` = highest priority, `15` = lowest application priority), and compile-time `static_assert` checks (`PrioProcessor < PrioProducer` to prevent queue starvation):

| Thread Handle (`k_tid_t`) | Entry Function | Stack Size | Zephyr Priority | Heartbeat Flag | Role & Execution Timing |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `logger_tid` | `logger_thread` | `1024 B` | `4` (Highest) | `g_logger_alive` | Dequeues `PrintCmd`, `PrintBME280Cmd`, and `PrintLPS22HBCmd` from `LOGGER_Q` (`500 ms` timeout) and emits formatted telemetry logs. |
| `processor_tid` | `processor_thread` | `1024 B` | `5` | `g_processor_alive` | Dequeues `SensorReadCmd` and `ComputeCmd` from `PROCESSOR_Q` (`500 ms` timeout), executes infusion rate calculations, and dispatches log jobs to `LOGGER_Q`. |
| `producer_tid` | `producer_thread` | `1024 B` | `6` | `g_producer_alive` | Initializes sensors and cycles every `500 ms` through `ProducerState::ReadBME` -> `ReadLPS` -> `ReadPAV`, enqueuing `SensorReadCmd` tasks. |
| `hr_prod_tid` | `heart_rate_producer_thread` | `256 B` | `8` | `g_hr_prod_alive` | Generates biometric heart-rate samples (`65-145 BPM` waveform) every `500 ms`, pushes to `hr_buffer` under `hr_buffer.mutex`, and signals `display_sem`. |
| `disp_cons_tid` | `display_consumer_thread` | `1024 B` | `9` | `g_disp_cons_alive` | Waits on `display_sem` (`500 ms` timeout), pops heart-rate samples from `hr_buffer`, and logs `[Display Consumer] Rendered Heart Rate: <val> bpm`. |
| `bms_comm_tid` | `bms_comm_thread` | `1024 B` | `10` | `g_bms_comm_alive` | Polls INA226 (`0x40`) and NTC thermistor every `1000 ms` with exponential backoff (`20 ms` -> `160 ms`, up to 3 retries), runs Kalman Filter, and updates `BmsCache`. |
| `battery_tid` | `battery_monitor_thread` | `1024 B` | `11` | `g_batt_mon_alive` | Evaluates `SbsBattery::processFSM()` every `1000 ms`, enforcing `CUTOFF_SOC_PCT` (`10%`), `REENABLE_SOC_PCT` (`15%`), and full-charge NVS logging (`100%`). |
| `mem_mon_tid` | `memory_monitor_thread` | `1024 B` | `12` | `g_mem_mon_alive` | Ticks every `1000 ms`; queries `k_thread_stack_space_get()` every `30 s` to report unused stack watermarks across all 11 operational threads. |
| `shell_tid` | `shell_thread` | `3072 B` | `13` | `g_shell_alive` | Polls `UsbCdcFacade::readLine()` every `50 ms` (`ShellPollMs`) and dispatches interactive CLI commands (`status`, `set_rate`, `log_dump`, `reboot`). |
| `pr_tid` | `power_monitor_thread` | `1024 B` | `14` | Monitored by `MEM_SYS` | Runs `PowerManager::processFSM()` and blocks on `pm_wake_sem` for the remaining `ACTIVE_TIMEOUT_MS` (`30 s`) or `IDLE_TIMEOUT_MS` (`5 s`) window (`K_NO_WAIT` in deep sleep). |
| `trace_tid` | `trace_logger_thread` | `1024 B` | `15` (Lowest) | Monitored by `MEM_SYS` | Drains the 4096-entry lockless context-switch ring buffer (`g_trace_buffer`) every `50 ms`, filtering out `< 1000 µs` I2C waits and `490-510 ms` sleeps to log real preemption delays. |
| `active_burn_tid` | `active_burn_fn` *(STM32 only)* | `768 B` | `15` (Lowest) | Suspended in Idle/Stop | Duty-cycled SRAM multiply/load/store burner (`10 ms` work / `90 ms` sleep) running exclusively in `ACTIVE` state to simulate hardware power load (`> 50 mA`). |

## 4. Deep-Dive: Power Management, Fault Recovery & Safety Loops

### 1. 3-State Low-Power Architecture (`PM_STATE_ACTIVE` -> `PM_STATE_SUSPEND_TO_IDLE` -> `PM_STATE_SUSPEND_TO_RAM`)

Implemented across `src/Power_Management_System/`:
- **`PM_STATE_ACTIVE` (`30 s` inactivity timeout, `> 50 mA` target on STM32)**:
  - Holds `PM_STATE_SUSPEND_TO_IDLE` policy lock, enables extra AHB1/AHB2/AHB3/APB1/APB2 peripheral clocks (`ACTIVE_*_EXTRA_CLOCKS`), and runs `active_burn_tid` at priority `15`.
  - Any sensor value change exceeding its activity threshold (`BME280: 0x1000`, `LPS22HB Press: 0x50`, `LPS22HB Temp: 0x10`, `PAV3015: 0x05`), CLI input, Flash write, or state transition calls `PowerManager::reportActivity()`, resetting the inactivity timer and waking the MCU if asleep.
- **`PM_STATE_SUSPEND_TO_IDLE` (`5 s` timeout, `~5 mA` target on STM32)**:
  - Flushes UART logs (`120 ms`), suspends all 10 background worker threads (`suspend_background_threads()`), and suspends UART, ADC, and I2C devices via `pm_device_action_run()`.
  - Downscales `SYSCLK` from the `96 MHz` PLL to the **`16 MHz` HSI oscillator** (`IDLE_HCLK_DIV = 1`) via a consolidated `MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2, ...)` prescaler update, recalculates `SysTick->LOAD` to preserve wall-clock `k_uptime_get_32()` accuracy, disables Flash prefetch/ICache/DCache (`FLASH_LATENCY_0`), powers down the PLL and HSE (`HSEON`/`HSEBYP`/`CSSON`), drops regulator voltage to **Scale 3 (`VOS3`)**, and gates APB clocks while keeping `IDLE_APB1_EXTRA_CLOCKS` active.
  - **Optimized Idle -> STOP Handoff (`s_idle_exit_to_stop`)**: When transitioning from `PM_STATE_SUSPEND_TO_IDLE` directly into `PM_STATE_SUSPEND_TO_RAM`, `IdleState::exit()` leaves the ADC and background threads parked rather than waking and immediately re-suspending them.
- **`PM_STATE_SUSPEND_TO_RAM` (`60 s` total sleep, `25 s` RTC wake intervals, `~234 µA` typical on STM32)**:
  - **Pre-Sleep Hardware Shutdown**: Downscales `HCLK` to `4 MHz` (`16 MHz HSI / 4`), detaches USB (`usb_dc_detach()`), soft-disconnects `DP` pull-up (`USB_OTG_DCTL_SDIS`), powers down the USB transceiver (`GCCFG = 0`), gates USB PHY/HCLK (`PCGCCTL |= 0x3`), disables `ADC1/2/3` and `ADC123_COMMON->CCR` (`VREFINT`/temp/VBAT off), disables `DAC`, powers down the **SYSCFG I/O Compensation Cell** (`SYSCFG_CMPCR_CMP_PD = 0`, saving `~150-200 µA`), and disables the **Backup Regulator** (`PWR_CSR_BRE = 0`, saving `~150 µA`).
  - **Zero-Contention GPIO Sleep Policy**: Snapshots all 8 ports (`GPIOA-GPIOH`), preserves `PC14`/`PC15` (LSE 32.768 kHz crystal), parks unused/SWD/JTAG debug (`PA13`/`PA14`/`PA15`, `PB3`/`PB4`) and USB pins in `ANALOG` (`No Pull`), holds open-drain `I2C` pins at their sampled level, and configures push-pull alternate-function pins as inputs with weak pull-up/pull-down matching the external idle level to prevent mA-class contention.
  - **Debugger Detection & `PRIMASK = 1` WFI Entry**: Clears `DBGMCU->CR`, `DWT->CTRL`, and `CoreDebug->DEMCR`, warning over UART if an SWD debugger is attached (`CoreDebug->DHCSR.C_DEBUGEN == 1`, which forces STM32 STOP to run as SLEEP at `~0.9 mA` until power-cycled with SWD disconnected). Masks interrupts (`PRIMASK = 1` per PM0214 Section 2.5.2) with only `EXTI` line 17 (RTC Alarm) unmasked before executing `__WFI()`.
  - **`1 MHz` Mid-STOP Watchdog Housekeeping**: On each `25 s` RTC wakeup prior to `60 s`, downscales `HCLK` to **`1 MHz` (`HSI / 16`)**, feeds `IWDG`, re-arms RTC Alarm A for the remaining sleep time (verifying `RTC_FLAG_ALRAF` clear with the DWT cycle counter), and re-enters `__WFI()`. Upon full wakeup, restores all clocks/registers and schedules `usb_reconnect_work` (`250 ms` delay) to re-attach USB CDC.

### 2. System State Machine & Watchdog Supervisor (`DeviceContext`)

Managed in `src/Device_State_Machine+Watchdog.cpp` under `k_spinlock` protection (`device_state_lock` and `wdt_access_lock`):
- **Legal Transition Matrix**:
  - `INIT` -> `RUNNING` | `FAULT` | `SAFE_HALT`
  - `RUNNING` -> `INIT` | `FAULT` | `SAFE_HALT`
  - `FAULT` -> `SAFE_HALT`
  - `SAFE_HALT` -> `INIT`
- **Illegal Transition Guard**: Any transition request outside the allowed matrix is rejected, logged as an error, and immediately forces `current_state = SystemState::SAFE_HALT`.
- **Watchdog Starvation & Fault Reset (`daly_watchdog_feed_hook`)**: Every `1000 ms`, the hook atomically checks and clears (`atomic_cas(&flag, 1, 0)`) all 9 thread liveness flags (`g_producer_alive`, `g_processor_alive`, `g_logger_alive`, `g_hr_prod_alive`, `g_disp_cons_alive`, `g_bms_comm_alive`, `g_batt_mon_alive`, `g_mem_mon_alive`, `g_shell_alive`). In `SystemState::FAULT`, watchdog feeding is intentionally withheld so the hardware `IWDG` triggers a full SoC reset (`WDT_FLAG_RESET_SOC`), whereas in `SystemState::SAFE_HALT`, threads block safely while keeping the watchdog fed until manual reset or battery recovery.

### 3. 3-Stage Fault-Tolerant I2C Pipeline & Sensor Fault Escalation

Implemented in `src/Fault_Tolerant_I2C_Communication_Layer.cpp` and `src/RTOS_Command_based_thread_system/Sensor_Commands.cpp`:
1. **Stage 1 — Retry (`RetryStrategy`)**: Logs a warning and performs a `10 ms` backoff retry on transient errors (`NACK`, `ARBITRATION_LOST`).
2. **Stage 2 — Hardware Bus Recovery (`BusResetStrategy`)**: Invoked on bus lockups (`-ETIMEDOUT -> TIMEOUT`, `-EBUSY -> BUS_BUSY`) to toggle SCL and recover the bus.
3. **Stage 3 — Reliability-Scored Cache Fallback (`FailSafeStrategy`)**: Queries `StaticPool<CacheEntry, 64>` for `[sensor_addr, reg_addr]`. Returns the cached value only if `is_calibrated == true`, `age <= 3000 ms`, and `reliability_score >= 50` (`CACHE_RELIABILITY_MIN_FOR_FALLBACK`), decrementing `reliability_score` by `10` (`CACHE_RELIABILITY_DECAY_PER_USE`) on each fallback use.
4. **3-Strike Sensor Disconnect Escalation**: Once a sensor's cache entry expires or its reliability score drops below `50`, `SensorReadCmd::execute()` increments a per-sensor failure counter (`bme_failures`, `lps_failures`, `pav_failures`). Upon **3 consecutive uncached read failures**, it calls `SystemObjects::context().triggerFault("I2C Sensor Disconnected")`, prompting `producer_thread` to disable all sensors and lock the system in `SystemState::SAFE_HALT`.

### 4. Smart Battery Estimation, Disconnect Detection & BMS Watchdog (`SbsBattery`)

Implemented in `src/Smart_Battery_System/`:
- **Pack Disconnect & Plausibility Checks**: Detects physical battery disconnect when `pack_mv < 4000 mV` (`MIN_VALID_VOLTAGE_MV`), or `< 8700 mV` (`PACK_MIN_VOLTAGE_MV`) on initial connect or after a single-step drop `> 2000 mV`, cleanly zeroing cached telemetry and resetting `FsmClassifier`. Rejects implausible readings (`> 20000 mV` or `|I| > 8000 mA`) and single-step spikes (`ΔV > 2000 mV`, `ΔI > 8000 mA`, `ΔT > 20.0 °C` up to `MAX_CONSECUTIVE_JUMP_REJECTS = 3`).
- **1D Kalman Filter + OCV Resync**: Fuses coulomb counting (`accumulated_uAh`) with a 10-point 3S Open-Circuit Voltage lookup table (`8700 mV = 0%` to `12600 mV = 100%`) using IR drop compensation (`150 mΩ`) and adaptive measurement noise (`KF_MEAS_NOISE_REST = 2.0` vs. `KF_MEAS_NOISE_ACTIVE = 50.0`). Automatically re-seeds/resyncs the coulomb counter at full charge (`>= 12500 mV` at rest), full discharge (`<= 8800 mV` at rest), or after `30 min` of continuous rest (`|shunt_uv| < 150 µV`).
- **Cutoff, Recovery & BMS Watchdog**: Enters `BatteryFSM::CUTOFF` and requests `SystemState::SAFE_HALT` when `SoC < 10%` (`CUTOFF_SOC_PCT`), automatically transitioning back to `SystemState::INIT` once `SoC > 15%` (`REENABLE_SOC_PCT`). Triggers a critical `"BMS Communication Watchdog"` fault after `2` consecutive I2C/thermistor failures, `5` consecutive mutex timeouts, or `> 5000 ms` without valid communication.

### 5. Flash Circular Buffer (`FCB`) & One-Time 1,000-Cycle Endurance Test

Implemented in `src/Persistent_Configuration_System.cpp` and `src/main.cpp`:
- Mounts Zephyr's `FCB` (`f_magic = 0x12345678`) across 2 sectors (`storage_partition`, `256 KB`). If unformatted (`-ENOMSG`), safely erases and formats the partition while feeding the watchdog (aborting immediately if in `SystemState::FAULT`), then seeds default device IDs (`1001-1005`) and alarm thresholds (`80, 85, 90, 95, 100`) across `Slots 1-5`.
- On first boot, `config.validateEndurance(ConfigKey::ALARM_THRESHOLD_BASE)` executes a **1,000 write/read-verify cycle endurance test** (writing `uint16_t` values `0` through `999` to key `300`, while per-slot thresholds use keys `301-305`). Once `999` is persisted, subsequent boots detect `read_val == 999` and skip the endurance loop automatically to preserve Flash lifespan.

## 5. Repository Directory Structure

```text
HCE_task/
├── .github/
│   └── workflows/
│       └── main.yml                  # GitHub Actions CI/CD pipeline (Nucleo build, QEMU run, GTest + 100% coverage)
├── boards/
│   ├── mps2_an386.conf               # Kconfig overrides for QEMU ARM Cortex-M4 emulation
│   └── nucleo_f446re.conf            # Kconfig overrides for STM32 Nucleo-F446RE hardware
├── include/
│   ├── Power_Management_System/
│   │   ├── Power_Internal.h          # Timeout constants (30s/5s/60s/25s), STM32 RCC clock masks, internal helpers
│   │   └── Power_Management_System.h # IPowerState, IPowerObserver, PowerManager singleton API
│   ├── RTOS_Command_based_thread_system/
│   │   ├── BME280.h                  # Bosch BME280 calibration & compensation math + I2C driver (0x76)
│   │   ├── Command_Core.h            # ICommand interface, 16-slot static pool, processor/logger queues
│   │   ├── LPS22HB.h                 # ST LPS22HB pressure/temperature accumulation & decoding (0x5C)
│   │   ├── PAV3015.h                 # Posifa PAV3015 quadratic airflow decoding & QEMU mock reader (0x28)
│   │   ├── Print_Commands.h          # PrintCmd, PrintBME280Cmd, PrintLPS22HBCmd formatting commands
│   │   ├── RTOS_Command_Based_Thread_System.h # Umbrella header for the command-based thread subsystem
│   │   ├── Sensor_Commands.h         # SensorReadCmd & ComputeCmd pipeline stages
│   │   ├── Sensor_Common.h           # SensorID, ReadLength (Word/Triple/Block), descriptors, log tags
│   │   ├── System_Objects.h          # ThreadSystemPowerObserver & global thread heartbeat flags
│   │   ├── Thread_Common.h           # Test-aware thread loop condition macros
│   │   └── Threads.h                 # Thread priorities, stack sizes, and entry point declarations
│   ├── Smart_Battery_System/
│   │   ├── Battery_Config.h          # 3S pack limits, Kalman noise tuning, OCV-to-SoC lookup table
│   │   ├── Battery_Types.h           # Strong unit types (Millivolts, Milliamps, Kelvin, Percent), BmsCache
│   │   ├── Fsm_classifier.h          # Shunt voltage EMA filter & hysteresis state classifier
│   │   ├── INA226_Driver.h           # TI INA226 register map (0x40), calibration constants, I2C driver
│   │   ├── Kalman_Filter.h           # 1D Kalman Filter predict/update equations for SoC estimation
│   │   ├── Retry_Policy.h            # Host-testable generic exponential backoff retry template
│   │   ├── Smart_Battery_System.h    # SbsBattery class, telemetry cache, fault/retry thresholds
│   │   └── Thermistor.h              # 12-bit ADC 8x oversampling & 34-point NTC LUT (-40 C to 125 C)
│   ├── Device_State_Machine+Watchdog.h # SystemState FSM, WatchdogTimer, DeviceContext
│   ├── Fault_Tolerant_I2C_Communication_Layer.h # Result<T>, I2CStrategy hierarchy, I2CManager
│   ├── Persistent_Configuration_System.h # ConfigKey, InfusionDeviceConfig, FlashPowerGuard, ConfigStore (FCB)
│   ├── RTOS_Synchronization_Layer.h   # ZephyrSemaphore, ZephyrMutex, ZephyrWorkQueue, SharedHeartRateBuffer
│   ├── Static_Memory+MISRA_Compliance_Layer.h # StaticPool<T, N> zero-heap allocator
│   └── USB_CDC_Virtual_COM_Shell_Interface.h # UsbCdcFacade, UsbShell CLI parser, status snapshot formatter
├── scripts/
│   └── check_coverage.sh             # gcovr script enforcing 100% line and 100% branch coverage
├── src/
│   ├── Power_Management_System/
│   │   ├── Power_ActiveIdleStates.cpp # ActiveState (clock boost + burner) & IdleState (16 MHz HSI) execution
│   │   ├── Power_Hardware.cpp        # STM32 USB OTG FS power-down, GPIO analog snapshots, clock scaling
│   │   ├── Power_Management_System.cpp # PowerManager core FSM, observer notification, reset cause logging
│   │   ├── Power_RTC.cpp             # RTC Alarm A scheduling, DWT-verified flag clearing, EXTI line 17
│   │   └── Power_StopState.cpp       # StopState suspend-to-RAM entry, 1 MHz 25s watchdog wake loop, exit recovery
│   ├── RTOS_Command_based_thread_system/
│   │   ├── BME280.cpp                # BME280 ROM calibration loader and temperature/pressure/humidity math
│   │   ├── Command_Core.cpp          # Static pool allocation and message queue dispatch
│   │   ├── Command_Size_Checks.cpp   # Compile-time static_assert size & alignment verification
│   │   ├── Consumer_Threads.cpp      # processor_thread and logger_thread implementations
│   │   ├── LPS22HB.cpp               # LPS22HB register init and two-stage pressure/temp accumulation
│   │   ├── PAV3015.cpp               # PAV3015 probe init and quadratic airflow decoding
│   │   ├── Print_Commands.cpp        # Deferred logging formatters for sensor measurements
│   │   ├── Producer_Thread.cpp       # Round-robin sensor polling state machine (ReadBME -> ReadLPS -> ReadPAV)
│   │   ├── Sensor_Commands.cpp       # Hardware I2C reads, 3-strike fault escalation, and compute chaining
│   │   ├── System_Objects.cpp        # System service singletons and power observer registration
│   │   └── Thread_Trace.cpp          # 4096-entry lockless context-switch trace buffer & logger thread
│   ├── Smart_Battery_System/
│   │   ├── Battery_Runtime.cpp       # BMS communication & monitoring RTOS threads
│   │   ├── Fsm_classifier.cpp        # Exponential moving average & hysteresis state transitions
│   │   ├── INA226_Driver.cpp         # INA226 register reads/writes with big-endian byte swapping
│   │   ├── Kalman_Filter.cpp         # Covariance projection, Kalman gain, and [0%, 100%] SoC clamping
│   │   ├── Smart_Battery_System.cpp  # SbsBattery polling, jump rejection, coulomb resync, cache publishing
│   │   └── Thermistor.cpp            # ADC channel setup, 8x oversampled reads, NTC LUT interpolation
│   ├── Device_State_Machine+Watchdog.cpp # Spinlock state transitions, thread health checks, IWDG feeding
│   ├── Fault_Tolerant_I2C_Communication_Layer.cpp # I2C recovery strategies and 64-entry reliability-scored cache
│   ├── Persistent_Configuration_System.cpp # FCB init, slot provisioning, and 1,000-cycle Flash endurance test
│   ├── RTOS_Synchronization_Layer.cpp # Heart-rate producer/consumer threads and 1 Hz status workqueue
│   ├── Static_Memory+MISRA_Compliance_Layer.cpp # 30s stack watermark monitor thread and static pool instantiation
│   ├── USB_CDC_Virtual_COM_Shell_Interface.cpp # Interrupt-driven UART/CDC ring buffer and CLI command handlers
│   ├── main.cpp                      # Boot sequence, USB DTR handshake, FCB config load, workqueue start
│   └── qemu_sensor_mocks.c           # Emulated I2C/ADC sensors (INA226, LPS22HB, B3950 NTC, BME280 w/ 5s fault)
├── svd/
│   └── STM32F446.svd                 # CMSIS System View Description for STM32F446 peripheral debugging
├── tests/
│   ├── CMakeLists.txt                # Host GTest build configuration linking checked-in Zephyr mocks & modular suites
│   ├── battery/                      # Modular Smart Battery System unit test suite (run_battery_tests)
│   │   ├── test_Battery_Monitor_Thread.cpp
│   │   ├── test_Battery_Runtime.cpp
│   │   ├── test_Bms_Comm_Thread.cpp
│   │   ├── test_Sensor_Drivers.cpp
│   │   ├── test_Smart_Battery_FSM.cpp
│   │   ├── test_Smart_Battery_Fetch.cpp
│   │   ├── test_Smart_Battery_Poll.cpp
│   │   ├── test_Smart_Battery_Soc.cpp
│   │   ├── test_Smart_Battery_System.cpp
│   │   ├── test_battery_common.h
│   │   └── test_battery_mocks.cpp
│   ├── mocks/                        # Checked-in Zephyr OS & hardware peripheral mocks for reproducible CI/host builds
│   │   ├── mock_globals.cpp          # Global mock state & deterministic fault injection variables
│   │   └── include/zephyr/           # Stub headers (kernel, device, drivers, fs/fcb, kvss/nvs, logging, pm, storage, sys, usb)
│   ├── power/                        # Modular Power Management System unit test suite (run_pm_tests)
│   │   ├── test_Power_ActiveIdleStates.cpp
│   │   ├── test_Power_Hardware.cpp
│   │   ├── test_Power_Integration.cpp
│   │   ├── test_Power_Management_System.cpp
│   │   ├── test_Power_Singleton.cpp
│   │   ├── test_Power_StopState.cpp
│   │   ├── test_Power_patch.h
│   │   ├── test_power_common.h
│   │   └── test_power_mocks.cpp
│   ├── rtos/                         # Modular RTOS Command Pipeline unit test suite (run_command_tests)
│   │   ├── test_Command_Core.cpp
│   │   ├── test_Compute_Cmd.cpp
│   │   ├── test_Consumer_Threads.cpp
│   │   ├── test_Print_Commands.cpp
│   │   ├── test_Producer_Thread.cpp
│   │   ├── test_Sensor_Math.cpp
│   │   ├── test_Sensor_Read_Cmd.cpp
│   │   ├── test_System_Objects.cpp
│   │   ├── test_Thread_Trace.cpp
│   │   ├── test_rtos_common.h
│   │   └── test_rtos_mocks.cpp
│   ├── test_I2C_Fault_Recovery.cpp   # Unit tests for I2C retry, bus reset, and fail-safe caching
│   ├── test_config_store.cpp         # Unit tests for FCB persistence, rotation, and slot validation
│   ├── test_state_machine.cpp        # Unit tests for DeviceContext transitions and watchdog supervision
│   ├── test_static_memory.cpp        # Unit tests for StaticPool exhaustion, bounds check, and stack monitor
│   ├── test_synchronization.cpp      # Unit tests for mutex, semaphore, workqueue, and heart-rate buffer
│   └── test_usb_shell.cpp            # Unit tests for USB CDC RX ring buffer and CLI command parsing
├── CMakeLists.txt                    # Top-level Zephyr application build script
├── Kconfig                           # Custom Kconfig symbols (LOG_PREEMPTION_DELAY, SOC_MPS2_AN386, LOG_EXECUTION_CYCLES)
├── coverage_report.html              # Generated gcovr HTML coverage report (100% line & branch coverage)
├── mps2_an386.overlay                # Devicetree overlay for QEMU Cortex-M4 (emulated I2C, ADC, RTC, 512 KB Flash)
├── nucleo_f446re.overlay             # Devicetree overlay for STM32F446RE (clocks, USB CDC, I2C1, ADC1, sleep pinctrl)
├── prj.conf                          # Base Zephyr kernel, C++20, MPU, FCB, logging, and peripheral configuration
└── west.yml                          # West manifest pinned to zephyrproject-rtos v4.4.0
```

## 6. Hardware Targets & Devicetree Configuration

The project builds cleanly for both physical STM32 hardware and QEMU ARM Cortex-M4 emulation using board-specific configurations in `boards/` and devicetree overlays (`nucleo_f446re.overlay` and `mps2_an386.overlay`):

| Feature / Peripheral | Primary Hardware (`nucleo_f446re`) | QEMU Emulation (`mps2/an386`) |
| :--- | :--- | :--- |
| **MCU & System Clock** | STM32F446RE (ARM Cortex-M4F @ `96 MHz` via `8 MHz` HSE bypass PLL; `AHB/1`, `APB1/4`, `APB2/2`) | ARM MPS2+ AN386 (QEMU Cortex-M4 @ `1000` ticks/sec, `CONFIG_SOC_MPS2_AN386=y` selecting `HAS_PM`) |
| **Console & Shell UART** | `usart3` (`PC10` TX, `PC11` RX @ `115200` baud) & `usb_otg_fs` (`cdc_acm_uart0`: `PA11` DM, `PA12` DP, `PA9` VBUS, `2048 B` TX/RX FIFOs) | `uart0` (`zephyr,console`) & `uart1` (`cdc-acm-uart0` alias) |
| **I2C Bus (`i2c1`)** | Hardware `&i2c1` (`PB8` SCL, `PB9` SDA @ `100 kHz`); `&i2c2` and `&i2c3` explicitly disabled | `zephyr,i2c-emul-controller` (`i2c_emul0` @ `0x50010000`) backed by `src/qemu_sensor_mocks.c` |
| **Sensors on `i2c1`** | Bosch **BME280** (`0x76`), ST **LPS22HB** (`0x5C`), Posifa **PAV3015** (`0x28`), TI **INA226** (`0x40`) | Emulated **INA226** (`0x40`, `11.1 V` / `100 mA`), **LPS22HB** (`0x5C`, `24.0 °C`), **BME280** (`0x76`, injects `-EIO` disconnect after `5 s`), and **PAV3015** C mock (`0x28`) |
| **Thermistor ADC** | `&adc1` Channel 0 (`PA0`, 12-bit, internal ref, prescaler `2`) | `zephyr,adc-emul` (`adc_emul0` Channel 0, initialized via `SYS_INIT`) |
| **RTC & Sleep Mode** | `&rtc` driven by LSE (`clk_lse`) with `suspend-to-ram` (`stop0`, `min-residency = 100 ms`, `exit-latency = 2 ms`) | `zephyr,rtc-emul` (`rtc_emul0`, 2 alarms) + `mps2_stop_timer` |
| **Watchdog Timer** | STM32 Independent Watchdog (`&iwdg`) | CMSDK APB Watchdog (`&wdog0`) |
| **Flash Layout (`FCB`)** | Internal `&flash0`: `code_partition` (`0x00000000`, `256 KB`) + `storage_partition` (`0x00040000`, `256 KB`) | `zephyr,sim-flash` (`512 KB` @ `0x50000000`, `erase-block-size = 128 KB`, `write-block-size = 1`) |
| **Sleep Pin Protection** | Dedicated `ANALOG` pinctrl sleep states on `PC10`, `PC11`, `PB8`, `PB9`, `PA0`, `PA9` + full port `GPIOA-GPIOH` register save/restore | Simulated device PM state actions (`pm_device_suspend_tolerant`) |

## 7. Prerequisites & Workspace Setup

### 1. Install Host Dependencies (Ubuntu / Debian)

```bash
sudo apt-get update
sudo apt-get install -y \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-venv python3-tk python3-wheel xz-utils file \
  make gcc g++ gcc-multilib g++-multilib libsdl2-dev libmagic1 \
  qemu-system-arm lcov gcovr libgtest-dev binutils
```

### 2. Initialize the Zephyr Workspace (`v4.4.0`)

The repository provides a self-contained `west.yml` manifest locked to **Zephyr `v4.4.0`**:

```bash
# Install west
pip3 install --user -U west

# Clone the repository into a workspace directory
mkdir zephyr-workspace && cd zephyr-workspace
git clone https://github.com/abhijith-account/HCE_task.git custom_app
cd custom_app

# Initialize the workspace using the local west.yml manifest and pull Zephyr v4.4.0
west init -l .
west update
west zephyr-export

# Install Zephyr Python dependencies
pip3 install --user -r ../zephyr/scripts/requirements.txt
```

Ensure the [Zephyr SDK](https://www.google.com/search?q=https://github.com/zephyrproject-rtos/sdk-ng) is installed and configured (or execute inside the official `zephyrprojectrtos/ci:latest` Docker container).

## 8. Building, Flashing & Running QEMU Emulation

### Build & Flash for STM32 Nucleo-F446RE

```bash
# Cross-compile for STM32 Nucleo-F446RE
west build -b nucleo_f446re . --build-dir build_nucleo --pristine

# Flash onto a connected Nucleo-F446RE board via ST-LINK
west flash --build-dir build_nucleo
```

Key build outputs in `build_nucleo/zephyr/`:
- `zephyr.elf` — ELF image (use with `svd/STM32F446.svd` for hardware register inspection in GDB/VS Code)
- `zephyr.bin` / `zephyr.hex` — Deployable binary images
- `zephyr.map` / `zephyr.dts` / `.config` — Linker map, merged devicetree, and resolved Kconfig

> **Hardware Current Measurement Note (`STOP` Mode):** An active SWD debug session sets `CoreDebug->DHCSR.C_DEBUGEN = 1`, which keeps `HCLK`/`FCLK` running during STM32 `STOP` mode (`~0.9 mA` instead of `~234 µA`). To measure true `STOP` current on `nucleo_f446re`, end the debug session, disconnect SWD, and power-cycle the board (`C_DEBUGEN` is cleared only on power-on reset).

### Build & Run Cortex-M4 Emulation in QEMU (`mps2/an386`)

```bash
# 1. Compile for the ARM MPS2+ AN386 Cortex-M4 target
west build -b mps2/an386 . --build-dir build_mps2 --pristine

# 2. Execute in QEMU (runs with a 120s timeout, or press Ctrl+A then X to exit)
timeout 120s west build -d build_mps2 -t run
```

**Expected QEMU Runtime Sequence:**
1. **Boot & FCB Provisioning (`0-2 s`)**: Formats `sim_flash`, seeds `Slots 1-5` (`Device IDs 1001-1005`, `Alarm Thresholds 80-100`), completes the 1,000-cycle Flash endurance test on `ALARM_THRESHOLD_BASE`, and sets the default infusion rate (`50 mL/hr`).
2. **Normal Sensor & BMS Telemetry (`2-5 s`)**: Polls emulated `BME280` (`0x76`), `LPS22HB` (`0x5C`), `PAV3015` (`0x28`), `INA226` (`11100 mV`, `100 mA`), and `B3950` NTC (`25.0 °C`), while `heart_rate_producer_thread` and `display_consumer_thread` log heart-rate samples (`65-145 bpm`).
3. **Live Fault Injection & Safe Halt (`> 5 s`)**: At `5000 ms`, `bme280_transfer` in `src/qemu_sensor_mocks.c` injects `-EIO` (`BME280 emulator simulating disconnect!`). `I2CManager` serves cached `BME280` readings while decaying `reliability_score` (`100 -> 50` by `10` per read), then after 3 consecutive uncached failures triggers `"I2C Sensor Disconnected"`, transitioning the system cleanly to `SystemState::FAULT -> SystemState::SAFE_HALT`.

### Custom Kconfig Symbols & Runtime Tracing

Defined in `Kconfig` and configured in `prj.conf`:
- `CONFIG_SOC_MPS2_AN386`: Selects `HAS_PM` so Zephyr's power management subsystem builds cleanly on the emulated `mps2/an386` target.
- `CONFIG_TRACING_USER=y`: Hooks `sys_trace_thread_switched_in_user()` and `sys_trace_thread_switched_out_user()` in `src/RTOS_Command_based_thread_system/Thread_Trace.cpp` to measure and log real thread preemption latencies.
- `CONFIG_LOG_PREEMPTION_DELAY=y` & `CONFIG_LOG_EXECUTION_CYCLES=n`: Custom Kconfig flags retained in `Kconfig` and `prj.conf` for cycle-profiling configuration.

## 9. USB CDC Virtual COM Shell Reference

Connecting to the USB OTG FS port (`VID: 0x0483`, `PID: 0x5740`, product `"Medical Firmware"`) opens an interactive CLI with the prompt `med-device:~$ `:

| Command | Syntax | Behavior, Output Format & Safety Validation |
| :--- | :--- | :--- |
| `status` | `status` | Prints a JSON snapshot of system state, battery SoC (`%`), and all 5 provisioned infusion slots (`Slots 1-5`, enforced at compile time via `static_assert(InfusionDeviceConfig::MaxSlot - InfusionDeviceConfig::MinSlot + 1 <= StatusSnapshot::MaxTrackedSlots)`):<br>`{"sys_state":1,"battery_soc":85,"devices":[{"device_id":1001,"rate":50,"alarm_threshold":80},...]}` |
| `set_rate` | `set_rate <device_id> <rate 1-100>` | Validates and persists a new infusion rate (`1-100 mL/hr`) in Flash (`FCB`) for a provisioned device (`1001-1005` by default), returning `Success: Device <id> infusion rate set to <rate>%`. Rejects the command if: (1) the system is locked in `SAFE_HALT`/`FAULT`, (2) `<rate>` is outside `1-100`, (3) `<device_id>` is not provisioned, or (4) `<rate>` exceeds the slot's `alarm_threshold` (`80, 85, 90, 95, 100`). |
| `log_dump` | `log_dump` | Iterates through persistent `ConfigStore::LogEntry` (`12 B` struct: `timestamp_ms`, `event_id`, `event_data`) records under `ConfigKey::FULL_CHARGE_LOG` in Flash FCB (yielding every 10 records via `k_yield()`), outputting `[%08lu] Event: %u, Data: %d` between `--- NVS System Log Dump ---` and `--- End of Logs ---` (or `No logs found.`). |
| `reboot` | `reboot` | Transmits `Rebooting system...` and invokes `sys_reboot(SYS_REBOOT_COLD)` after a `100 ms` flush delay. |

**Hardware USB Disconnect Interlock:** If the physical USB cable is unplugged or suspended after enumeration, `usb_status_cb()` in `src/main.cpp` immediately calls `sys_context.triggerFault("USB CDC Cable Disconnected")`.

## 10. Host Unit Testing & 100% Coverage Verification

The `tests/` directory contains 9 standalone C++20 **GoogleTest (`GTest`)** executables configured by `tests/CMakeLists.txt`. Rather than generating mocks at configure time, host-side Zephyr OS stub headers (`tests/mocks/include/zephyr/`) and shared mock state (`tests/mocks/mock_globals.cpp`) with deterministic fault injection (`g_i2c_force_errno`, `g_i2c_bus_recovery_attempts`, `g_pm_state`) are checked directly into the repository so clean checkouts and CI builds are immediately reproducible.

### Unit Test Executables

| Test Binary | Source Files (`tests/`) | Verified Functionality & Edge Cases |
| :--- | :--- | :--- |
| `run_pm_tests` | `power/test_power_mocks.cpp`<br>`power/test_Power_Management_System.cpp`<br>`power/test_Power_ActiveIdleStates.cpp`<br>`power/test_Power_StopState.cpp`<br>`power/test_Power_Integration.cpp`<br>`power/test_Power_Hardware.cpp`<br>`power/test_Power_Singleton.cpp` | `PM_STATE_ACTIVE` -> `PM_STATE_SUSPEND_TO_IDLE` -> `PM_STATE_SUSPEND_TO_RAM` FSM transitions, STM32 RCC clock scaling, USB OTG & GPIO sleep states, RTC Alarm A wakeups, observer callbacks, and 3-strike PM fault escalation. |
| `run_usb_shell_tests` | `test_usb_shell.cpp` | ISR RX ring-buffer wrap/overflow, DTR handshake, `status`/`set_rate`/`log_dump`/`reboot` command parsing, alarm threshold rejection, and USB disconnect fault handling. |
| `run_battery_tests` | `battery/test_battery_mocks.cpp`<br>`battery/test_Smart_Battery_System.cpp`<br>`battery/test_Smart_Battery_Fetch.cpp`<br>`battery/test_Smart_Battery_Poll.cpp`<br>`battery/test_Smart_Battery_Soc.cpp`<br>`battery/test_Smart_Battery_FSM.cpp`<br>`battery/test_Sensor_Drivers.cpp`<br>`battery/test_Bms_Comm_Thread.cpp`<br>`battery/test_Battery_Monitor_Thread.cpp`<br>`battery/test_Battery_Runtime.cpp` | INA226 endianness & register reads, NTC thermistor ADC oversampling & LUT interpolation, exponential backoff retries, 1D Kalman Filter SoC estimation, OCV resync, pack disconnect detection, and BMS watchdog cutoff/recovery. |
| `run_static_memory_tests` | `test_static_memory.cpp` | `StaticPool<T, N>` exhaustion, null/out-of-bounds pointer safety, double-free rejection, alignment guarantees, and `memory_monitor_thread` stack watermark reporting. |
| `run_config_tests` | `test_config_store.cpp` | FCB mount/format on empty flash (`-ENOMSG`), abort on `SystemState::FAULT`, magic word validation (`0x12345678`), sector rotation on `-ENOSPC`, slot provisioning, and 1,000-cycle Flash endurance verification. |
| `run_i2c_faults_tests` | `test_I2C_Fault_Recovery.cpp` | 8/16/24/64-bit register transfers, `RetryStrategy`, `BusResetStrategy` SCL recovery, `FailSafeStrategy` 64-entry cache fallback with reliability decay (`100 -> 50`), and bus error injection (`-EIO`, `-ETIMEDOUT`, `-EBUSY`). |
| `run_state_machine_tests` | `test_state_machine.cpp` | Legal and illegal `SystemState` transitions (`INIT`, `RUNNING`, `FAULT`, `SAFE_HALT`), spinlock synchronization, and 9-thread heartbeat watchdog supervision (`daly_watchdog_feed_hook`). |
| `run_sync_tests` | `test_synchronization.cpp` | `ZephyrSemaphore`, `ZephyrMutex`, `ZephyrWorkQueue`, `SharedHeartRateBuffer` producer/consumer synchronization, and 1 Hz `status_work` execution. |
| `run_command_tests` | `rtos/test_rtos_mocks.cpp`<br>`rtos/test_Command_Core.cpp`<br>`rtos/test_System_Objects.cpp`<br>`rtos/test_Sensor_Read_Cmd.cpp`<br>`rtos/test_Compute_Cmd.cpp`<br>`rtos/test_Sensor_Math.cpp`<br>`rtos/test_Print_Commands.cpp`<br>`rtos/test_Producer_Thread.cpp`<br>`rtos/test_Consumer_Threads.cpp`<br>`rtos/test_Thread_Trace.cpp` | `ICommand` placement-new pool lifecycle, `PROCESSOR_Q`/`LOGGER_Q` routing, BME280/LPS22HB/PAV3015 compensation math, 3-strike sensor disconnect escalation, and 4096-entry lockless thread preemption trace logging. |

### Checked-In Zephyr Mock Architecture (`tests/mocks/`)

All 9 host test executables compile against `tests/mocks/include` (`MOCK_DIR`) and link `tests/mocks/mock_globals.cpp` (`MOCK_GLOBALS_SRC`), providing deterministic host-side stubs for:
- **Kernel & Synchronization**: `zephyr/kernel.h`, `zephyr/spinlock.h`, `zephyr/device.h`, `zephyr/drivers.h`, `zephyr/debug/thread_analyzer.h`
- **Peripheral Drivers**: `zephyr/drivers/adc.h`, `counter.h`, `flash.h`, `gpio.h`, `i2c.h`, `rtc.h`, `uart.h`, `watchdog.h`, `usb/usb_dc.h`
- **Storage, Power & System Services**: `zephyr/fs/fcb.h`, `zephyr/kvss/nvs.h`, `zephyr/storage/flash_map.h`, `zephyr/pm/{device,pm,policy}.h`, `zephyr/logging/log.h`, `zephyr/sys/{__assert,atomic,byteorder,printk,reboot}.h`, `zephyr/usb/usb_device.h`

### Running Tests & Enforcing 100% Coverage Locally

```bash
# 1. Configure and build all 9 host GTest binaries with GCOV instrumentation
mkdir -p tests/build
cd tests/build
cmake ..
make -j$(nproc)

# 2. Execute all unit test suites
./run_pm_tests
./run_usb_shell_tests
./run_battery_tests
./run_static_memory_tests
./run_config_tests
./run_i2c_faults_tests
./run_state_machine_tests
./run_sync_tests
./run_command_tests
cd ../..

# 3. Enforce 100% line and 100% branch coverage across src/
chmod +x ./scripts/check_coverage.sh
./scripts/check_coverage.sh
```

`scripts/check_coverage.sh` invokes `gcovr` with `--fail-under-line 100 --fail-under-branch 100` (excluding unreachable `default:` assert branches and mock headers) to guarantee zero untested codepaths, and outputs the HTML summary tracked in `coverage_report.html`.

## 11. Continuous Integration Pipeline (GitHub Actions)

Defined in `.github/workflows/main.yml` (`Zephyr Firmware CI`), the pipeline runs inside `zephyrprojectrtos/ci:latest` and executes five mandatory stages:

1. **Zephyr Version Pin Check**: Initializes `west` and verifies that `zephyr/VERSION` matches `EXPECTED_ZEPHYR_VERSION=4.4.0`.
2. **STM32 Cross-Compilation (`nucleo_f446re`)**: Builds `build_nucleo/zephyr/zephyr.{elf,bin,map}` with `-Wall -Wextra -Werror` zero-warning enforcement.
3. **Cortex-M4 QEMU Emulation (`mps2/an386`)**: Runs the firmware in QEMU for `120 s`, asserting clean continuous output and non-zero Heartbeat tick counts.
4. **Host Unit Tests & 100% Coverage Gate**: Compiles and executes all 9 GTest binaries using the checked-in `tests/mocks/` headers and runs `./scripts/check_coverage.sh` (must achieve `line=100%`, `branch=100%`).
5. **Published Workflow Artifacts**:
   - `QEMU-M4-Execution-Report` (`qemu_m4_report.md`)
   - `QEMU-Execution-Logs` (`zephyr.map`, `zephyr.dts`, and `.config` for both `build_nucleo` and `build_mps2`)
   - `Coverage-Dashboard` (`coverage_report/`)
   - `Nucleo-Firmware` (`build_nucleo/zephyr/zephyr.hex`, `build_nucleo/zephyr/zephyr.bin`, and `firmware/zephyr.elf`)
