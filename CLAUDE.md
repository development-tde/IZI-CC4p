# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**IZI-CC4+** is the firmware of the **IZI-DriveCC4+** (model name set in `IZI-CC4+/izilink/iziplus_module_def.c`), a four-channel constant-current LED driver by TDE-Lighttech that is controlled as an **IZI+ powerline slave** (DCB1M modem, master is IZI-PowerCom behind an IZI-Access hub). Remote: `https://github.com/development-tde/IZI-CC4p.git`. MCU: ATSAME51J18A (`<avrdevice>` in the `.cproj`; the linker scripts are the `same51j20a_*` variants), Cortex-M4F, FreeRTOS V10.0.0 (`thirdparty/RTOS`), 120 MHz core (`GCLK_GEN3_FREQ` in `misc/timer_def.h`).

Two sub-projects, one solution `IZI-CC4+.atsln` (which also lists the sibling `..\Izi-Lib-sam\IZI-Lib-sam.cproj`):
- `IZI-CC4+/` — application, currently **v1.2.71** (`IZI-CC4+/version.c`, `.image.device_type = IZIPLUS_DEVTYPE_BASE` = `0x220`, `device_type_range = 64`, `hw_version_start = 1 .. hw_version_end = 3`)
- `IZI-Boot-sam/` — bootloader **v1.0.5** (`IZI-Boot-sam/version.c`)

**Origin (git log, 8 commits since 2026-03-31):** the repo is a **fork of IZI-MoodSpot, not of IZI-CC2+**. The first real commit (`1e2f152`) carries `IZI-CC4+/.project-backup/IZI-MoodSpot_*.zip` and `.atmel-start-backup/IZI-PowerCom_*.zip`, the source tree is file-for-file the MoodSpot tree (`izican/`, `dmx/`, `driver/stepdown.c`, `izilink/iziplus_module_def.c`, …), and the untracked `Debug/aprebuild.bat` still names `IZI-MoodSpot.hex`. Leftover comments ("HP and MP" in `version.c`, "IZI-MoodSpot+ HP + emitter offset" in `iziplus_module_def.h`) are MoodSpot heritage. Later commits: "Step 2" (4-channel stepdown/analog/state), protocol 1.2, 6 s no-com fix, warm-dimming/tuneable modes + external NTC table, `hwref_min/hwref_max` in the emitter def (2026-09-05).

**Versus IZI-CC2+ (verified):** CC2+ drives 2 channels (`stepdown.h` there has no `Ch3/Ch4` functions), uses IZI+ type `0x100`/range 32, adds a Casambi bridge (`casambi/cbm.c`, `IZIOUTPUT_SRC_CAS`) and binds the DCB1M to USART3. CC4+ drives 4 channels, uses `0x220`/range 64, has `IZIOUTPUT_SRC_DMX` as second output source and binds the DCB1M to USART1 at 921600 baud.

**Hardware as wired in `IZI-CC4+/atmel_start_pins.h` / `driver_init.c`:** dim outputs `DIM1` PB13 (TCC3_WO1), `DIM2` PA19 (TCC1_WO3), `DIM3` PA24 (TCC2_WO2), `DIM4` PA21 (TCC0_WO1); channel enables `SW1..SW4`; per-channel current via an **MCP4728** DAC on SERCOM3 I2C (`SDA` PA22, `SCL` PA23, `LDAC` PA2, `RDY` PA5); LED-voltage sense `ADC_CH1..4`, `ADC_VIN`, `ADC_TINT1/2`, `EXT_NTC`, `EXT_NTC2`; modem pins `PLC_RST/HDC/TXON/CSEL0/CSEL1`; status LED `LED_R` PB1 / `LED_G` PB2 / `LED_B` PB0; `SWITCH` PB10 (identify button) and `EXT` PB11 (external input); `HW_ID0..3` → `get_hw_id()` (hardware revision), `DEV_ID0..2` → `get_device_id()` (variant).

## Build System

**Toolchain:** ARM GCC via Atmel Studio 7; `.cproj` files define the build, generated Makefiles can be run directly:

```sh
make -C IZI-CC4+/Release all      # application, tracked Makefile
make -C IZI-CC4+/Debug all        # application, Debug/ is git-ignored (IZI-CC4+/.gitignore)
make -C IZI-Boot-sam/Release all  # bootloader (Boot's Debug/ is git-ignored too)
```

| Configuration | Symbols (`.cproj`) | Linker script | Result |
|---------------|--------------------|---------------|--------|
| Release | `NDEBUG`, `IZIPLUS_DRIVER=1`, `USART2_INT_PRIO=5`, `xUSART2_DMA`, `DEBUG_UART=0`, `USE_CONTACT_TRIGGER` | `-Tsame51j20a_flash_boot.ld` (rom `ORIGIN 0x4000`, `LENGTH 0x1BF00`) | runs behind the 16 kB bootloader |
| Debug | Release set plus `DEBUG`, `USART1_BAUD=921600` | `-Tsame51j20a_flash_debug.ld` (rom `ORIGIN 0x0`) | stand-alone, **overwrites the bootloader when flashed** |

Both configurations keep `DEBUG_UART=0`, so no build has trace output (see Trace / Logging).

**Pre/post-build (Release only, `IZI-CC4+/Release/`):** `aprebuild.bat` runs `IziVersionBuilder.exe ..\version.c IZI-CC4+.hex`, which **increments `.version.v.build` in `version.c` on every build** (`IZI-VersionBuilder\IziVersionBuilder\Program.cs`). `amerge.bat` runs `mergebin ../../IZI-Boot-sam/Release/IZI-Boot-sam.bin IZI-CC4+.bin firmware.bin 544 IZI-CC4+.hex output IZI-CC4+` — `544` is the device type `0x220` (`args[3]` = device group in `MergeBinary\mergebinary\Program.cs`) — producing `Release/output/IZI-CC4+_V<app>.bin/.hxx` and the merged `IZI-CC4+_V<app>_V<boot>.bin`. `IziVersionBuilder.exe`, `mergebin.exe` and all `Release/*.bin/.hex/.elf/.lss/.map` are **tracked** (so every build dirties the working tree); `Release/output/` is only partly tracked — the `_V1.2.5`…`_V1.2.15` images are committed, newer ones sit untracked.

**Bootloader `IZI-Boot-sam/`:** `IZI-Boot-sam.cproj` compiles `main.c`, `misc/rprintf.c`, `uart/usart4.c`, `version.c` and has a `<ProjectReference>` to `..\..\Izi-Lib-sam\IZI-Lib-sam.cproj` (for `Crc16Fast`). Rom is `0x0`–`0x3FFC` with the `version_u` at `BOOT_VERSION_ADDRESS 0x3FFC`. `main.c` acts on `boot_data.action` (`BOOT_ACTION_STARTAPP`, `BOOT_ACTION_COPY_EXT`), CRC-checks the `firmware_t` at `FLASH_APP_END - FLASH_APP_FW_INFO_SIZE`, and copies the mirror image from `FLASH_APP_MIRROR_BASE 0x20000` page-wise to `FLASH_APP_START 0x4000`. `boot_data` is shared with the app through the `.boot_ram` region at `0x2001FF00` (both linker scripts). No pre/post-build scripts.

**Flash/debug probe:** J-Link (also Atmel-ICE and nEDBG entries in the `.cproj`); settings in `IZI-CC4+/jlink.config` and `IZI-Boot-sam/jlink.config`; the tracked `jlink*.log` files are probe noise. There are no unit tests.

## Architecture

### RTOS tasks

`main.c`: `atmel_start_init()` → `Eeprom_init()` + `bod33_ensure(2.8 V)` (`BOD_ENABLED`) → `IziPlus_Module_SetFixtureParameters()` → `AppConfig_Init()` → auto-generate a production serial if `PRODUCTION_SERIAL == 0xFFFFFFFF` → `DMA_Init()`, `timing_init()`, `State_Init()`, `IziInput_Init()`, `IziPlus_Init()`, `StepDown_Init()` → `vTaskStartScheduler()`. The task enum in `includes.h` (`os_report_task_t`, mask `OS_REPORT_ALLTASKS 0x1F`) versus what is actually created:

| Enum | Task name | File | Priority / stack | Created? |
|------|-----------|------|------------------|----------|
| `TASK_DEBUG` (0) | `Debug` | `misc/debug.c` | idle+1, 2 kB | **no** — `Debug_Init()` is commented out in `main.c` |
| `TASK_CANDRIVER` (1) | `IziCan` (library) | `IZI-Lib-sam/can/izi_can_driver.c` | – | **no** — driver compiled but no `izican_*` call in device code |
| `TASK_STEPDOWN` (2) | `StepDown` | `driver/stepdown.c` | idle+6, 2 kB | yes (`StepDown_Init`, 10 ms loop) |
| `TASK_IZIPLUS` (3) | `IziPlus` | `izilink/iziplus_driver.c` | idle+4, 2 kB | yes (`IziPlus_Init`; also starts the library `IziPlusRx` task) |
| `TASK_IZIDATA` (4) | – | – | – | unused; `izi_input.c` runs from a FreeRTOS timer |

`vApplicationIdleHook()` refreshes the watchdog (`WD_Refresh()`) and clears `boot_data.reset_wd_cnt` after 5 s; `vApplicationStackOverflowHook()` sets `boot_data.last_code = '*'`.

### `izilink/` — IZI+ slave application
- `iziplus_driver.c/h` — device side of the IZI+ protocol on top of the library slave stack: `dcbm1_driver_init()`, `iziplus_dataframe_init(IZIPLUS_DEVTYPE)`, states `IZIPLUS_STATE_STARTUP/COMMISSIONING/COMMISSION_EXIT/NORMAL` (`IZIPLUS_COMMISSIONING_TIME 2000` ms), com watchdogs `IZIPLUS_COM_ACTIVE_TIME 2000` / `IZIPLUS_COM_TOKEN_TIME 9000` ms, `IziPlus_ContactReport()` under `USE_CONTACT_TRIGGER`, `IziPlus_Debug()`.
- `iziplus_module_def.c/h` — the fixture model: `IZIPLUS_DEVTYPE_BASE 0x220`, modes `MODE_1_CHANNEL` 0 … `MODE_CHANGEOVER_WD_DUAL` 11 (TW/WD single/dual and changeover variants), config parameters (`CONFIG_PAR_CURRENT` → `150 + 50·v` mA, `CONFIG_PAR_FILTER`, `CONFIG_PAR_CURVE`, `CONFIG_PAR_SWAP`, `CONFIG_VIRTUAL_IN`), `iziplus_fixture_def` (up to 12 modes / 12 configs / 24 monitors) and the 256-byte `emitter_fixture_def` (`max_channels`, `test_lvls`, black-body curve, forward-voltage and temperature tables, `hwref_min`/`hwref_max` — `0/0` means exact `hwref` match, trailing `crc`). `IziPlus_NoComCheck()` ignores the first 6000 ticks after power-up.
- `izi_input.c/h` — `SWITCH` button (identify after `TIME_START_IDENTIFY`) and `EXT` input debounce (`EXT_DEBOUNCE`), `IziInput_GetExtInput()`.

### `driver/` — output engine
- `stepdown.c/h` — four PWM channels on TCC3/TCC1/TCC2/TCC0 through the event system, MCP4728 current setting (`Mcp4728_Init()`, `Mcp4728_WriteDacs()`), per-channel `stepdown_control_t` (filter, curve, frequency dithering `per/cc0/ccx[2][MAX_TABLE]`, Vled tracking, `pwm_offset`, `force_calibration`), safety-off and power limiting; `StepDown_Init()` also calls `Analog_Init()`. Only TCC0/TCC1 are 24-bit (comment at line 68).
- `curve.c/h` — `quadratic_curve[256]`.

### `misc/`
- `analog.c/h` — ADC1 scan `ADC_CH1..4`, `ADC_TINT1/2`, `EXT_NTC2`; ADC0 scan `EXT_NTC`, `ADC_VIN` (`ADC_VIN_MAX 54348` mV); `ntc_table[512]`, `ntc_table_ext[512]`.
- `state.c/h` — error/warning bitmaps (`STATE_ERROR_OUTPUT1..4_SHORT`, `STATE_ERROR_NO_COM`, `STATE_ERROR_VIN_LOW`, `STATE_WARNING_OUTPUT1..4_OPEN`, `STATE_WARNING_POWER1..4_HIGH`, …) shown on the RGB LED; `State_SetWarning()` is what the library calls.
- `debug.c/h` — USART2 console (not started, see below). `timing.c/h` — TC-based µs callbacks (`timing_set_callback0/1`); initialised in `main.c`, no compiled caller. `timer_def.h` — GCLK/TC/TCC ids.

### `appconfig.c/h` — persistent data (Smart EEPROM via the library)
| Struct | Version | Content |
|--------|---------|---------|
| `appconfig_t` (512 B) | `CONFIG_VERSION 7` | `name`, `pan_id`, `pl_frequency/pl_rate/pl_txlevel`, `dbg_on`, `short_id`, `channel`, `mode`, `dmxfail`, `config[IZI_MAX_CONFIG]`, `variant`, `typeid` |
| `appcalib_t` (128 B) | `CALIB_VERSION 1` | `pwm_offset[4]`, `vled_max_mv[4]` |
| `applog_t` (128 B) | `LOG_VERSION 1` | `operating_sec`, `active_sec`, `active_ch_sec[4]`, `ntc_temp_max`, `supply_min`, `change_count` |

### Not compiled (MoodSpot leftovers)
`izican/`, `dmx/`, `powerline/`, `uart/` and `USB_Serial.c` are **absent from `IZI-CC4+.cproj`**; the DCB1M and USART drivers that are built are the library's. The `usb/` stack (`usbdc.c`, `cdcdf_acm.c`) is compiled but nothing in device code calls `usbdc_*`/`cdcdf_acm_*`. `iziplus_driver.c` and `main.c` still include `dmx.h`, `dmx_rdm_module.h`, `rdm.h`, `11LC160.h` header-only.

### Shared library usage
Compiled from `..\..\IZI-Lib-sam\` by the `.cproj` (mechanism and APIs: `C:\projects\trunk\IZI-Lib-sam\CLAUDE.md`): `can/izi_can_driver.c`, `dmac/dmac.c`, `driver/mcp4725.c`, `driver/mcp4728.c`, `izilink/izi_output.c`, `izilink/slave/iziplus_dataframe.c`, `izilink/slave/iziplus_networkframe.c`, `misc/bod.c`, `misc/rprintf.c`, `misc/rtctime.c`, `misc/smart_eeprom.c`, `powerline/dcbm1_driver.c`, `production/production.c`, `uart/i2cx.c`, `uart/usart1.c`, `uart/usart2.c`. Include paths also name `izilink/master`, `dmx` and a non-existent `izi_can` folder (stale), some with the spelling `Izi-Lib-sam`.

## Key Compile Flags (includes.h)

| Flag | Value / state | Effect |
|------|---------------|--------|
| `BOD_ENABLED` | on | `bod33_ensure(BOD33_LEVEL_FROM_VOLTAGE(2.8f), …)` in `main.c` |
| `IZIOUTPUT_SRC`, `IZIOUTPUT_SRC_IZI 0`, `IZIOUTPUT_SRC_DMX 1` | on | two-source `izi_output.c` level buffer |
| `DMA_CHANNELS 2`, `USART1_DMA`, `USART1_DMA_CHANNEL 0` | on | DMA TX for the modem UART |
| `DCBM1_UART_INIT/PUTC/PUTBFR` = `Usart1_*`, `USART1_BAUD` (default `2*460800`) | on | DCB1M bound to USART1 at 921600 |
| `IZIPLUS_DATAFRAME_ONLY_RXINT`, `SLAVE_DEVTYPE_GROUP DEVTYPE_GROUP_POWERLINE` | on | library slave/CAN role |
| `SMOOTH_MODE_SWITCH`, `MS_TEMPCORR_OFF` | on | smooth mode crossfade; temperature correction disabled |
| `MS_MEASURE`, `MS_TEST_NO_EMITTER`, `MS_LOAD_ATW/TESTER/TW`, `MS_ENABLE_TEST_SWITCH`, `DCBM1_TRACE_ENABLE`, `USART2_DMA` | commented out | measurement/test aids — keep off in releases |
| `USART2_BAUD 250000`, `USART2_STOP_BITS 2`, `USART2_TX_BFR_SIZE 260` | when `DEBUG_UART == 0` | DMX timing on USART2 (unused: `Dmx_Init()` is never called) |
| `DEBUG_UART=0`, `IZIPLUS_DRIVER=1`, `USE_CONTACT_TRIGGER` | `.cproj` symbols | trace off; `IZIPLUS_DRIVER` is only read by the uncompiled `izican/`; contact reporting in `iziplus_driver.c` |

## Trace / Logging

Mechanism (`esp_printf`, `trace_level_t`, `REPORT_STACK`) is the library pattern — see `IZI-Lib-sam\CLAUDE.md`. Device-specific: `includes.h` defines the `trace_level_t` enum, `REPORT_STACK()`/`ReportStackSize()`, `OS_TRACE_*` (`[osx]`, `OS_TRACE_BUILD_LVL 2`), defaults `IZI_DFLT_TRACE_LVL`, `IZI_OUTPUT_DFLT_TRACE_LVL`, `DCBM1_DFLT_TRACE_LVL`, `ITO_DFLT_TRACE_LVL`, `STATE_DFLT_TRACE_LVL`, `IZI_INPUT_DFLT_TRACE_LVL` (all `TRACE_LEVEL_INFO`), `IZI_CAN_TRACE_BUILD_LVL 3`; `misc/state.h` adds `STATE_TRACE_*` (`[sta]`). Every build level is forced to `-1` when `DEBUG_UART == 0`, which both configurations set, so **no build emits trace**. The console in `misc/debug.c` (USART2, `Debug_Putc` fans out on `dbg_on`: `DBG_USB_MASK 0x02` → `Usart2_Putc`, `DBG_CAN_MASK 0x01` is commented out) understands `a` (supply voltage/temperature), `S…` (`StepDown_Debug`), `VI`/`VW<n><v>` (MCP4725 init/write), `ci`/`cd`/`cp`/`cs` (`AppConfig_Init/Default/Print`/set), `o` (heap + task list), `D<n>[..F]` (debug mask, `F` stores it in `appconfig_t.dbg_on`), `I…` (`IziPlus_Debug`), `PW<c>` (`Production_Write` test record); `C` (CAN) is commented out.

## Counterpart on the other side

- **Bus path:** CC4+ → IZI+ powerline → IZI-PowerCom (master, `IZI-Lib-sam\izilink\master\`) → CAN → IZI-Access hub. The hub's `C:\projects\trunk\IZI-Access-lpc\izi\izi_driver.c` keeps the `izi_module_t` list (`base.device_type` from the discover response) and `IZI-Access-lpc\izi\izi_modules.c` loads the per-type definition from its filesystem as `/DEVTYPES/<devtype>.tde` (line 403) — for this device `/DEVTYPES/544.tde`. No `CC4` string exists in `IZI-Access-lpc` or `IZI-Services`; handling is generic by type id and `.tde` definition.
- **Desktop:** `C:\projects\trunk\IZI-Supervisor\IZI-Supervisor\References\DeviceTypeDefs.tde` line 1672 defines `<DeviceTypes Name="IZI-DriveCC4+" FirmwareName="IZI-DriveCC4+" ID="544">` with modes "1 Channel", "4 Channel Master", "4 Channel", "4 channel 16bit", … matching the `MODE_*` ids here. The copies in `IZI-Utils\IziUtils\References\` and `IZI-Manager\SpotManager\References\` do **not** contain ID 544. The file is parsed by `IZI-Utils\IziUtils\IziLink\IziLinkDevices.cs` (`ReadDeviceTypes`), located by `IZI-Supervisor\IZI-Supervisor\MainViewModel.cs` line 80 and refreshed from `.tsu` packages by `IZI-Services\Services\IziFileService.cs`. Module lists arrive as `IZI-Services\IziTopic\Commands\Control\TopicRspGetModules.cs`; firmware-image checks (`IMAGE_MARKER`, `device_type`, `device_type_range`) are listed in the library CLAUDE.md.
- **Not IZI-Link:** CC4+ speaks IZI+, not the ASCII IZI-Link protocol. `IZI-Utils\IziUtils\IziLink\IziLinkDevices.cs` line 82 (`DeviceTypeID = 0x05, Name = "IZIDriveCC4"`) and `IZI-Manager\SpotManager\IziLink\IziLinkMaster.cs` line 1420 ("Faked device CC4") refer to the legacy IZI-DriveCC4 (`.tde` ID 5), a different product.

## Debugging recipes

- **No output on any UART:** expected — `DEBUG_UART=0` in both `.cproj` configurations and `Debug_Init()` is commented out in `main.c`. Set `DEBUG_UART=1` in the symbols, uncomment `Debug_Init()`, and USART2 becomes the console (the DMX baud settings in `includes.h` drop out automatically).
- **Edits to `powerline/dcbm1_driver.c`, `uart/usart1.c`, `izican/*`, `dmx/*` do nothing:** they are not in the `.cproj`; change the library copies in `IZI-Lib-sam` (the local `dcbm1_driver.c` already differs from the library by ~150 lines).
- **`version.c` and `Release/*` show as modified after every build:** `aprebuild.bat` bumps the build number and the outputs are tracked; commit them deliberately with the code they belong to.
- **Firmware upload refused / wrong emitter table:** the image announces `device_type 0x220`, range 64, hardware 1–3; the board reports `get_hw_id()` (HW_ID0..3) as `IziPlus_Module_GetHwRevision()` and `get_device_id()` (DEV_ID0..2) as variant; emitter defs are matched on `hwref_min..hwref_max` (or exact `hwref` when both are 0).
- **Device dead after a Debug flash:** the Debug linker script places the app at `0x0`, erasing IZI-Boot-sam; reflash `IZI-Boot-sam/Release/IZI-Boot-sam.bin` or use a `Release/output/IZI-CC4+_V*_V*.bin` merged image.
- **No-com errors right after power-up:** `IziPlus_NoComCheck()` waits 6000 ticks; afterwards `STATE_ERROR_NO_COM` / `STATE_ERROR_INCOMPLETE_COM` follow the 2 s active / 9 s token timers in `iziplus_driver.c`.
- **Debug build fails to link:** `IZI-CC4+/Debug/Makefile` links `-lIZI-Lib-sam` from the absolute path `C:\projects\trunk\Izi-Lib-sam\Debug`, so the library's Debug archive must exist and trunk must live at `C:\projects\trunk`.

## Key Files Quick Reference

| File | Role |
|------|------|
| `IZI-CC4+.atsln`, `IZI-CC4+/IZI-CC4+.cproj` | Solution and build definition (symbols, include paths, library sources) |
| `IZI-CC4+/main.c` | Boot-record handling, EEPROM/BOD init, task start-up, idle-hook watchdog |
| `IZI-CC4+/includes.h` | Compile flags, task enum, trace defaults, `REPORT_STACK` |
| `IZI-CC4+/version.c` | Firmware version 1.2.71, device type `0x220`, range 64, HW 1–3 |
| `IZI-CC4+/appconfig.h` | `appconfig_t`, `appcalib_t`, `applog_t` (`CONFIG_VERSION 7`) |
| `IZI-CC4+/atmel_start_pins.h` | Every pin name used above |
| `IZI-CC4+/izilink/iziplus_driver.c` | IZI+ slave application task |
| `IZI-CC4+/izilink/iziplus_module_def.c/h` | Fixture/emitter definitions, modes, configs, monitors |
| `IZI-CC4+/driver/stepdown.c` | 4-channel PWM/DAC current control task |
| `IZI-CC4+/misc/analog.c` | ADC scans, NTC tables |
| `IZI-CC4+/misc/state.c` | Error/warning state and RGB LED |
| `IZI-CC4+/Release/aprebuild.bat`, `amerge.bat` | Version bump and bootloader merge |
| `IZI-Boot-sam/main.c`, `IZI-Boot-sam/version.c` | Bootloader logic, boot version 1.0.5 |
