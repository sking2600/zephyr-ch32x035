# WCH DMA Test Sample

This sample validates the WCH CH32 DMA driver by performing memory-to-memory
transfers and verifying data integrity.

## Overview

The test performs the following validations:
1. **Device Readiness** - Confirms DMA controller is initialized
2. **Memory-to-Memory Transfer** - Copies data between SRAM buffers
3. **Callback Verification** - Ensures interrupt-driven completion works
4. **Data Integrity** - Verifies transferred data matches source
5. **Status API** - Tests `dma_get_status()` functionality
6. **Attribute API** - Tests `dma_get_attribute()` functionality

## Building

```bash
west build -p always -b ch32l103_evt samples/drivers/dma/wch_dma_test
```

## Flashing

```bash
west flash
```

## Expected Output

```
*** Booting Zephyr OS ***
[00:00:00.000,000] <inf> dma_test: ====================================
[00:00:00.000,000] <inf> dma_test: WCH DMA Driver Validation Test
[00:00:00.000,000] <inf> dma_test: ====================================
[00:00:00.000,000] <inf> dma_test: DMA device ready: dma@40020000
[00:00:00.000,000] <inf> dma_test: === Test: Memory-to-Memory Transfer ===
[00:00:00.000,000] <inf> dma_test: Starting DMA transfer...
[00:00:00.000,000] <inf> dma_test: DMA callback: channel=0, status=0
[00:00:00.000,000] <inf> dma_test: Memory-to-Memory transfer: PASSED
[00:00:00.000,000] <inf> dma_test: === Test: Get Status ===
[00:00:00.000,000] <inf> dma_test: Get status: PASSED
[00:00:00.000,000] <inf> dma_test: === Test: Get Attributes ===
[00:00:00.000,000] <inf> dma_test: Get attributes: PASSED
[00:00:00.000,000] <inf> dma_test: ====================================
[00:00:00.000,000] <inf> dma_test: SUCCESS: All 3 tests passed!
[00:00:00.000,000] <inf> dma_test: ====================================
```

## LED Indicators

If `led0` is defined in the board DTS:
- **Solid ON** - All tests passed
- **Blinking** - One or more tests failed

## Supported Boards

- ch32l103_evt
- (Other CH32V/CH32L/CH32X boards with DMA support)
