.. _ch32x035_evt:

WCH CH32X035 EVT
################

Overview
********

The CH32X035-EVT-R0 is a development board for the WCH CH32X035 RISC-V microcontroller.
The CH32X035 features a QingKe V4C core with USB PD PHY, making it suitable for USB Power Delivery applications.

Hardware Features
*****************

- **SOC**: WCH CH32X035G8U6 (RISC-V QingKe V4C)
- **Flash**: 62KB
- **RAM**: 20KB
- **Clock**: Up to 48MHz
- **Peripherals**:
    - USB PD PHY (Source/Sink)
    - USB 2.0 FS Device/Host
    - 4x UART
    - 1x SPI, 1x I2C
    - 2x OPA, 2x CMP
    - 12-bit ADC
    - PIOC (Programmable I/O Controller)

Supported Features
==================

The Zephyr 'ch32x035_evt' board configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | :dtcompatible:`wch,ch32-pfic`       |
+-----------+------------+-------------------------------------+
| SYSTICK   | on-chip    | :dtcompatible:`wch,ch32-systick`    |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | :dtcompatible:`wch,ch32v-uart`      |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | :dtcompatible:`wch,ch32-gpio`       |
+-----------+------------+-------------------------------------+
| PINCTRL   | on-chip    | :dtcompatible:`wch,ch32-pinctrl`    |
+-----------+------------+-------------------------------------+
| USB PD    | on-chip    | :dtcompatible:`wch,ch32x035-usbpd`  |
+-----------+------------+-------------------------------------+

Programming and Debugging
*************************

The board includes an on-board WCH-Link debugger (if using the official EVT) or requires an external WCH-Link.

Flashing
========

.. code-block:: console

   west flash

Debugging
=========

.. code-block:: console

   west debug

References
**********

- `WCH CH32X035 Website <https://www.wch.cn/products/CH32X035.html>`_
