# fix_qtpy_rp2040
Fix flash memory non-volatile settings

This utility will erase the flash and bump the read driver strength to 75% for Winbond Q64JVXGIQ flash memory on an Adafruit QT Py RP2040.

Simply drag install the uf2 file to your device. Give the utility about a minute to run, if you connect an LED to A3 it will blink when  complete. Reset your QT Py RP2040 and you should be good to go.

The utility runs out of SRAM, the changes it makes are non-volatile, so they will survive reset and power cycling.
