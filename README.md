# OWL Firmware

Firmware for the OWL measurement tool.

Uses the [SimpleFOC](https://simplefoc.com/) library.

## How to flash

1. Grab the latest UF2 firmware image from the [releases](https://github.com/sedlak477/owl-firmware/releases) page.
1. Reboot the controller into BOOT mode by holding down the BOOT button while powering/resetting the OWL.
1. The controller will show up as a storage medium in your file explorer.
1. Drop the firmware image into the folder.
1. After a short time, the controller will reboot into the new firmware.

## Related repositories

[owl-ranger](https://github.com/sedlak477/owl-ranger): A script for stepping through some orientations and recording measurements.
[pyowl](https://github.com/sedlak477/pyowl): A python library for interacting with the OWL.
