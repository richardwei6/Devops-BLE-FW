# Devops-BLE-FW
This repository contains the microcontroller code for DevOps hardware that uses the nRF52 BLE platform.
This is kept separate from the main DevOps repository because the BLE feature is mutually exclusive with the `bare-metal` requirement in mbed_app.json that is needed to build LPC1549 targets with SD card support.

See the [main README in the Devops-FW repository](https://github.com/CalSol/Devops-FW).
Most (if not all?) of it is still applicable to this repository, but it will not be duplicated here for DRY-ness.
