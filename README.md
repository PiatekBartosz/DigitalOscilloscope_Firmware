# DigitalOscilloscope
This project is home page for digital oscilloscope project

## Docker build profiles

The Makefile builds release by default and keeps artifacts separate by
configuration:

```sh
make
make debug=n
make debug=y
```

`make build debug=y` and `make deploy debug=n` are equivalent low-level forms.
`make rebuild debug=y` rebuilds only that configuration; `make clean` removes
both configuration trees.

## Project structure
The project is divideed into following modules:
1. Hardware repo - Digital oscilosope annalog front end.
2. Firmware repo - STM32 H7232ZG Zephyr RTOS firmware.
3. Software repo - Software with UI for the oscilloscope.
4. FPGA repo - Cyclon IV connected to AF and STM32 MCU.
