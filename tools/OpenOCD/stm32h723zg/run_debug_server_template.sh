#!/bin/bash
/opt/st/OpenOCD/src/openocd -s /opt/st/OpenOCD/tcl -f /opt/st/OpenOCD/tcl/interface/stlink.cfg -f /opt/st/OpenOCD/tcl/target/stm32h7x.cfg -c "bindto 0.0.0.0"
