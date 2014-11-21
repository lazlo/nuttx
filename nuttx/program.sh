openocd -f interface/stlink-v2.cfg -f target/stm32f4x_stlink.cfg -c "program nuttx verify reset"
