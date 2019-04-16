I2C Library
===========

Currently consists only of I2C master PHY, which is based on I2C IP-core from
OpenCores https://opencores.org/projects/i2c

I did some code refactoring and converted it to SystemVerilog. I did this only
in educational purposes.

How to use
----------

Parameters:

**SCL_FREQ** - desired SCL frequency, specified in Hz

**CLK_FREQ** - core frequency, specified in Hz

Signals:

**clk_i**      - clock frequency

**rst_i**      - asynchronous reset

**cmd_i**      - command input, if core is not busy it will ecexute
                 non-NOP command. List of commands specified in i2c_pkg package

**data_i**     - data input for WRITE command

**data_o**     - data output for READ command, cmd_done validates this output

**cmd_done_o** - indicates that last command was completed

**arb_lost_o** - if this signal is asserted core has lost bus arbitration
                 and command is needed to executed again

**bus_busy_o** - when bus is busy core doesn't aquire commands

**sda_i, sda_o, sda_oe, scl_i, scl_o, scl_oe** - I2C bus signals
for tri-state buffers 
