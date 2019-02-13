vlib work
vlog -sv -incr -f ./files
vsim -novopt i2c_example
do wave.do
run -all

