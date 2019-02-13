onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -divider INT_IF
add wave -noupdate /i2c_example/int_if/sda_i
add wave -noupdate /i2c_example/int_if/sda_o
add wave -noupdate /i2c_example/int_if/sda_oe
add wave -noupdate /i2c_example/int_if/scl_i
add wave -noupdate /i2c_example/int_if/scl_o
add wave -noupdate /i2c_example/int_if/scl_oe
add wave -noupdate -divider DUT_IF
add wave -noupdate /i2c_example/dut_if/sda
add wave -noupdate /i2c_example/dut_if/scl
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {525 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 265
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {99645 ns}
