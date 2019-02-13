`include "../src/class/I2CMaster.sv"

`timescale 1 ps / 1 ps

module i2c_example;

parameter int SCL_FREQ = 400_000;

wand sda;
wand scl;

// Pull-Up
assign sda = 1'b1;
assign scl = 1'b1;

wire sda_o;
wire sda_i;
wire sda_oe;
wire scl_o;
wire scl_i;
wire scl_oe;

i2c_if dut_if (
  .sda ( sda ),
  .scl ( scl )
);

i2c_int_if int_if (
  .sda_o  ( sda_o  ),
  .sda_i  ( sda_i  ),
  .sda_oe ( sda_oe ),
  .scl_o  ( scl_o  ),
  .scl_i  ( scl_i  ),
  .scl_oe ( scl_oe )
);

i2c_ext_connector ext_connector (
  .int_if ( int_if ),
  .ext_if ( dut_if )
);

I2CMaster #(
  .SCL_FREQ ( SCL_FREQ )
) master;

initial
  begin
    master = new( .i2c_int_if_v ( int_if ) );
    repeat( 10 )
      #( master.SCL_T );
    master.gen_start();
    master.send_byte( 8'h72 );
    master.gen_repeated_start();
    master.send_byte( 8'ha4 );
    master.gen_stop();
    repeat( 10 )
      #( master.SCL_T );
    $stop();
  end

endmodule
