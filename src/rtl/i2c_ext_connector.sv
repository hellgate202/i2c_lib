module i2c_ext_connector
(
  i2c_int_if int_if,
  i2c_if     ext_if
);

  assign ext_if.sda   = ( int_if.sda_oe ) ? int_if.sda_o : 1'bz;
  assign ext_if.scl   = ( int_if.scl_oe ) ? int_if.scl_o : 1'bz;
  assign int_if.sda_i = ext_if.sda;
  assign int_if.scl_i = ext_if.scl;

endmodule
