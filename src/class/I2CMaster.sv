`timescale 1 ps / 1 ps

class I2CMaster #(
  parameter int SCL_FREQ = 400_000
);

localparam real SCL_T    = 64'd1_000_000_000_000 / real'( SCL_FREQ );
localparam int  MODE     = ( int'(SCL_T) >= 10_000_000 ) ? 0 :
                           ( int'(SCL_T) >= 2_500_000  ) ? 1 : 2;
localparam int  T_SU_STA  = ( MODE == 0 ) ? 4700000 :
                            ( MODE == 1 ) ? 600000  : 260000;
localparam int  T_HD_STA  = ( MODE == 0 ) ? 4700000 :
                            ( MODE == 1 ) ? 600000  : 260000;
localparam int  T_SU_DAT  = ( MODE == 0 ) ? 250000  :
                            ( MODE == 1 ) ? 100000  : 50000;
localparam int  T_HD_DAT  = 300000;
localparam int  T_SU_STO  = ( MODE == 0 ) ? 4000000 :
                            ( MODE == 1 ) ? 600000  : 260000;
localparam int  T_VD_DAT  = ( MODE == 0 ) ? 3450000 :
                            ( MODE == 1 ) ? 900000  : 450000;
localparam int  T_ACK_DAT = ( MODE == 0 ) ? 3450000 :
                            ( MODE == 1 ) ? 900000  : 450000;
localparam int  T_BUF     = ( MODE == 0 ) ? 4700000 :
                            ( MODE == 1 ) ? 1300000 : 500000;

virtual i2c_int_if i2c_int_if_v;

function new(
  virtual i2c_int_if i2c_int_if_v
);

  this.i2c_int_if_v = i2c_int_if_v;
  init_interface();

endfunction

function void init_interface();

  i2c_int_if_v.sda_o  = 1'b1;
  i2c_int_if_v.sda_oe = 1'b0;
  i2c_int_if_v.scl_o  = 1'b1;
  i2c_int_if_v.scl_oe = 1'b0;

endfunction

task automatic gen_start();

  i2c_int_if_v.sda_o  = 1'b0;
  i2c_int_if_v.sda_oe = 1'b1;
  #( T_HD_STA );
  i2c_int_if_v.scl_o  = 1'b0;
  i2c_int_if_v.scl_oe = 1'b1;

endtask

task automatic send_byte(
  input bit [7 : 0] data
);

  #( T_HD_DAT );
  for( int i = 8; i > 0; i-- )
    begin 
      i2c_int_if_v.sda_o = data[i - 1];
      #( SCL_T / 2 - T_HD_DAT );
      i2c_int_if_v.scl_o = 1'b1;
      #( SCL_T / 2 );
      i2c_int_if_v.scl_o = 1'b0;
      #( T_HD_DAT );
    end

endtask

task automatic wait_ack ();

  i2c_int_if_v.sda_oe = 1'b0;
  fork
    begin
      #( T_ACK_DAT );
      if( i2c_int_if_v.sda_i )
        begin
          $display("NACK received");
          $stop();
        end
    end
  join_none
  #( SCL_T / 2 - T_HD_DAT );
  i2c_int_if_v.scl_o  = 1'b1;
  #( SCL_T / 2 );
  i2c_int_if_v.scl_o  = 1'b0;
  i2c_int_if_v.sda_oe = 1'b1;

endtask

task automatic read_byte(
  output bit [7 : 0] data
);
 
  i2c_int_if_v.sda_oe = 1'b0;
  for( int i = 8; i > 0; i-- )
    begin
      #( SCL_T / 2 );
      i2c_int_if_v.scl_o = 1'b1;
      data[i - 1] = i2c_int_if_v.sda_i;
      #( SCL_T / 2 );
      i2c_int_if_v.scl_o = 1'b0;
    end
  #( T_HD_DAT );
  i2c_int_if_v.sda_oe = 1'b1;
  i2c_int_if_v.sda_o  = 1'b0;
  #( SCL_T / 2 - T_HD_DAT );
  i2c_int_if_v.scl_o  = 1'b1;
  #( SCL_T / 2 );
  i2c_int_if_v.scl_o  = 1'b0;

endtask

task automatic gen_stop();

  #( SCL_T / 2 );
  i2c_int_if_v.scl_o  = 1'b1;
  i2c_int_if_v.scl_oe = 1'b0;
  #( T_SU_STO );
  i2c_int_if_v.sda_o  = 1'b1;
  i2c_int_if_v.sda_oe = 1'b0;
  #( T_BUF );

endtask

task automatic gen_repeated_start();

  i2c_int_if_v.sda_o = 1'b1;
  #( SCL_T / 2 - T_HD_DAT );
  i2c_int_if_v.scl_o = 1'b1;
  #( T_SU_STA );
  i2c_int_if_v.sda_o = 1'b0;
  #( T_HD_STA );
  i2c_int_if_v.scl_o = 1'b0;

endtask

task automatic write_byte(
  input bit [7 : 0] data,
  input bit [6 : 0] addr
);

  gen_start();
  send_byte( { addr, 1'b0 } );
  wait_ack();
  send_byte( data );
  gen_stop();

endtask

task automatic write_burst(
  input bit [7 : 0] data [$],
  input bit [6 : 0] addr
);

  gen_start();
  send_byte( { addr, 1'b0 } );
  wait_ack();
  while( data.size() > 0 )
    begin
      send_byte( data.pop_front() );
      wait_ack();
    end
  gen_stop();

endtask

endclass
