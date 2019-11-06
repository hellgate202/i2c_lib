module i2c_slave_phy #(
  parameter string MODE        = "FAST",
  parameter int    CLK_T       = 10000
)(
  input          clk_i,
  input          rst_i,
  input          sda_i,
  output         sda_o,
  output         sda_oe,
  input          scl_i,
  output [6 : 0] dev_addr_o,
  output [7 : 0] wr_data_o,
  output         wr_o,
  output         rd_o,
  input  [7 : 0] rd_data_i
);

localparam int SPIKE_FILT_L = 50000 / CLK_T;

logic [1 : 0]                sda_mtstb;
logic [1 : 0]                scl_mtstb;
logic [SPIKE_FILT_L - 1 : 0] sda_filt_d;
logic [SPIKE_FILT_L - 1 : 0] scl_filt_d;
logic                        sda_filt, sda_filt_d;
logic                        scl_filt, scl_filt_d;
logic                        sda_posedge;
logic                        sda_negedge;
logic                        scl_posedge;
logic                        scl_negedge;

enum logic [2 : 0] { IDLE_S,
                     START_S,
                     ADDR_S,
                     RW_S,
                     ACK_S,
                     DATA_S,
                     REPEATED_START_S,
                     STOP_S } state, next_state;

// Metastability protection
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      sda_mtstb <= '1;
      scl_mtstb <= '1;
    end
  else
    begin
      sda_mtstb <= { sda_mtstb[0], sda_i };
      scl_mtstb <= { scl_mtstb[0], scl_i };
    end

// Filtering 50ns noise spikes
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      sda_filt_d <= '1;
      scl_filt_d <= '1;
    end
  else
    begin
      sda_filt_d <= { sda_filt_d[SPIKE_FILT_L - 2 : 0], sda_mtstb[1] };
      scl_filt_d <= { scl_filt_d[SPIKE_FILT_L - 2 : 0], scl_mtstb[1] };
    end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    sda_filt <= '0;
  else
    if( &sda_filt_d )
      sda_filt <= 1'b1;
    else
      if( !( |sda_filt_d ) )
        sda_filt <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    scl_filt <= '0;
  else
    if( &scl_filt_d )
      scl_filt <= 1'b1;
    else
      if( !( |scl_filt_d ) )
        scl_filt <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      sda_filt_d <= 1'b1;
      scl_filt_d <= 1'b1;
    end
  else
    begin
      sda_filt_d <= sda_filt;
      scl_filt_d <= scl_filt;
    end

assign sda_posedge = sda_filt && !sda_filt_d;
assign sda_negedge = !sda_filt && sda_filt_d;
assign scl_posedge = scl_filt && !scl_filt_d;
assign scl_negedge = !scl_filt && scl_filt_d;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    state <= IDLE_S;
  else
    state <= next_state;

always_comb
  begin
    next_state = state;
    case( state )
      IDLE_S:
        begin
        end
    endcase
  end

endmodule
