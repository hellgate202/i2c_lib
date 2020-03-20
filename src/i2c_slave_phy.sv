import i2c_slave_pkg::*;

module i2c_slave_phy #(
  parameter int CLK_T = 10000
)(
  input          clk_i,
  input          rst_i,
  input          sda_i,
  output         sda_o,
  output         sda_oe,
  input          scl_i,
  input  [1 : 0] cmd_i,
  output         start_o,
  output         stop_o,
  output         data_o,
  input          data_i,
  output         cmd_done_o,
  output         ready_o
);

localparam int SPIKE_FILT_L    = 50000 / CLK_T;
localparam int DATA_HOLD_PS    = 300000;
localparam int DATA_HOLD_TICKS = DATA_HOLD_PS / CLK_T + 1;
localparam int CNT_W           = $clog2( DATA_HOLD_TICKS );

logic [1 : 0]                sda_mtstb;
logic [1 : 0]                scl_mtstb;
logic [SPIKE_FILT_L - 1 : 0] sda_filt_d;
logic [SPIKE_FILT_L - 1 : 0] scl_filt_d;
logic                        sda_filt, sda_filt_d1;
logic                        scl_filt, scl_filt_d1;
logic                        sda_posedge;
logic                        sda_negedge;
logic                        scl_posedge;
logic                        scl_negedge;
logic                        hold_cnt_en;
logic [CNT_W : 0]            hold_cnt;
logic                        rx_data_val;
logic                        wr_done;
logic                        write_in_progress;

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
      sda_filt_d1 <= 1'b1;
      scl_filt_d1 <= 1'b1;
    end
  else
    begin
      sda_filt_d1 <= sda_filt;
      scl_filt_d1 <= scl_filt;
    end

assign sda_posedge = sda_filt && !sda_filt_d1;
assign sda_negedge = !sda_filt && sda_filt_d1;
assign scl_posedge = scl_filt && !scl_filt_d1;
assign scl_negedge = !scl_filt && scl_filt_d1;

enum logic [2 : 0] { IDLE_S,
                     RD_WAIT_SCL_POSEDGE_S,
                     RD_WAIT_SCL_NEGEDGE_S,
                     WR_WAIT_SCL_POSEDGE_S,
                     WR_WAIT_SCL_NEGEDGE_S,
                     WR_WAIT_SDA_HOLD_S } state, next_state;

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
          if( cmd_i == READ )
            next_state = RD_WAIT_SCL_POSEDGE_S;
          else
            if( cmd_i == WRITE )
              next_state = WR_WAIT_SCL_POSEDGE_S;
        end
      RD_WAIT_SCL_POSEDGE_S:
        begin
          if( scl_posedge )
            next_state = RD_WAIT_SCL_NEGEDGE_S;
        end
      RD_WAIT_SCL_NEGEDGE_S:
        begin
          if( sda_posedge || sda_negedge || scl_negedge )
            next_state = IDLE_S;
        end
      WR_WAIT_SCL_POSEDGE_S:
        begin
          if( scl_posedge )
            next_state = WR_WAIT_SCL_NEGEDGE_S;
        end
      WR_WAIT_SCL_NEGEDGE_S:
        begin
          if( sda_posedge || sda_negedge )
            next_state = IDLE_S;
          else
            if( scl_negedge )
              next_state = WR_WAIT_SDA_HOLD_S;
        end
      WR_WAIT_SDA_HOLD_S:
        begin
          if( hold_cnt == DATA_HOLD_TICKS )
            next_state = IDLE_S;
        end
    endcase
  end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    hold_cnt <= '0;
  else
    if( !hold_cnt_en )
      hold_cnt <= '0;
    else
      hold_cnt <= hold_cnt + 1'b1;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    hold_cnt_en <= 1'b0;
  else
    if( state == WR_WAIT_SCL_NEGEDGE_S && scl_negedge )
      hold_cnt_en <= 1'b1;
    else
      if( hold_cnt == ( DATA_HOLD_TICKS[CNT_W : 0] - 1'b1 ) )
        hold_cnt_en <= 1'b0;

assign rx_data_val       = state == RD_WAIT_SCL_NEGEDGE_S && scl_negedge;
assign wr_done           = state == WR_WAIT_SDA_HOLD_S && hold_cnt == DATA_HOLD_TICKS[CNT_W : 0];
assign write_in_progress = state == WR_WAIT_SCL_NEGEDGE_S || state == WR_WAIT_SCL_POSEDGE_S || 
                           state == WR_WAIT_SDA_HOLD_S || state == IDLE_S && cmd_i == WRITE;

assign start_o    = scl_filt && sda_negedge;
assign stop_o     = scl_filt && sda_posedge;
assign ready_o    = state == IDLE_S;
assign cmd_done_o = rx_data_val || wr_done;
assign data_o     = sda_filt;
assign sda_o      = 1'b0;
assign sda_oe     = write_in_progress && !data_i;

endmodule
