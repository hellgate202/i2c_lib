import i2c_pkg::*;

module i2c_master_phy#(
  parameter int SCL_FREQ = 400_000,
  parameter int CLK_FREQ = 100_000_000
)(
  input                clk_i,
  input                rst_i,
  input        [2 : 0] cmd_i,
  input                data_i,
  output logic         data_o,
  output logic         cmd_done_o,
  output logic         arb_lost_o,
  output logic         bus_busy_o,
  input                sda_i,
  output logic         sda_o,
  output logic         sda_oe,
  input                scl_i,
  output logic         scl_o,
  output logic         scl_oe
);

localparam int CLK_T            = 64'd1_000_000_000_000 / CLK_FREQ;
localparam int SCL_T            = 64'd1_000_000_000_000 / SCL_FREQ;
localparam int TICKS_PER_SCL_T  = SCL_T / CLK_T;
localparam int TICKS_PER_STATE  = TICKS_PER_SCL_T / 5;
localparam int TICKS_PER_SAMPLE = TICKS_PER_SCL_T / 16;

localparam int SMPL_CNT_WIDTH   = $clog2( TICKS_PER_SAMPLE );
localparam int STATE_CNT_WIDTH  = $clog2( TICKS_PER_STATE );


logic [1 : 0]                   mstb_sda, mstb_scl;
logic [2 : 0]                   spk_flt_sda, spk_flt_scl;
logic                           sda, scl;
logic                           sda_d, scl_d, scl_oe_d;

logic [SMPL_CNT_WIDTH - 1 : 0]  sample_cnt;
logic                           sample_lane;
logic [STATE_CNT_WIDTH - 1 : 0] state_cnt;
logic                           next_state_allowed;
logic                           scl_stretch;
logic                           scl_driven;
logic                           arb_lost;
logic                           stop_req;
logic                           stop_detected;
logic                           start_detected;
logic                           bus_busy;

assign sample_lane        = sample_cnt == TICKS_PER_SAMPLE;
assign next_state_allowed = state_cnt == TICKS_PER_SAMPLE ||
                            scl_driven;
assign scl_o              = 1'b0;
assign sda_o              = 1'b0;

assign scl_driven         = scl_d && ~scl && ~scl_oe;

enum logic [4 : 0] { IDLE_S,
                     START_SDA_HIGH_S,
                     START_SCL_HIGH_S,
                     START_SDA_LOW_0_S,
                     START_SDA_LOW_1_S,
                     START_SCL_LOW_S,
                     STOP_SDA_LOW_S,
                     STOP_SCL_HIGH_0_S,
                     STOP_SCL_HIGH_1_S,
                     STOP_SDA_HIGH_S,
                     READ_SCL_LOW_0_S,
                     READ_SCL_HIGH_0_S,
                     READ_SCL_HIGH_1_S,
                     READ_SCL_LOW_1_S,
                     WRITE_SCL_LOW_0_S,
                     WRITE_SCL_HIGH_0_S,
                     WRITE_SCL_HIGH_1_S,
                     WRITE_SCL_LOW_1_S } state, next_state;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    state <= IDLE_S;
  else
    if( next_state_allowed )
      state <= next_state;

always_comb
  begin
    next_state = state;
    case( state )
      IDLE_S:
        begin
          case( cmd_i )
            START:   next_state = START_SDA_HIGH_S;
            STOP:    next_state = STOP_SDA_LOW_S;
            READ:    next_state = READ_SCL_LOW_0_S;
            WRITE:   next_state = WRITE_SCL_LOW_0_S;
            default: next_state = IDLE_S;
          endcase
        end
      START_SDA_HIGH_S:   next_state = START_SCL_HIGH_S;
      START_SCL_HIGH_S:   next_state = START_SDA_LOW_0_S;
      START_SDA_LOW_0_S:  next_state = START_SDA_LOW_1_S;
      START_SDA_LOW_1_S:  next_state = START_SCL_LOW_S;
      START_SCL_LOW_S:    next_state = IDLE_S;
      STOP_SDA_LOW_S:     next_state = STOP_SCL_HIGH_0_S;
      STOP_SCL_HIGH_0_S:  next_state = STOP_SCL_HIGH_1_S;
      STOP_SCL_HIGH_1_S:  next_state = STOP_SDA_HIGH_S;
      STOP_SDA_HIGH_S:    next_state = IDLE_S;
      READ_SCL_LOW_0_S:   next_state = READ_SCL_HIGH_0_S;
      READ_SCL_HIGH_0_S:  next_state = READ_SCL_HIGH_1_S;
      READ_SCL_HIGH_1_S:  next_state = READ_SCL_LOW_1_S;
      READ_SCL_LOW_1_S:   next_state = IDLE_S;
      WRITE_SCL_LOW_0_S:  next_state = WRITE_SCL_HIGH_0_S;
      WRITE_SCL_HIGH_0_S: next_state = WRITE_SCL_HIGH_1_S;
      WRITE_SCL_HIGH_1_S: next_state = WRITE_SCL_LOW_1_S;
      WRITE_SCL_LOW_1_S:  next_state = IDLE_S;
    endcase
  end

assign cmd_done_o = state != IDLE_S && next_state == IDLE_S && next_state_allowed;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      mstb_sda <= '1;
      mstb_scl <= '1;
    end
  else
    begin
      mstb_sda <= { mstb_sda[0], sda_i };
      mstb_scl <= { mstb_scl[0], scl_i };
    end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      spk_flt_sda <= '1;
      spk_flt_scl <= '1;
    end
  else
    begin
      spk_flt_sda <= { spk_flt_sda[1 : 0], mstb_sda[1] };
      spk_flt_scl <= { spk_flt_scl[1 : 0], mstb_scl[1] };
    end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      sda <= 1'b1;
      scl <= 1'b1;
    end
  else
    if( sample_lane )
      begin
        sda <= ( ( spk_flt_sda[0] && spk_flt_sda[1] ) ||
                 ( spk_flt_sda[1] && spk_flt_sda[2] ) ||
                 ( spk_flt_sda[0] && spk_flt_sda[2] ) );
        scl <= ( ( spk_flt_scl[0] && spk_flt_scl[1] ) ||
                 ( spk_flt_scl[1] && spk_flt_scl[2] ) ||
                 ( spk_flt_scl[0] && spk_flt_scl[2] ) );
      end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      sda_d <= 1'b1;
      scl_d <= 1'b1;
    end
  else
    begin
      sda_d <= sda;
      scl_d <= scl;
    end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    sample_cnt <= '0;
  else
    if( sample_cnt == TICKS_PER_SAMPLE )
      sample_cnt <= '0;
    else
      sample_cnt <= sample_cnt + 1'b1;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    state_cnt <= '0;
  else
    if( state_cnt == TICKS_PER_STATE || scl_driven )
      state_cnt <= '0;
    else
      if( ~scl_stretch )
        state_cnt <= state_cnt + 1'b1;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    scl_oe_d <= 1'b0;
  else
    scl_oe_d <= scl_oe;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    scl_stretch <= 1'b0;
  else
    if( ~scl_oe && scl_oe_d && ~scl )
      scl_stretch <= 1'b1;
    else
      if( scl )
        scl_stretch <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    stop_req <= 1'b0;
  else
    if( cmd_i == STOP && next_state_allowed )
      stop_req <= 1'b1;
    else
      if( next_state_allowed )
        stop_req <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    begin
      stop_detected  <= 1'b0;
      start_detected <= 1'b0;
    end
  else
    begin
      stop_detected  <= sda && ~sda_d && scl;
      start_detected <= ~sda && sda_d && scl;
    end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    arb_lost_o <= 1'b0;
  else
    if( ( ~sda_oe && ~sda && state == WRITE_SCL_HIGH_1_S ) ||
        ( stop_detected && ~stop_req ) )
      arb_lost_o <= 1'b1;
    else
      arb_lost_o <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    bus_busy_o <= 1'b0;
  else
    if( start_detected )
      bus_busy_o <= 1'b1;
    else
      if( stop_detected )
        bus_busy_o <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    data_o <= 1'b0;
  else
    if( scl && ~scl_d )
      data_o <= sda;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    sda_oe <= 1'b0;
  else
    case( state )
      START_SDA_HIGH_S:  sda_oe <= 1'b0;
      START_SDA_LOW_0_S: sda_oe <= 1'b1;
      STOP_SDA_LOW_S:    sda_oe <= 1'b1;
      STOP_SDA_HIGH_S:   sda_oe <= 1'b0;
      READ_SCL_LOW_0_S:  sda_oe <= 1'b0;
      WRITE_SCL_LOW_0_S: sda_oe <= ~data_i;
    endcase

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    scl_oe <= 1'b0;
  else
    case( state )
      START_SCL_HIGH_S:   scl_oe <= 1'b0;
      START_SCL_LOW_S:    scl_oe <= 1'b1;
      STOP_SCL_HIGH_0_S:  scl_oe <= 1'b0;
      READ_SCL_LOW_0_S:   scl_oe <= 1'b1;
      READ_SCL_HIGH_0_S:  scl_oe <= 1'b0;
      READ_SCL_LOW_1_S:   scl_oe <= 1'b1;
      WRITE_SCL_LOW_0_S:  scl_oe <= 1'b1;
      WRITE_SCL_HIGH_0_S: scl_oe <= 1'b0;
      WRITE_SCL_LOW_1_S:  scl_oe <= 1'b1;
    endcase

endmodule
