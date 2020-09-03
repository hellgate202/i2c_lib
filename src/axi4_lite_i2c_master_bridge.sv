/*
  Performs conversion between AXI4-Lite and I2C
  Device address is defined in slave_addr_i signal
  Data over data channels and address over adresses channels must come in the same tick
  All AXI4-Lite address bytes are treated as register address
  in slave deivce.
  All AXI4-Lite write data bytes will be written in single I2C transaction,
  with respect to wstrb signal, but wstrb supports only right alligned values
  like 0b0001, 0b0011, 0b0111, 0b1111.
  So to full support of AXI4-Lite you will need to place a transaction aligner
  before this module.
  In case of NACK on slave device error will be transmitted through bresp signal
*/

// Package with I2C commands
import i2c_master_pkg::*;

module axi4_lite_i2c_master_bridge #(
  parameter int CLK_FREQ   = 100_000_000,
  parameter int SCL_FREQ   = 400_000,
  parameter int DATA_WIDTH = 8,
  parameter int ADDR_WIDTH = 8
)(
  input              clk_i,
  input              rst_i,
  input [6 : 0]      slave_addr_i,
  axi4_lite_if.slave axi4_lite_i,
  // I2C signals passed in such way to be platform
  // independant, they need to be connected to tristate buffers
  input              sda_i,
  output             sda_o,
  output             sda_oe,
  input              scl_i,
  output             scl_o,
  output             scl_oe
);

localparam int BYTES_PER_ADDR  = ADDR_WIDTH / 8;
localparam int BYTES_PER_DATA  = DATA_WIDTH / 8;
localparam int SHIFT_REG_WIDTH = 8 + DATA_WIDTH + ADDR_WIDTH;
localparam int ADDR_CNT_WIDTH  = $clog2( BYTES_PER_DATA + 1 );
localparam int DATA_CNT_WIDTH  = $clog2( BYTES_PER_DATA + 1 );

// All transmited data stored here
logic [SHIFT_REG_WIDTH - 1 : 0] shift_reg;
logic                           shift;
logic                           tx_bit;
logic                           rx_bit;
// Conditions when we need to interrupt transmission
logic                           arb_lost;
logic                           sda_error;
logic                           scl_error;
logic                           bus_error;
// 0 - read, 1 - write
logic                           rw;
logic [1 : 0]                   phy_cmd;
logic                           phy_cmd_done;
logic [2 : 0]                   bit_cnt;
logic [ADDR_CNT_WIDTH - 1 : 0]  addr_byte_cnt;
logic [DATA_CNT_WIDTH - 1 : 0]  data_byte_cnt;
logic [DATA_CNT_WIDTH - 1 : 0]  axi_data_bytes;
logic                           tx_in_progress;
logic [DATA_WIDTH - 1 : 0]      rx_data_reg;
logic                           nack_received;
logic                           write_done;
logic                           unfinished_resp;

enum logic [3 : 0] { IDLE_S,
                     START_S,
                     // Slave address transmission during
                     // register address transaction
                     SA_RA_S,
                     SA_RA_ACK_S,
                     // Register address transmission
                     RA_S,
                     RA_ACK_S,
                     // Repeated start
                     RSTART_S,
                     // Slave address transmission during
                     // read transaction
                     SA_RD_S,
                     SD_RD_ACK_S,
                     // Receiving data
                     RD_DATA_S,
                     RD_DATA_ACK_S,
                     RD_DATA_NACK_S,
                     // Transmitting data
                     WR_DATA_S,
                     WR_DATA_ACK_S,
                     STOP_S } state, next_state;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    state <= IDLE_S;
  else
    if( bus_error )
      state <= IDLE_S;
    else
      state <= next_state;

/*
  Same begining for read and write:
  1) Start condition
  2) Sending slave address with write bit
  3) Sending all bytes of register address
  For read transaction:
  4) Repeated start condition
  5) Sending slave address with read bit
  6) Receiving all read data bytes (with ACK after every but the last)
  7) Sending NACK
  8) Stop condition
  For write transaction:
  4) Send all write data bytes
  5) Stop condition

  For every sent byte we check for ACK bit
  If it is NACK - we terminate transmission through stop condition
*/

always_comb
  begin
    next_state = state;
    case( state )
      IDLE:
        begin
          // Read or write request over AXI4
          if( ( axi4_lite_i.awvalid || axi4_lite_i.arvalid ) && !unfinished_resp )
            next_state = START_S;
        end
      START_S:
        begin
          if( phy_cmd_done )
            next_state = SA_RA_S;
        end
      SA_RA_S:
        begin
          if( phy_cmd_done && bit_cnt == 3'd7 )
            next_state = SA_RA_ACK_S;
        end
      SA_RA_ACK_S:
        begin
          if( phy_cmd_done )
            if( rx_bit )
              next_state = STOP_S;
            else
              next_state = RA_S;
        end
      RA_S:
        begin
          if( phy_cmd_done && bit_cnt == 3'd7 )
            next_state = RA_ACK_S;
        end
      RA_ACK_S:
        begin
          if( phy_cmd_done )
            if( rx_bit )
              next_state = STOP_S;
            else
              if( addr_byte_cnt == ADDR_CNT_WIDTH'( BYTES_PER_ADDR ) )
                if( rw )
                  next_state = WR_DATA_S;
                else
                  next_state = RSTART_S;
              else
                next_state = RA_S;
        end
      WR_DATA_S:
        begin
          if( phy_cmd_done && bit_cnt == 3'd7 )
            next_state = WR_DATA_ACK_S;
        end
      WR_DATA_ACK_S:
        begin
          if( phy_cmd_done )
            if( rx_bit )
              next_state = STOP_S;
            else
              if( data_byte_cnt == axi_data_bytes )
                next_state = STOP_S;
              else
                next_state = WR_DATA_S;
        end
      RSTART_S:
        begin
          if( phy_cmd_done )
            next_state = SA_RD_S;
        end
      SA_RD_S:
        begin
          if( phy_cmd_done && bit_cnt == 3'd7 )
            next_state = SA_RD_ACK;
        end
      SA_RD_ACK_S:
        begin
          if( phy_cmd_done )
            if( rx_bit )
              next_state = STOP_S;
            else
              next_state = RD_DATA_S;
        end
      RD_DATA_S:
        begin
          if( phy_cmd_done && bit_cnt == 3'd7 )
            // -1 beacuse it will inctrement in the same tick as this check
            if( data_byte_cnt == DATA_CNT_WIDTH'( ( BYTES_PER_DATA - 1 ) ) )
              next_state = RD_DATA_NACK_S;
            else
              next_state = RD_DATA_ACK_S;
        end
      RD_DATA_ACK_S:
        begin
          if( phy_cmd_done )
            next_state = RD_DATA_S;
        end
      RD_DATA_NACK_S:
        begin
          if( phy_cmd_done )
            next_state = STOP_S;
        end
      STOP_S:
        begin
          if( phy_cmd_done )
            next_state = IDLE_S;
        end
    endcase
  end

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    shift_reg <= SHIFT_REG_WIDTH'( 0 );
  else
    // We initialize shift reg with data when AXI4-Lite write or read command
    // arrives
    if( state == IDLE_S )
      begin
        if( axi4_lite_i.awvalid )
          shift_reg <= { slave_addr_i, 1'b0, axi4_lite_i.awaddr, axi4_lite_i.awdata };
        else
          if( axi4_lite_i.arvalid )
            if( BYTES_PER_DATA == 1 )
              shift_reg <= { slave_addr_i, 1'b1, axi4_lite_i.araddr, slave_addr_i, 1'b0 };
            else
              shift_reg <= { slave_addr_i, 1'b1, axi4_lite_i.araddr, slave_addr_i, 1'b0,
                             ( DATA_WIDTH - 8 )'( 0 ) };
      end
    else
      // Shifting MSB first
      if( shift )
        shift_reg <= shift_reg << 1;

// Amount of data bytes based on wstrb signal (left-most zero)
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    axi_data_bytes <= DATA_CNT_WIDTH'( 0 );
  else
    if( state == IDLE_S )
      if( &axi4_lite_i.wstrb )
        axi_data_bytes <= DATA_CNT_WIDTH'( BYTES_PER_DATA );
      else
        for( int i = 0; i < BYTES_PER_DATA; i++ )
          if( !axi4_lite_i.wstrb[i] )
            axi_data_bytes <= DATA_CNT_WIDTH'( i ); 

// Shifting when we need to send new bit from shift_reg (slave address, reg
// address and write data)
assign shift     = phy_cmd_done && ( state == SA_RA_S || state == RA_S || state == SA_RD_S ||
                                     state == WR_DATA_S );
assign tx_bit    = state == RD_DATA_ACK_S ? 1'b0 : state == RD_DATA_NACK_S ? 1'b1 :
                   shift_reg[SHIFT_REG_WIDTH - 1];
assign bus_error = arb_lost || sda_error || scl_error;

// Holding what type of operation we are performing
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    rw <= 1'b0;
  else
    if( state == IDLE_S )
      if( axi4_lite_i.awvalid )
        rw <= 1'b1;
      else
        if( axi4_lite_i.arvalid )
          rw <= 1'b0;

// When we sending actual data (data or address)
assign tx_in_progress = state == SA_RA_S || state == SA_RD_S || state == RA_S ||
                        state == WR_DATA_S || state == RD_DATA_S;
assign nack_received  = phy_cmd_done && rx_bit &&
                        ( state == SA_RA_ACK_S || state == RA_ACK_S || state == WR_DATA_ACK_S ||
                          state == SA_RD_ACK_S )
assign write_done     = phy_cmd_done && !rx_bit && state == WR_DATA_ACK_S &&
                        data_byte_cnt == DATA_CNT_WIDTH'( BYTES_PER_DATA );
assign read_done      = phy_cmd_done && state == RD_DATA_NACK_S;

// Counter for rx/tx bits
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    bit_cnt <= 3'd0;
  else
    if( bus_error )
      bit_cnt <= 3'd0;
    else
      if( tx_in_progress && phy_cmd_done )
        bit_cnt <= bit_cnt + 1'b1;

// Counter for tx address bytes
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    addr_byte_cnt <= ADDR_CNT_WIDTH'( 0 );
  else
    if( state == IDLE_S )
      addr_byte_cnt <= ADDR_CNT_WIDTH'( 0 );
    else
      if( state == RA_S && phy_cmd_done && bit_cnt == 3'd7 )
        addr_byte_cnt <= addr_byte_cnt + 1'b1;

// Counter for tx/rx data bytes
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    data_byte_cnt <= DATA_CNT_WIDTH'( 0 );
  else
    if( state == IDLE_S )
      data_byte_cnt <= DATA_CNT_WIDTH'( 0 );
    else
      if( ( state == RD_DATA_S || state == WR_DATA_S ) && 
          phy_cmd_done && bit_cnt == 3'd7 )
        data_byte_cnt <= data_byte_cnt + 1'b1;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    rx_data_reg <= DATA_WIDTH'( 0 );
  else
    if( state == RD_DATA_S && phy_cmd_done )
      rx_data_reg <= { rx_data_reg[DATA_WIDTH - 2 : 0], rx_bit };

// Bit level I2C controller
i2c_master_phy #(
  .SCL_FREQ   ( SCL_FREQ     ),
  .CLK_FREQ   ( CLK_FREQ     )
) i2c_bit_controller (
  .clk_i      ( clk_i        ),
  .rst_i      ( rst_i        ),
  .cmd_i      ( phy_cmd      ),
  .data_i     ( tx_bit       ),
  .data_o     ( rx_bit       ),
  .cmd_done   ( phy_cmd_done ),
  .arb_lost_o ( arb_lost     ),
  .sda_err_o  ( sda_error    ),
  .scl_err_o  ( scl_error    ),
  // We have only one AXI4-Lite master
  // so we are the one who knocks
  .bus_busy_o (              ),
  .sda_i      ( sda_i        ),
  .sda_o      ( sda_o        ),
  .sda_oe     ( sda_oe       ),
  .scl_i      ( scl_i        ),
  .scl_o      ( scl_o        ),
  .scl_oe     ( scl_oe       )
);

// If master doesn't acknolage response
assign unfinished_resp     = axi4_lite_i.bvalid || axi4_lite_i.rvalid;

// AXI4-Lite control logic
// We ready for new transaction in IDLE and when we don have unanswered
// requests (master lower it's ready signals)
assign axi4_lite_i.awready = state == IDLE_S && !unfinished_resp;
assign axi4_lite_i.arready = state == IDLE_S && !unfinished_resp;
assign axi4_lite_i.wready  = state == IDLE_S && !unfinished_resp;

// We return write response code everytime write transaction ends
// Even if it fails
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    axi4_lite_i.bvalid <= 1'b0;
  else
    if( rw && ( nack_received || write_done || bus_error )
      axi4_lite_i.bvalid <= 1'b1;
    else
      if( axi4_lite_i.bready )
        axi4_lite_i.bvalid <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    axi4_lite_i.bresp <= 2'b00;
  else
    if( write_done )
      axi4_lite_i.bresp <= 2'b00;
    else
      // If it fails we return SLAVERR
      if( bus_error || nack_received )
        axi4_lite_i.bresp <= 2'b10;

assign axi4_lite_i.rdata = rx_data_reg;

// THe same for read channel
always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    axi4_lite_i.rvalid <= 1'b0;
  else
    if( !rw && ( nack_received || read_done || bus_error )
      axi4_lite_i.rvalid <= 1'b1;
    else
      if( axi4_lite_i.rready )
        axi4_lite_i.rvalid <= 1'b0;

always_ff @( posedge clk_i, posedge rst_i )
  if( rst_i )
    axi4_lite_i.rresp <= 2'b00;
  else
    if( read_done )
      axi4_lite_i.rresp <= 2'b00;
    else
      if( bus_error || nack_received )
        axi4_lite_i.rresp <= 2'b10;

endmodule
