module user_proj_example #(
    parameter BITS = 8
)(
`ifdef USE_POWER_PINS
    inout vccd1,    // User area 1 1.8V supply
    inout vssd1,    // User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    //input wb_rst_i,
    input wbs_stb_i,
    // input wbs_cyc_i,
    input wbs_we_i,
    //input [3:0] wbs_sel_i,
    input [BITS-1:0] wbs_dat_i,
    //input [31:0] wbs_adr_i,
    //output wbs_ack_o,
    output [BITS-1:0] wbs_dat_o,

    
    // Logic Analyzer Signals
    

    // IOs
    inout  sda,
    output scl,

    // IRQ
   // output [2:0] irq
);
    wire i_clk;
    wire i_strobe;
    wire [BITS-1:0] i_addr_data; 
    wire i_cmd;
    wire io_sda;
    wire io_scl;
    wire [BITS-1:0]o_data;
    //wire [2:0]o_status;
    //wire [BITS-1:0] count;
    //wire valid;
    //wire [3:0] wstrb;
    //wire [BITS-1:0] la_write;

    // WB MI A
   // assign valid = wbs_cyc_i && wbs_stb_i; 
   // assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    //assign wbs_dat_o = {{(32-BITS){1'b0}}, rdata};
    //assign wdata = wbs_dat_i[BITS-1:0];

    // IO
    assign scl = io_scl;
    assign sda = io_sda;

    // IRQ
    //assign irq = 3'b000;    // Unused

    // LA
    //assign la_data_out = {{(128-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    //assign la_write = ~la_oenb[63:64-BITS] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign i_clk = wb_clk_i;
    assign i_strobe=wbs_stb_i;
    assign i_addr_data=wbs_dat_i[BITS-1:0];
    assign i_cmd=wbs_we_i;
    assign o_data=wbs_dat_o[BITS-1:0];

    //assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    i2c_master #(
    ) i2c_master(
        .i_clk(i_clk),
        .i_strobe(i_strobe),
        .i_addr_data(i_addr_data),
        .i_cmd(i_cmd),
        .io_sda(io_sda),
        .io_scl(io_scl),
        .o_data(o_data[7:0])
        //.o_status(o_status[17:15])
        //.la_input(la_data_in[63:64-BITS]),
        //.count(count)
    );

endmodule

module i2c_master #(
)(
    input [7:0] i_addr_data,        // Address and Data
    input i_cmd,            // Command (r/w)
    input i_strobe,            // Latch inputs
    input i_clk,
    inout io_sda,
    output io_scl,
    output reg [7:0] o_data,        // Output data on reads
    //output wire [2:0] o_status        // Request status
    );

parameter ST_CMD_ADDR = 1,
         ST_CMD_DATA = 2,
         ST_SEND_START = 3,
         ST_RD_DATA = 4,
         ST_WR_DATA = 5,
         ST_SEND_STOP = 6,
         ST_CHECK_WR_ACK = 7,
         ST_CHECK_ADDR_ACK = 8,
         ST_SEND_ACK = 9,
         ST_WR_ADDR = 10;

reg [6:0] addr;    // Address
reg cmd;
wire [7:0] addr_cmd;
assign addr_cmd = {addr, cmd};

reg status_err_nack_addr, status_err_nack_data, status_data_ready;
//assign o_status = {status_err_nack_addr, status_err_nack_data, status_data_ready };

assign io_sda = reg_sda;
reg reg_sda;
     
reg [3:0] state;
reg [3:0] pos_state;
reg [3:0] pos_count;
reg [3:0] neg_state;
reg [3:0] neg_count;

reg wr_sda_neg;
reg wr_sda_pos;    /* Write enable for positive or negative edge of clock  */
reg reg_sda_pos;
reg reg_sda_neg;    /* Data register for SDA for positive or negative clock */
reg [7:0] in_data;/* Data latched from input for write or Buffer for read*/

assign io_scl = i_clk;
initial
begin
    wr_sda_neg = 0;
    wr_sda_pos = 0;
    reg_sda_neg = 0;
    reg_sda_pos = 0;
    status_err_nack_addr = 0;
    status_err_nack_data = 0;
    status_data_ready = 0;
    pos_state = ST_CMD_ADDR;
    neg_state = 0;
    pos_count = 8;
    neg_count = 9;
end

/*
 * There are 2 state variables triggered on pos and neg edge.
 * Always set the state sampled on the next edge based
 * on what the state changed to on the previous edge.
 * This will prevent the following case:
 * on falling edge, state change to check ack. on rising edge,
 * ack is checked and state is changed to write-data. then on
 * next following edge, state should be write-data and not
 * (write-data | check-ack). So we use a priority logic. The or
 * logic wont work. */

always @*
begin
    case (i_clk)
        1:
        begin
            if (pos_state != 0)
                state <= pos_state;
            else
                state <= neg_state;

            /*
             * Carry forward the bit being written in the previous edge
             * (pos/neg) if nothing is to be written on the current edge.
             * This will make sure data remains written for a complete
             * clock cycle.
             */
            if (wr_sda_pos == 1'b1)
                reg_sda = reg_sda_pos;
            else if(wr_sda_neg == 1'b1)
                reg_sda = reg_sda_neg;
            else
                reg_sda = 1'bZ;
        end
        0:
        begin
            if (neg_state != 0)
                    state <= neg_state;
            else
                    state <= pos_state;

            if (wr_sda_neg == 1'b1)
                reg_sda = reg_sda_neg;
            else if(wr_sda_pos == 1'b1)
                reg_sda = reg_sda_pos;
            else
                reg_sda = 1'bZ;
        end
        endcase;
end

always @(posedge i_clk)
begin
    pos_state <= 0;
    wr_sda_pos <= 0;

    case(state)
    ST_CMD_ADDR:
    if (i_strobe)
    begin
        addr <= i_addr_data[6:0];
        cmd <= i_cmd;
        if (i_cmd == 1)                // Read
            pos_state <= ST_SEND_START;
        else
            pos_state <= ST_CMD_DATA;
    end
    else
            pos_state <= ST_CMD_ADDR;

    ST_CMD_DATA:
    if (i_strobe)
    begin
        in_data <= i_addr_data;
        pos_state <= ST_SEND_START;
    end

    ST_SEND_START:
    begin
        wr_sda_pos <= 1;
        reg_sda_pos <= 0;
        pos_state <= ST_WR_ADDR;
    end
    ST_SEND_STOP:
    begin
        wr_sda_pos <= 1;
        reg_sda_pos <= 1;
        pos_state <= ST_CMD_ADDR;
        status_data_ready <= 1;
    end
    
    ST_SEND_ACK:
    begin
    /* Ack was sent last falling edge, and the slave is sampling it.
     * Prepare for Stop command next falling edge */
        pos_state <= ST_SEND_STOP;
        o_data <= in_data;
    end
    endcase

    if (state == ST_CHECK_WR_ACK || state == ST_CHECK_ADDR_ACK)
        if (io_sda != 0)                /* Its a write NACK */
        begin
            if (state == ST_CHECK_ADDR_ACK)
                status_err_nack_addr <= 1;
            else
                status_err_nack_data <= 1;
            pos_state <= ST_CMD_ADDR;    /* Back to default state; */
        end
        else
        begin
            if (state == ST_CHECK_ADDR_ACK)
                if (cmd == 1)
                    pos_state <= ST_RD_DATA;
                else
                    pos_state <= ST_WR_DATA;
            else
                pos_state <= ST_SEND_STOP;
        end

    if (state == ST_RD_DATA)
    begin
        if (pos_count > 0)
        begin
            in_data[pos_count-1] <= io_sda;    // pos_count = 8..1
            pos_count <= pos_count - 1'b1;
        end
        if (pos_count == 1)
        begin
            /*
             * We just read the last data bit. Prepare to put Ack on the bus on the
             * next falling edge.
             */
             pos_state <= ST_SEND_ACK;
             pos_count <= 8;
        end
        else
            /*
             * More bits to go, continue to read
             */
            pos_state <= ST_RD_DATA;
    end
end

always @(negedge i_clk)
begin
    neg_state <= 0;
    wr_sda_neg <= 0; /* Always release bus on next falling edge by default */

    if (state == ST_WR_DATA || state == ST_WR_ADDR)
    begin
        if (neg_count == 1)
        begin
            neg_count <= 9;            /* Reset counter for future use */
            if (state == ST_WR_DATA)
                neg_state <= ST_CHECK_WR_ACK;
            else
                neg_state <= ST_CHECK_ADDR_ACK;
        end
        else
        begin
            wr_sda_neg <= 1;
            if (state == ST_WR_DATA)
                reg_sda_neg <= in_data[neg_count - 2]; /* neg_count = 9..2 */
            else
                reg_sda_neg <= addr_cmd[neg_count - 2];
            neg_count <= neg_count - 1'b1;
            neg_state <= state;        /* Continue to write data or address */
        end
    end

    if (state == ST_SEND_STOP || state == ST_SEND_ACK)
    begin
        /*
         * Before next rising edge, pull the Data line low.
         * Incase of stop command, next rising edge will pull line high.
         * Incase of Ack, next rising edge will keep line low to signal Ack.
         */
        wr_sda_neg <= 1;
        reg_sda_neg <= 0;
    end
end
endmodule
`default_nettype wire

