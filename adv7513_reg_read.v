/*
----------------------------------------
Stereoscopic Vision System
Senior Design Project - Team 11
California State University, Sacramento
Spring 2015 / Fall 2015
----------------------------------------

Omnivision ADV7513 Register Debug
Authors:  Greg M. Crist, Jr. (gmcrist@gmail.com)

Description:
  Allows for reading of registers
*/
module adv7513_reg_read(clk, reset, clk_div, sda, scl, start, done, reg_addr, reg_data);
    input clk, reset;
    inout sda, scl;
    input start;
    output done;

    input [11:0] clk_div;

    input [7:0] reg_addr;
    output reg [7:0] reg_data;

    reg [1:0] state;
    reg [6:0] chip_addr;

    localparam x_chip_addr = 7'h72;

    localparam s_idle = 0,
               s_cmd  = 1,
               s_wait = 2;

    reg [7:0] data_in;
    reg write_en;
    reg read_en = 1'b0;
    wire [2:0] i2c_status;
    wire i2c_done;
    wire i2c_busy;
    wire i2c_write_mode = 1'b0;
    wire [7:0] data_out;
    wire sda_out;
    wire sda_oen;
    wire scl_out;
    wire scl_oen;

    reg [7:0] adv_reg_data;


    i2c_master #(
        .ADDR_BYTES(1),
        .DATA_BYTES(1))
    i2c_master (
        .clk        (clk),
        .reset      (reset),
        .clk_div    (clk_div),
        .chip_addr  (chip_addr),
        .reg_addr   (reg_addr),
        .data_in    (data_in),
        .write_en   (write_en),
        .write_mode (i2c_write_mode),
        .read_en    (read_en),
        .status     (i2c_status),
        .done       (i2c_done),
        .busy       (i2c_busy),
        .data_out   (data_out),
        .sda_in     (sda),
        .scl_in     (scl),
        .sda_out    (sda_out),
        .sda_oen    (sda_oen),
        .scl_out    (scl_out),
        .scl_oen    (scl_oen));


    assign done = ~i2c_busy && i2c_done;

    // SDA Input / Output
    assign sda_in = sda;
    assign sda = (sda_oen == 0) ? sda_out : 1'bz;

    // SCL Input / Output
    assign scl_in = scl;
    assign scl = (scl_oen == 0) ? scl_out : 1'bz;

    always @ (posedge clk or negedge reset) begin
        if (~reset) begin
            state       <= s_idle;
            write_en    <= 1'b0;
            read_en     <= 1'b0;
        end
        else begin
            case (state)
                s_idle: begin
                    write_en <= 1'b0;
                    read_en  <= 1'b0;

                    state <= start ? s_cmd : s_idle;
                end

                s_cmd: begin
                    read_i2c(x_chip_addr, reg_addr);
                    state <= s_wait;
                end

                s_wait: begin
                    state <= (i2c_busy && ~i2c_done) ? s_wait : s_idle;
                end
            endcase
        end
    end

    task write_i2c;
        input [6:0] t_chip_addr;
        input [7:0] t_reg_addr;
        input [7:0] t_data;

        begin
            chip_addr <= t_chip_addr;
//            reg_addr  <= t_reg_addr;
            data_in   <= t_data;
            write_en  <= 1;
        end
    endtask

    task read_i2c;
        input [6:0] t_chip_addr;
        input [7:0] t_reg_addr;

        begin
            chip_addr <= t_chip_addr;
//            reg_addr  <= t_reg_addr;
            read_en   <= 1;
        end
    endtask
endmodule
