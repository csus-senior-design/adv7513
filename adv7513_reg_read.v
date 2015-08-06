/*
----------------------------------------
Stereoscopic Vision System
Senior Design Project - Team 11
California State University, Sacramento
Spring 2015 / Fall 2015
----------------------------------------

Analog Devices ADV7513 Register Debug
Authors:  Greg M. Crist, Jr. (gmcrist@gmail.com)
          Padraic Hagerty    (guitarisrockin@hotmail.com)

Description:
  Allows for reading of registers
*/
module adv7513_reg_read  #(
        parameter CHIP_ADDR = 7'h39,
        parameter I2C_CLKDIV = 206,
        parameter I2C_TXN_DELAY = 0
    )(
        clk,
        reset,
        sda,
        scl,
        start,
        done,
        reg_addr_in,
        reg_data
    );

    input clk, reset;
    inout sda, scl;
    input start;
    output reg done;

    input [7:0] reg_addr_in;
    output reg [7:0] reg_data;

    (* syn_encoding = "safe" *)
    reg [1:0] state;
    reg [6:0] chip_addr;
    
    reg [7:0] reg_addr;
    
    reg [25:0]  cnt;
    reg         cnt_en;

    localparam s_idle = 0,
               s_cmd  = 1,
               s_wait = 2,
               s_done = 3;
               
    localparam DLY_1s = 26'd50000000;

    reg [7:0] data_in = 8'h0;
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

    i2c_master #(
        .ADDR_BYTES(1),
        .DATA_BYTES(1))
    i2c_master (
        .clk        (clk),
        .reset      (reset),
        .open_drain (1'b1),
        .clk_div    (I2C_CLKDIV),
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


//    assign done = ~i2c_busy && state == s_idle && prev_state == s_done;

    // SDA Input / Output
    //assign sda_in = sda;
    assign sda = (sda_oen == 0) ? sda_out : 1'bz;

    // SCL Input / Output
    //assign scl_in = scl;
    assign scl = (scl_oen == 0) ? scl_out : 1'bz;

    always @ (posedge clk) begin
        // Flop the input so it's not lost
        reg_addr <= reg_addr_in;
    
        if (~reset) begin
            done        <= 1'b0;
            state       <= s_idle;
            write_en    <= 1'b0;
            read_en     <= 1'b0;
            reg_data    <= 8'h00;
            cnt         <= 26'd0;
            cnt_en      <= 1'b0;
        end
        else begin
            read_en <= 1'b0;
            write_en <= 1'b0;
            case (state)
                s_idle: begin
                    state <= start ? s_cmd : s_idle;
                    
                    if (done && cnt == DLY_1s) begin
                        done <= 1'b0;
                        cnt <= 26'd0;
                        cnt_en <= 1'b0;
                    end else if (cnt_en)
                        cnt <= cnt + 26'd1;
                end

                s_cmd: begin
                    read_i2c(CHIP_ADDR, reg_addr);
                    done     <= 1'b0;
                    state    <= s_wait;
                end

                s_wait: begin
                    //write_en <= 1'b0;
                    //read_en  <= 1'b0;
                    state    <= (read_en || i2c_busy) ? s_wait : s_done;
                end

                s_done: begin
                    done     <= 1'b1;
                    reg_data <= data_out;
                    state    <= s_idle;
                    cnt <= 26'd0;
                    cnt_en <= 1'b1;
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
            data_in   <= t_data;
            write_en  <= 1'b1;
        end
    endtask

    task read_i2c;
        input [6:0] t_chip_addr;
        input [7:0] t_reg_addr;

        begin
            chip_addr <= t_chip_addr;
            read_en   <= 1'b1;
        end
    endtask
endmodule
