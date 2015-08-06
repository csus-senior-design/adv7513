/*
----------------------------------------
Stereoscopic Vision System
Senior Design Project - Team 11
California State University, Sacramento
Spring 2015 / Fall 2015
----------------------------------------

Analog Devices ADV7513 Initialization
Authors:  Greg M. Crist, Jr. (gmcrist@gmail.com)
          Padraic Hagerty    (guitarisrockin@hotmail.com)

Description:
  Initializes the ADV7513 IC on the Terasic Cyclone V Starter GX board
*/
module adv7513_init #(
        parameter CHIP_ADDR = 7'h39,
        parameter I2C_CLKDIV = 206,
        parameter I2C_TXN_DELAY = 0
    )(
        input clk,
        input reset,
        inout sda,
        inout scl,
        input start,
        output done
    );

    (* syn_encoding = "safe" *)
    reg [1:0] state;
    reg [5:0] cmd_counter;
    reg [6:0] chip_addr;

    localparam cmd_count = 40;

    localparam s_idle = 0,
               s_iter = 1,
               s_cmd  = 2,
               s_wait = 3;

    reg [7:0] reg_addr;
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
        .clk_div    (I2C_CLKDIV),
        .open_drain (1'b1),
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


    assign done = cmd_counter >= cmd_count && ~i2c_busy;

    // SDA Input / Output
    //assign sda_in = sda;
    assign sda = (sda_oen == 0) ? sda_out : 1'bz;

    // SCL Input / Output
    //assign scl_in = scl;
    assign scl = (scl_oen == 0) ? scl_out : 1'bz;

    always @ (posedge clk) begin
        if (~reset) begin
            state       <= s_idle;
            cmd_counter <= 4'd0;
            write_en    <= 1'b0;
            read_en     <= 1'b0;
        end
        else begin
            case (state)
                s_idle: begin
                    state    <= start ? s_iter : s_idle;
                    write_en <= 1'b0;
                    read_en  <= 1'b0;
                end

                s_iter: begin
                    state       <= cmd_counter < cmd_count ? s_cmd : s_idle;
                    cmd_counter <= cmd_counter + 1'b1;
                end

                s_cmd: begin
                    state <= done ? s_iter : s_wait;

                    case (cmd_counter)
                        // Wait for 10 clock cycles before initializing
                        // Power-up
                        /*10: write_i2c(CHIP_ADDR, 8'hD6, 8'hC0); // HPD is always high
                        11: write_i2c(CHIP_ADDR, 8'h41, 8'h00); // Power-up TX

                        // Required Registers
                        12: write_i2c(CHIP_ADDR, 8'h98, 8'h03);
                        13: write_i2c(CHIP_ADDR, 8'h9A, 8'hE0);
                        14: write_i2c(CHIP_ADDR, 8'h9C, 8'h30);
                        15: write_i2c(CHIP_ADDR, 9'h9D, 8'h01);
                        16: write_i2c(CHIP_ADDR, 8'hA2, 8'hA4);
                        17: write_i2c(CHIP_ADDR, 8'hA3, 8'hA4);
                        18: write_i2c(CHIP_ADDR, 8'hE0, 8'hD0);
                        19: write_i2c(CHIP_ADDR, 8'hF9, 8'h00);
                        
                        // Video mode setup
                        20: write_i2c(CHIP_ADDR, 8'h15, 8'h00); // 24 bit RGB 4:4:4 input
                        21: write_i2c(CHIP_ADDR, 8'h16, 8'h38); // 4:4:4, 8 bit per color output
                        22: write_i2c(CHIP_ADDR, 8'h17, 8'h02); // Set aspect ratio to 16:9
                        23: write_i2c(CHIP_ADDR, 8'hAF, 8'h02); // Enable HDMI mode*/
						
						// ADI recommended settings
						09: write_i2c(CHIP_ADDR, 8'hD6, 8'hC0); // HPD is always high
						10: write_i2c(CHIP_ADDR, 8'h01, 8'h00); // Set N Value(6144)
						11: write_i2c(CHIP_ADDR, 8'h02, 8'h18); // Set N Value(6144)
						12: write_i2c(CHIP_ADDR, 8'h03, 8'h00); // Set N Value(6144)
						13: write_i2c(CHIP_ADDR, 8'h15, 8'h00); // 24 bit RGB 4:4:4 input
						14: write_i2c(CHIP_ADDR, 8'h16, 8'h70); // 4:4:4, 8 bit per color output
																// ADI says to use 8'h60 for the ADV7511,
																// but we are using 8 bit color inputs,
																// not 12 bit inputs
						15: write_i2c(CHIP_ADDR, 8'h18, 8'h46);	// CSC disabled
						16: write_i2c(CHIP_ADDR, 8'h40, 8'h80); // General Control packet enable
						17: write_i2c(CHIP_ADDR, 8'h41, 8'h10); // Power down control
						18: write_i2c(CHIP_ADDR, 8'h48, 8'h08); // Data right justified
						19: write_i2c(CHIP_ADDR, 8'h49, 8'h00); // No truncation
						20: write_i2c(CHIP_ADDR, 8'h4C, 8'h00); // 12 bit output (fixed register?)
						21: write_i2c(CHIP_ADDR, 8'h55, 8'h00); // Set RGB 4:4:4 in AVI Info Frame
																// ADI says to use 8'h40 for the ADV7511
																// evaluation board, but we are not using
																// YCrCb 4:4:4
						22: write_i2c(CHIP_ADDR, 8'h56, 8'h08); // Set active format Aspect
						23: write_i2c(CHIP_ADDR, 8'h96, 8'h20); // HPD interrupt clear (setting vsync interrupt?)
						24: write_i2c(CHIP_ADDR, 8'h98, 8'h03); // ADI recommended write
						25: write_i2c(CHIP_ADDR, 8'h99, 8'h02); // "
																// skipping 8'h9A and 8'h9B (?)
						26: write_i2c(CHIP_ADDR, 8'h9C, 8'h30); // PLL filter R1 value
						27: write_i2c(CHIP_ADDR, 8'h9D, 8'h61); // Set clock divide
						28: write_i2c(CHIP_ADDR, 8'hA2, 8'hA4); // ADI recommended write
						29: write_i2c(CHIP_ADDR, 8'hA3, 8'hA4); // "
						30: write_i2c(CHIP_ADDR, 8'hA5, 8'h04); // "
						31: write_i2c(CHIP_ADDR, 8'hAB, 8'h40); // "
						32: write_i2c(CHIP_ADDR, 8'hAF, 8'h16); // Set HDMI mode
						33: write_i2c(CHIP_ADDR, 8'hBA, 8'h60); // No clock delay
						34: write_i2c(CHIP_ADDR, 8'hD1, 8'hFF); // ADI recommended write
						35: write_i2c(CHIP_ADDR, 8'hDE, 8'hD8); // "
						36: write_i2c(CHIP_ADDR, 8'hE4, 8'h60); // VCO swing reference voltage
						37: write_i2c(CHIP_ADDR, 8'hFA, 8'h7D); // Nbr of times to search for good phase
						
						// Registers that must be set according to the ADV7513
						// documentation, but are not set from the ADV7511
						// script
						38: write_i2c(CHIP_ADDR, 8'h9A, 8'hE0);
						39: write_i2c(CHIP_ADDR, 8'hE0, 8'hD0);
						40: write_i2c(CHIP_ADDR, 8'hF9, 8'h00);

                        // Clear HPD interrupts
//                        19: write_i2c(CHIP_ADDR, 8'h96, 8'hFF);

                        // Video input mode
//                        20: write_i2c(CHIP_ADDR, 8'h41, 8'h00);

// From ADV7511
/*
                         0: write_i2c(CHIP_ADDR, 8'h01, 8'h00); // Set N Value (6144)
                         1: write_i2c(CHIP_ADDR, 8'h02, 8'h18); // Set N Value(6144)
                         2: write_i2c(CHIP_ADDR, 8'h03, 8'h00); // Set N Value(6144)
                         3: write_i2c(CHIP_ADDR, 8'h15, 8'h00); // Input 444 (RGB or YCrCb) with Separate Syncs
                         4: write_i2c(CHIP_ADDR, 8'h16, 8'h61); // 44.1kHz fs, YPrPb 444
                         5: write_i2c(CHIP_ADDR, 8'h18, 8'h46); // CSC disabled
                         6: write_i2c(CHIP_ADDR, 8'h40, 8'h80); // General Control Packet Enable
                         7: write_i2c(CHIP_ADDR, 8'h41, 8'h10); // Power Down control
                         8: write_i2c(CHIP_ADDR, 8'h48, 8'h48); // Reverse bus, Data right justified
                         9: write_i2c(CHIP_ADDR, 8'h48, 8'hA8); // Set Dither_mode - 12-to-10 bit
                        10: write_i2c(CHIP_ADDR, 8'h4C, 8'h06); // 12 bit Output
                        11: write_i2c(CHIP_ADDR, 8'h55, 8'h00); // Set RGB444 in AVinfo Frame
                        12: write_i2c(CHIP_ADDR, 8'h55, 8'h08); // Set active format Aspect
                        13: write_i2c(CHIP_ADDR, 8'h96, 8'h20); // HPD Interrupt clear
                        14: write_i2c(CHIP_ADDR, 8'h98, 8'h03); // ADI required Write
                        15: write_i2c(CHIP_ADDR, 8'h98, 8'h02); // ADI required Write
                        16: write_i2c(CHIP_ADDR, 8'h9C, 8'h30); // ADI required Write
                        17: write_i2c(CHIP_ADDR, 8'h9D, 8'h61); // Set clock divide
                        18: write_i2c(CHIP_ADDR, 8'hA2, 8'hA4); // ADI required Write
                        19: write_i2c(CHIP_ADDR, 8'h43, 8'hA4); // ADI required Write
                        20: write_i2c(CHIP_ADDR, 8'hAF, 8'h16); // Set HDMI Mode
                        21: write_i2c(CHIP_ADDR, 8'hBA, 8'h60); // No clock delay
                        22: write_i2c(CHIP_ADDR, 8'hDE, 8'h9C); // ADI required write
                        23: write_i2c(CHIP_ADDR, 8'hE4, 8'h60); // ADI required Write
                        24: write_i2c(CHIP_ADDR, 8'hFA, 8'h7D); // Nbr of times to search for good phase
*/
                        default: state <= s_iter; //          Do nothing
                    endcase
                end

                s_wait: begin
                    write_en <= 1'b0;
                    read_en  <= 1'b0;
                    state    <= i2c_done ? s_iter : s_wait;
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
            reg_addr  <= t_reg_addr;
            data_in   <= t_data;
            write_en  <= 1'b1;
        end
    endtask

    task read_i2c;
        input [6:0] t_chip_addr;
        input [7:0] t_reg_addr;

        begin
            chip_addr <= t_chip_addr;
            reg_addr  <= t_reg_addr;
            read_en   <= 1'b1;
        end
    endtask
endmodule
