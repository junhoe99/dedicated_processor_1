`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/09/17 14:20:11
// Design Name: 
// Module Name: dedicated_processor_counter
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module dedicated_processor_counter (
    input logic clk,
    input logic rst,
    output logic [7:0] out
);
    //logic w_ALt10;
    //logic w_AsrcSel;
    //logic w_ALoad;
    //logic w_OutBufSel;

    //DataPath DP (
    //    .clk(clk),
    //    .rst(rst),
    //    .AsrcSel(w_AsrcSel),
    //    .ALoad(w_ALoad),
    //    .OutBufSel(w_OutBufSel),
    //    .ALt10(w_ALt10),
    //    .out(out)
    //);

    //ControlUnit CU (
    //    .clk(clk),
    //    .rst(rst),
    //    .ALt10(w_ALt10),
    //    .AsrcSel(w_AsrcSel),
    //    .ALoad(w_ALoad),
    //    .OutBufSel(w_OutBufSel)
    //);

    logic ALt10;
    logic AsrcSel;
    logic ALoad;
    logic OutBufSel;

    DataPath DP (
        .*  // 자동 연결 (auto connection)
    );

    ControlUnit CU (
        .*  // 자동 연결 (auto connection)
    );
endmodule


module DataPath (
    // Inputs
    input  logic       clk,
    input  logic       rst,
    input  logic       AsrcSel,
    input  logic       ALoad,
    input  logic       OutBufSel,
    // Output
    output logic       ALt10,
    output logic [7:0] out
);
    logic [7:0] w_mux2Areg, w_AregOut;
    logic [7:0] w_Adder_sum;

    //Instantiation
    mux_2x1 MUX (
        .AsrcSel(AsrcSel),
        .a(8'b0),
        .b(w_Adder_sum),
        .mux2reg(w_mux2Areg)
    );

    Areg AREG (
        .clk  (clk),
        .rst  (rst),
        .ALoad(ALoad),
        .iAreg(w_mux2Areg),
        .oAreg(w_AregOut)
    );

    comparator COMPARATOR (
        .a(w_AregOut),
        .b(8'd10),
        .ALt10(ALt10)
    );

    Adder ADDER (
        .a  (w_AregOut),
        .b  (8'd1),
        .sum(w_Adder_sum)
    );

    outBuf OUTBUF (
        .Areg_data(w_AregOut),
        .OutBufSel(OutBufSel),
        .out(out)
    );

endmodule


module mux_2x1 (
    input logic AsrcSel,
    input logic [7:0] a,
    input logic [7:0] b,
    output logic [7:0] mux2reg  // A reg로 입력되는 값이기 때문에
);
    always_comb begin
        mux2reg = 8'b0;
        case (AsrcSel)
            1'b0: mux2reg = a;
            1'b1: mux2reg = b;
        endcase
    end
    // SV에서 always_comb를 쓰면 wire/reg 구분 없이 쓸 수 있다.
endmodule


module Areg (
    input logic clk,
    input logic rst,
    input logic ALoad,
    input logic [7:0] iAreg,
    output logic [7:0] oAreg
);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) oAreg <= 8'b0;
        else if (ALoad) oAreg <= iAreg;
    end
endmodule

module comparator (
    input logic [7:0] a,
    input logic [7:0] b,
    output logic ALt10
);
    assign ALt10 = (a < b);
endmodule

module Adder (
    input  logic [7:0] a,
    input  logic [7:0] b,
    output logic [7:0] sum
);
    assign sum = a + b;
endmodule

module outBuf (
    input logic [7:0] Areg_data,
    input logic OutBufSel,
    output logic [7:0] out
);
    assign out = OutBufSel ? Areg_data : 8'bz;  // Tri-state buffer 
endmodule

module ControlUnit (
    input logic clk,
    input logic rst,
    input logic ALt10,
    output logic AsrcSel,
    output logic ALoad,
    output logic OutBufSel
);

    typedef enum bit [2:0] {
        S0,
        S1,
        S2,
        S3,
        S4
    } state_e;

    state_e c_state, n_state;

    //State Register
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            c_state <= S0;
        end else begin
            c_state <= n_state;
        end
    end

    //Next State Logic
    always_comb begin
        n_state = c_state;
        case (c_state)
            S0: begin  // A = 0;
                n_state = S1;
            end

            S1: begin  // A < 10;
                if (ALt10) begin  // A < 10이면, S2로
                    n_state = S2;
                end else begin
                    n_state = S4;
                end
            end

            S2: begin  // out = A;
                n_state = S3;
            end

            S3: begin  // A = A + 1;
                n_state = S1;  // 다시 S1으로
            end

            S4: begin  // halt state
                n_state = S4;  // Stay in S4
            end
            
            default: begin
                n_state = S0;
            end
        endcase
    end

    //Output Logic (Moore Machine - outputs depend only on current state)
    always_comb begin
        // Default values
        AsrcSel = 1'b0;
        ALoad = 1'b0;
        OutBufSel = 1'b0;
        
        case (c_state)
            S0: begin  // A = 0;
                AsrcSel   = 1'b0;  // Select 0 for initial load
                ALoad     = 1'b1;  // Load A register with 0
                OutBufSel = 1'b0;  // Disable output buffer
            end

            S1: begin  // A < 10;
                AsrcSel   = 1'b0;  // Don't care
                ALoad     = 1'b0;  // Don't load A register
                OutBufSel = 1'b0;  // Disable output buffer
            end

            S2: begin  // out = A;
                AsrcSel   = 1'b0;  // Don't care
                ALoad     = 1'b0;  // Don't load A register
                OutBufSel = 1'b1;  // Enable output buffer 
            end

            S3: begin  // A = A + 1;
                AsrcSel   = 1'b1;  // Select Adder output
                ALoad     = 1'b1;  // Load A register with Adder output
                OutBufSel = 1'b0;  // Disable output buffer
            end

            S4: begin  // halt state
                AsrcSel   = 1'b0;  // Don't care
                ALoad     = 1'b0;  // Don't load A register
                OutBufSel = 1'b0;  // Disable output buffer
            end
        endcase
    end

endmodule
