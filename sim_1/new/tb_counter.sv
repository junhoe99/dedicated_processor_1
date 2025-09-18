`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/09/17 15:35:53
// Design Name: 
// Module Name: tb_counter_verilog
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


module tb_counter();

    logic clk = 0, rst = 1;
    logic [7:0] out = 0;


    dedicated_processor_counter dut (
        .clk(clk),
        .rst(rst),
        .out(out)
    );  

    always #5 clk = ~clk; // 10ns period (100MHz)

    initial begin
        #0;
        @(posedge clk);
        @(posedge clk);
        @(negedge clk);
        rst = 0;
        #1000;
        $stop;
    end

endmodule
