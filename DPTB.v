`timescale 1ns/1ps

module DataPathTB;
    reg Clk;
    wire [31:0] dataOut;

    DataPath DP(
        .CLK(Clk),
        .DS(dataOut)
    );

    always #5 Clk = ~Clk;

    initial begin
        Clk = 0;
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        #20
        $stop;
    end

endmodule
