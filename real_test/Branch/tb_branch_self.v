`timescale 1ns/1ps
module tb_branch_self;
    reg clk = 0;
    reg rst = 1;
    cpu_top #(.MEMFILE("programs/branch_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        #20 rst = 0;
        repeat (100) @(posedge clk);
        if (dut.rf.regs[3] !== 32'd7) begin $display("FAIL x3=%0d expected 7", dut.rf.regs[3]); $finish; end
        $display("PASS: BRANCH");
        $finish;
    end
endmodule
