`timescale 1ns/1ps
module tb_loaduse_self;
    reg clk = 0;
    reg rst = 1;
    cpu_top #(.MEMFILE("programs/loaduse_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        #20 rst = 0;
        // run long enough for store, load, and dependent add to complete and write-back
        repeat (120) @(posedge clk);
        if (dut.rf.regs[1] !== 32'd8) begin $display("FAIL x1=%0d expected 8", dut.rf.regs[1]); $finish; end
        if (dut.rf.regs[3] !== 32'd16) begin $display("FAIL x3=%0d expected 16", dut.rf.regs[3]); $finish; end
        $display("PASS: LOAD-USE STALL");
        $finish;
    end
endmodule
