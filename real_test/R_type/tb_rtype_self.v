`timescale 1ns/1ps
module tb_rtype_self;
    reg clk = 0;
    reg rst = 1;
    cpu_top #(.MEMFILE("programs/rtype_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        #20 rst = 0;
        repeat (80) @(posedge clk);
        if (dut.rf.regs[3] !== 32'd13) begin $display("FAIL x3=%0d expected 13", dut.rf.regs[3]); $finish; end
        if (dut.rf.regs[4] !== 32'd7) begin $display("FAIL x4=%0d expected 7", dut.rf.regs[4]); $finish; end
        if (dut.rf.regs[5] !== 32'd11) begin $display("FAIL x5=%0d expected 11", dut.rf.regs[5]); $finish; end
        if (dut.rf.regs[6] !== 32'd2) begin $display("FAIL x6=%0d expected 2", dut.rf.regs[6]); $finish; end
        $display("PASS: R-TYPE");
        $finish;
    end
endmodule
