`timescale 1ns/1ps
module tb_itype_self;
    reg clk = 0;
    reg rst = 1;
    cpu_top #(.MEMFILE("programs/itype_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        #20 rst = 0;
        repeat (80) @(posedge clk);
        if (dut.rf.regs[2] !== 32'd15) begin $display("FAIL x2=%0d expected 15", dut.rf.regs[2]); $finish; end
        if (dut.rf.regs[3] !== 32'd11) begin $display("FAIL x3=%0d expected 11", dut.rf.regs[3]); $finish; end
        if (dut.rf.regs[4] !== 32'd8) begin $display("FAIL x4=%0d expected 8", dut.rf.regs[4]); $finish; end
        if (dut.rf.regs[5] !== 32'd1) begin $display("FAIL x5=%0d expected 1", dut.rf.regs[5]); $finish; end
        $display("PASS: I-TYPE");
        $finish;
    end
endmodule
