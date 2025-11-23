`timescale 1ns/1ps
module tb_fetch_self;
    reg clk = 0;
    reg rst = 1;
    cpu_top #(.MEMFILE("programs/fetch_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        $display("=== FETCH SELF-CHECK ===");
        #20 rst = 0;
        repeat (60) @(posedge clk);
        if (dut.rf.regs[1] !== 32'd5) begin $display("FAIL x1=%0d expected 5", dut.rf.regs[1]); $finish; end
        if (dut.rf.regs[2] !== 32'd7) begin $display("FAIL x2=%0d expected 7", dut.rf.regs[2]); $finish; end
        if (dut.rf.regs[3] !== 32'd12) begin $display("FAIL x3=%0d expected 12", dut.rf.regs[3]); $finish; end
        if (dut.rf.regs[4] !== 32'd9) begin $display("FAIL x4=%0d expected 9", dut.rf.regs[4]); $finish; end
        $display("PASS: FETCH");
        $finish;
    end
endmodule
