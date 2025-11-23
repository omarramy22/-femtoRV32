`timescale 1ns/1ps
module tb_forwarding_self;
    reg clk = 0;
    reg rst = 1;
    cpu_top #(.MEMFILE("programs/forwarding_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        #20 rst = 0;
        repeat (80) @(posedge clk);
        if (dut.rf.regs[5] !== 32'd10) begin $display("FAIL x5=%0d expected 10", dut.rf.regs[5]); $finish; end
        if (dut.rf.regs[6] !== 32'd6)  begin $display("FAIL x6=%0d expected 6", dut.rf.regs[6]);  $finish; end
        $display("PASS: FORWARDING");
        $finish;
    end
endmodule
