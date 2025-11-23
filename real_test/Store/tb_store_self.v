`timescale 1ns/1ps
module tb_memory_self;
    reg clk=0;
    reg rst=1;
    cpu_top #(.MEMFILE("programs/memory_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        #20 rst = 0;
        repeat (120) @(posedge clk);
        if (dut.rf.regs[2] !== 32'd50) begin $display("FAIL x2=%0d expected 50", dut.rf.regs[2]); $finish; end
        $display("PASS: MEMORY LW/SW");
        $finish;
    end
endmodule
