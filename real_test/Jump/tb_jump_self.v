`timescale 1ns/1ps
module tb_jump_self;
    reg clk=0;
    reg rst=1;
    cpu_top #(.MEMFILE("programs/jump_test.mem")) dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk), .leds(), .Anode(), .LED_out());
    always #5 clk = ~clk;
    initial begin
        #20 rst = 0;
        repeat (140) @(posedge clk);
        if (dut.rf.regs[1] === 32'd0) begin $display("WARN: x1 is zero -- verify link semantics"); end
        if (dut.rf.regs[2] !== 32'd7) begin $display("FAIL x2=%0d expected 7", dut.rf.regs[2]); $finish; end
        $display("PASS: JAL/JALR (basic checks)");
        $finish;
    end
endmodule
