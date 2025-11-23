module tb_fetch;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));

    always #5 clk = ~clk;

    initial begin
        $display("===== BASIC FETCH TEST =====");

        // Load a trivial program into memory
        // program:
        //   addi x1, x0, 5
        //   addi x2, x0, 7
        //   add  x3, x1, x2
        //
        // Save this in test_fetch.mem before running

        #10 rst = 0;

        repeat (10) begin
            #10;
            $display("PC = %0d, Inst = %h", dut.pc_out, dut.IF_ID_out[31:0]);
        end

        $finish;
    end
endmodule
