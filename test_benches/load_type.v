module tb_load_use;
    reg clk = 0;
    reg rst = 1;

    cpu_top dut(.clk(clk), .rst(rst), .ledsel(0), .ssdSel(0), .ssdClk(clk));
    always #5 clk = ~clk;

    initial begin
        $display("===== LOAD-USE HAZARD TEST =====");

        // program:
        //   lw  x1, 0(x2)
        //   add x3, x1, x4   # must stall 1 cycle

        #10 rst = 0;

        repeat (25) begin
            #10;
            $display("PC=%0d IF_ID=%h ID_EX_rd=%0d MemRead=%b",
                dut.pc_out,
                dut.IF_ID_out[31:0],
                dut.ID_EX_Rd,
                dut.ID_EX_ctrl[7]);
        end

        $finish;
    end
endmodule
