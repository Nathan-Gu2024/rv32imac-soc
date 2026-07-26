 `timescale 1ns/1ps

module tcm #( 
    parameter ADDR_WIDTH = 32, 
    parameter TCM_BASE = 32'h4000_0000, 
    parameter TCM_BYTES = 65536, 
    parameter INIT_FILE = ""
) (
    input wire clk, rst,
    // Port A, read, instructions
    input wire i_req, 
    input wire [ADDR_WIDTH - 1:0] i_addr, 
    output reg [31:0] i_rdata, 
    output reg i_ready, 

    // Port B, read/write, data
    input wire d_req, d_we, 
    input wire [ADDR_WIDTH - 1 : 0] d_addr, 
    input wire [31:0] d_wdata,
    input wire [3:0] d_wmask, 
    output reg [31:0] d_rdata, 
    output reg d_ready
); 
    localparam TCM_WORDS = TCM_BYTES / 4;
    localparam INDEX_BITS = $clog2(TCM_WORDS);

    // reg [31:0] mem [0 : TCM_WORDS - 1];
    (* ram_style = "block" *) reg [31:0] mem [0 : TCM_WORDS - 1];

    wire [ADDR_WIDTH - 1 : 0] i_offset_bytes = i_addr - TCM_BASE; 
    wire [ADDR_WIDTH - 1 : 0] d_offset_bytes = d_addr - TCM_BASE; 

    // wire [$clog2(TCM_WORDS) - 1 : 0] i_index = i_offset_bytes[INDEX_BITS + 1 : 2];
    // wire [$clog2(TCM_WORDS) - 1 : 0] d_index = d_offset_bytes[INDEX_BITS + 1 : 2];
    
    initial begin
        if (INIT_FILE != "") begin
            $display("Loading TCM file %s", INIT_FILE);
            $readmemh(INIT_FILE, mem);
        end
    end
    
    
    initial begin
        // Tiny CPU bring-up program at 0x4000_0000
        // lui  x1, 0x2        -> x1 = 0x00002000
        // addi x2, x0, 1      -> x2 = 1
        // sw   x2, 0(x1)      -> LED MMIO write
        // jal  x0, 0          -> loop forever
    
//        mem[0] = 32'h000020b7;
//        mem[1] = 32'h00100113;
//        mem[2] = 32'h0020a023;
//        mem[3] = 32'h0000006f;
        //li   x1, 0x40000100     # TCM test address
        //li   x2, 0x5            # test value
        //sw   x2, 0(x1)
        //lw   x3, 0(x1)
        //bne  x2, x3, fail
        
        //li   x4, 0x2
        //li   x5, 0x2000
        //sw   x4, 0(x5) # LEDs = pass
        //loop:
        //jal  x0, loop
        
        //fail:
        //li   x4, 0x7
        //li   x5, 0x2000
        //sw   x4, 0(x5) # LEDs = fail
        
        //fail_loop: 
        //jal  x0, fail 
        
         
    // TCM load/store test at 0x4000_0000
    // Expected:
    // led[2:0] = 010 pass
    // led[2:0] = 111 fail
        mem[0]  = 32'h400000B7; // lui  x1, 0x40000
        mem[1]  = 32'h10008093; // addi x1, x1, 0x100  ; x1 = 0x40000100
        mem[2]  = 32'h00500113; // addi x2, x0, 5
        mem[3]  = 32'h0020A023; // sw   x2, 0(x1)
        mem[4]  = 32'h0000A183; // lw   x3, 0(x1)
    
        mem[5]  = 32'h00000013; // nop
        mem[6]  = 32'h00000013; // nop
    
        mem[7]  = 32'h00311C63; // bne  x2, x3, fail
    
        mem[8]  = 32'h00200213; // addi x4, x0, 2
        mem[9]  = 32'h000022B7; // lui  x5, 0x2        ; x5 = 0x2000
        mem[10] = 32'h00028293; // addi x5, x5, 0
        mem[11] = 32'h0042A023; // sw   x4, 0(x5)      ; LED = 010 pass
        mem[12] = 32'h0000006F; // loop forever
    
        mem[13] = 32'h00700213; // addi x4, x0, 7
        mem[14] = 32'h000022B7; // lui  x5, 0x2
        mem[15] = 32'h00028293; // addi x5, x5, 0
        mem[16] = 32'h0042A023; // sw   x4, 0(x5)      ; LED = 111 fail
        mem[17] = 32'h0000006F; // loop forever
        
        // Store, currently 111, need 101
//        mem[0] = 32'h400000B7; // lui  x1, 0x40000
//        mem[1] = 32'h10008093; // addi x1, x1, 0x100  ; x1 = 0x40000100
//        mem[2] = 32'h00500113; // addi x2, x0, 5
//        mem[3] = 32'h0020A023; // sw   x2, 0(x1)      ; store to TCM
    
//        mem[4] = 32'h00300213; // addi x4, x0, 3      ; LED status = 011
//        mem[5] = 32'h000022B7; // lui  x5, 0x2        ; x5 = 0x2000
//        mem[6] = 32'h00028293; // addi x5, x5, 0
//        mem[7] = 32'h0042A023; // sw   x4, 0(x5)      ; write LEDs
//        mem[8] = 32'h0000006F; // loop forever

//            li   x1, 0x40000100
//            li   x2, 5
//            sw   x2, 0(x1)
//            lw   x4, 0(x1)
//            nop
//            nop
//            li   x5, 0x2000
//            sw   x4, 0(x5)
//            loop:
//            jal x0, loop
// Need 101, have 111
//            mem[0] = 32'h400000B7;
//            mem[1] = 32'h10008093;
//            mem[2] = 32'h00500113;
//            mem[3] = 32'h0020A023;
//            mem[4] = 32'h0000A203;
//            mem[5] = 32'h00000013; 
//            mem[6] = 32'h00000013; 
//            mem[7] = 32'h000022B7; 
//            mem[8] = 32'h00028293; 
//            mem[9] = 32'h0042A023;
//            mem[10] = 32'h0000006F;

    end

    wire [ADDR_WIDTH-1:0] i_offset = i_addr - TCM_BASE;
    wire [ADDR_WIDTH-1:0] d_offset = d_addr - TCM_BASE;

    wire [INDEX_BITS-1:0] i_index = i_offset[INDEX_BITS+1:2];
    wire [INDEX_BITS-1:0] d_index = d_offset[INDEX_BITS+1:2];

    wire i_in_range = (i_addr >= TCM_BASE) && (i_addr < (TCM_BASE + TCM_BYTES));
    wire d_in_range = (d_addr >= TCM_BASE) && (d_addr < (TCM_BASE + TCM_BYTES));


    reg [31:0] d_merged_word;
    always @(*) begin
        d_merged_word = mem[d_index]; 
        if (d_wmask[0]) d_merged_word[7:0] = d_wdata[7:0];
        if (d_wmask[1]) d_merged_word[15:8] = d_wdata[15:8];
        if (d_wmask[2]) d_merged_word[23:16] = d_wdata[23:16];
        if (d_wmask[3]) d_merged_word[31:24] = d_wdata[31:24];
    end 

    // Sync 1 cycle TCM; if inst and data ports touch same word 
    // -> return newly merged word for write-first
    always @(posedge clk) begin
        if (rst) begin
            i_rdata <= 32'b0;
            d_rdata <= 32'b0;
            i_ready <= 1'b0;
            d_ready <= 1'b0;
        end else begin
            i_ready <= 1'b0;
            d_ready <= 1'b0;

            if (d_req && d_in_range && d_we) begin 
                mem[d_index] <= d_merged_word;
            end 

            if (i_req && i_in_range) begin
                if (d_req && d_in_range && d_we && (i_index == d_index)) begin
                    i_rdata <= d_merged_word; 
                end else begin
                    i_rdata <= mem[i_index]; 
                end 
                i_ready <= 1'b1;
            end 

            if (d_req && d_in_range) begin
                d_rdata <= d_we ? d_merged_word : mem[d_index];
                d_ready <= 1'b1;
            end 
        end 
    end 

    // sim
//    `ifndef SYNTHESIS
//    integer i;
//    initial begin
//        for (i = 0; i < TCM_WORDS; i = i + 1) begin
//            mem[i] = 32'h0000_0000;
//        end
//    end
//    `endif
    
endmodule