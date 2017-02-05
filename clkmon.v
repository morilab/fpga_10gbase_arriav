module clkmon (
    input         CLK,       // [CLK]       clock(r1_*)
    input         RESET,     // [ASYNC]     reset
    input         MON_CLK,   // [MON_CLK]   clock(r2_*)
    output [15:0] MON_FREQ   // [CLK]       data
);

    parameter P_CLK_FREQ = 100*1000*1000; // CLK=100MHz
    parameter P_MON_UNIT =   1*1000*1000; // UNIT=1MHz
    
    // ----------------------------------
    // Multi clock domain
    // ----------------------------------
    // (1) CLK : r1_*
    reg [25:0]  r1_counter;
    reg         r1_latch;
    reg [15:0]  r1_mon_freq;
    
    // (2) MON_CLK : r2_*
    reg [19:0]  r2_counter_mega;
    reg [15:0]  r2_counter;
    reg         r2_latch;
    reg [15:0]  r2_mon_freq;
    
    // async buffer
    reg [2:0]   r1r2_latch;
    reg [15:0]  r2r1_mon_freq_d0;
    reg [15:0]  r2r1_mon_freq_d1;
    reg [15:0]  r2r1_mon_freq_d2;
    
    // ----------------------------------
    // (1)CLK domain
    // ----------------------------------
    always @(posedge CLK or posedge RESET) begin
        if(RESET)begin
            r1_counter <= P_CLK_FREQ-1'b1;
            r1_latch   <= 1'b0;
        end else begin
            if (r1_counter==26'd0)begin
                r1_counter <= P_CLK_FREQ-1'b1;
                r1_latch   <= ~r1_latch;
            end else begin
                r1_counter <= r1_counter-1'b1;
                r1_latch   <= r1_latch;
            end
        end
    end
    
    always @(posedge CLK or posedge RESET) begin
        if(RESET)begin
            r1_mon_freq <= 16'hFFFF;
        end else begin
            if(r2r1_mon_freq_d1==r2r1_mon_freq_d2)begin
                r1_mon_freq <= r2r1_mon_freq_d2;
            end else begin
                r1_mon_freq <= r1_mon_freq;
            end
        end
    end
    assign MON_FREQ = r1_mon_freq;
    
    // ----------------------------------
    // async domain
    // ----------------------------------
    always @(posedge MON_CLK or posedge RESET) begin
        if(RESET)begin
            r1r2_latch <= 3'd0;
        end else begin
            r1r2_latch <= {r1r2_latch[1:0],r1_latch};
        end
    end

    always @(posedge CLK or posedge RESET) begin
        if(RESET)begin
            r2r1_mon_freq_d0 <= 16'hFFFF;
            r2r1_mon_freq_d1 <= 16'hFFFF;
            r2r1_mon_freq_d2 <= 16'hFFFF;
        end else begin
            r2r1_mon_freq_d0 <= r2_mon_freq;
            r2r1_mon_freq_d1 <= r2r1_mon_freq_d0;
            r2r1_mon_freq_d2 <= r2r1_mon_freq_d1;
        end
    end
    
    // ----------------------------------
    // (2)MON_CLK domain
    // ----------------------------------
    always @(posedge MON_CLK or posedge RESET) begin
        if(RESET)begin
            r2_latch <= 1'b0;
        end else begin
            if(r1r2_latch[1]!=r1r2_latch[2])begin
                r2_latch <= 1'b1;
            end else begin
                r2_latch <= 1'b0;
            end
        end
    end
    
    always @(posedge MON_CLK or posedge RESET) begin
        if(RESET)begin
            r2_counter_mega <= P_MON_UNIT-1;
            r2_counter      <= 16'd0;
            r2_mon_freq     <= 16'hFFFF;
        end else begin
            if(r2_latch)begin
                r2_counter_mega <= P_MON_UNIT-1;
                r2_counter      <= 16'd0;
                r2_mon_freq     <= r2_counter;
                #0 $display("%t r2_mon_freq = %d",$time,r2_mon_freq);
            end else
            if(r2_counter_mega==20'd0)begin
                r2_counter_mega <= P_MON_UNIT-1;
                r2_counter      <= r2_counter+1'b1;
                r2_mon_freq     <= r2_mon_freq;
            end else begin
                r2_counter_mega <= r2_counter_mega-1'b1;
                r2_counter      <= r2_counter;
                r2_mon_freq     <= r2_mon_freq;
            end
        end
   end

endmodule
