`timescale 1ns/10ps
module IOTDF( clk, rst, in_en, iot_in, fn_sel, busy, valid, iot_out);
input          clk;
input          rst;
input          in_en;
input  [7:0]   iot_in;
input  [3:0]   fn_sel;
output         busy;
output         valid;
output [127:0] iot_out;

localparam S_IDLE       = 0;
localparam S_LOAD       = 1;

localparam F1 = 4'b0001;
localparam F2 = 4'b0010;
localparam F3 = 4'b0011;
localparam F4 = 4'b0100;
localparam F5 = 4'b0101;
localparam F6 = 4'b0110;
localparam F7 = 4'b0111;
localparam F8 = 4'b1000;
localparam F9 = 4'b1001;

parameter EXTRACT_LOW   = 128'h 6FFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
parameter EXTRACT_HIGH  = 128'h AFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
parameter EXCLUDE_LOW   = 128'h 7FFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
parameter EXCLUDE_HIGH  = 128'h BFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;

reg [130:0] sum_r, sum_w;
reg [127:0] Ans_w[0:1], Ans_r [0:1];
reg [127:0] sensor_data_w;

reg [7:0] data_r [0:15], data_w[0:15];
reg state_r, state_w;
reg [4:0] counter_r, counter_w;
reg round_counter_r, round_counter_w;
reg [4:0] P_iterator_r, P_iterator_w;
reg out_iterator_r, out_iterator_w;
reg start_r, start_w;
reg valid_r, valid_w;
reg busy_w;
reg [127:0] out_r, out_w;
integer i;

assign valid = valid_r;
assign iot_out = out_r;
assign busy = busy_w;

// FSM State transition
always @(*) begin
    state_w = state_r;
    start_w = start_r;
    if(rst) begin
        start_w = 1;
    end
    case (state_r)
        S_IDLE: begin
            if(start_r) state_w = S_LOAD;
            else state_w = S_IDLE;
        end
        S_LOAD: begin
            state_w = S_LOAD;
        end
    endcase
end



always @(*) begin
    counter_w = counter_r;
    P_iterator_w = P_iterator_r;
    round_counter_w = round_counter_r;
    valid_w = 0;
    busy_w = 0;
    sum_w = sum_r;
    out_iterator_w = out_iterator_r;
    out_w = out_r;
    for(i=0; i<8; i=i+1) begin
        Ans_w[i] = Ans_r[i];
        
    end
    for(i=0; i<16; i=i+1) begin
        data_w[i] = data_r[i];
    end
    
    sensor_data_w = {data_r[15], data_r[14], data_r[13], data_r[12], data_r[11], data_r[10], data_r[9], data_r[8], 
                    data_r[7], data_r[6], data_r[5], data_r[4], data_r[3], data_r[2], data_r[1], data_r[0]};

    case (state_r)
        S_LOAD: begin
            if(P_iterator_r < 8) begin
                counter_w = counter_r + 1;
                if(counter_r == 15) busy_w = 1;
                
                if(counter_r < 16) begin
                    data_w[counter_r] = iot_in;
                end
                else begin
                    counter_w = 0;
                    P_iterator_w = P_iterator_r + 1;
                    if(P_iterator_w == 8) busy_w = 1;
                    for(i=0; i<16; i=i+1) begin
                        data_w[i] = 0;
                    end
                    case (fn_sel)
                        F1: begin
                            // Max
                            if(P_iterator_r == 0) begin
                                Ans_w[0] = sensor_data_w;
                            end
                            else if(sensor_data_w > Ans_r[0]) begin
                                Ans_w[0] = sensor_data_w;
                            end
                        end
                        F2: begin
                            // Min
                            if(P_iterator_r == 0) begin
                                Ans_w[0] = sensor_data_w;
                            end
                            else if(sensor_data_w < Ans_r[0]) begin
                                Ans_w[0] = sensor_data_w;
                            end
                        end
                        F3: begin
                            // Top2Max
                            if(P_iterator_r == 0) begin
                                Ans_w[0] = sensor_data_w;
                            end
                            else if(P_iterator_r == 1)begin
                                if(sensor_data_w > Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                    Ans_w[1] = Ans_r[0];
                                end
                                else begin
                                    Ans_w[1] = sensor_data_w;
                                end
                            end
                            else begin
                                if(sensor_data_w > Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                    Ans_w[1] = Ans_r[0];
                                end
                                else if(sensor_data_w > Ans_r[1]) begin
                                    Ans_w[1] = sensor_data_w;
                                end
                            end
                        end
                        F4: begin
                            // Last2Min
                            if(P_iterator_r == 0) begin
                                Ans_w[0] = sensor_data_w;
                            end
                            else if(P_iterator_r == 1) begin
                                if(sensor_data_w < Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                    Ans_w[1] = Ans_r[0];
                                end
                                else begin
                                    Ans_w[1] = sensor_data_w;
                                end
                            end
                            else begin
                                if(sensor_data_w < Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                    Ans_w[1] = Ans_r[0];
                                end
                                else if(sensor_data_w < Ans_r[1]) begin
                                    Ans_w[1] = sensor_data_w;
                                end
                            end
                        end
                        F5: begin
                            // AVG
                            sum_w = sum_r + sensor_data_w;
                            if(P_iterator_r == 7) begin
                                Ans_w[0] = sum_w >> 3;
                            end
                            
                        end
                        F6: begin
                            // Extract
                            if(EXTRACT_LOW < sensor_data_w && sensor_data_w < EXTRACT_HIGH)begin
                                valid_w = 1;
                                out_w = sensor_data_w;
                            end
                        end
                        F7: begin
                            // Exclude
                            if(sensor_data_w < EXCLUDE_LOW || EXCLUDE_HIGH < sensor_data_w) begin
                                valid_w = 1;
                                out_w = sensor_data_w;
                            end
                        end
                        F8: begin
                            

                            // peak_max
                            if(round_counter_r == 0) begin
                                // Round Max
                                if(P_iterator_r == 0) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                else if(sensor_data_w > Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                sum_w[127:0] = Ans_w[0];
                            end
                            else begin
                                // Round Max
                                if(P_iterator_r == 0) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                else if(sensor_data_w > Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                if(Ans_w[0] > sum_r[127:0]) begin
                                    sum_w[127:0] = Ans_w[0];
                                end
                            end
                            
                        end
                        F9: begin
                            

                            // peak min
                            if(round_counter_r == 0) begin
                                if(P_iterator_r == 0) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                else if(sensor_data_w < Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                sum_w[127:0] = Ans_w[0];
                            end
                            else begin
                                if(P_iterator_r == 0) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                else if(sensor_data_w < Ans_r[0]) begin
                                    Ans_w[0] = sensor_data_w;
                                end
                                if(Ans_w[0] < sum_r[127:0]) begin
                                    sum_w[127:0] = Ans_w[0];
                                end
                            end
                        end
                        // default: 
                    endcase
                end
            end
            else begin
                P_iterator_w = 0;
                
                // Output
                case (fn_sel)
                    F1: begin
                        valid_w = 1;
                        out_w = Ans_r[0];
                        round_counter_w = 1;
                    end
                    F2: begin
                        valid_w = 1;
                        out_w = Ans_r[0];
                        round_counter_w = 1;
                    end
                    F3: begin
                        
                        valid_w = 1;
                        if(out_iterator_r == 0) begin
                            out_iterator_w = 1;
                            P_iterator_w = 8;
                            busy_w = 1;
                            out_w = Ans_r[0];
                        end 
                        else begin
                            out_iterator_w = 0;
                            out_w = Ans_r[1];
                            round_counter_w = 1;
                        end
                    end
                    F4: begin
                        
                        valid_w = 1;
                        if(out_iterator_r == 0) begin
                            out_iterator_w = 1;
                            P_iterator_w = 8;
                            busy_w = 1;
                            out_w = Ans_r[0];
                        end 
                        else begin
                            out_iterator_w = 0;
                            out_w = Ans_r[1];
                            round_counter_w = 1;
                        end
                    end
                    F5: begin
                        sum_w = 0;
                        valid_w = 1;
                        out_w = Ans_r[0];
                        round_counter_w = 1;
                    end
                    F8: begin
                        round_counter_w = 1;
                        if(round_counter_r == 0) begin
                            valid_w = 1;
                            out_w = Ans_r[0];
                        end
                        else if(sum_r[127:0] == Ans_r[0]) begin
                            valid_w = 1;
                            out_w = Ans_r[0];
                        end
                    end
                    F9: begin
                        round_counter_w = 1;
                        if(round_counter_r == 0) begin
                            valid_w = 1;
                            out_w = Ans_r[0];
                        end
                        else if(sum_r[127:0] == Ans_r[0]) begin
                            valid_w = 1;
                            out_w = Ans_r[0];
                        end
                    end
                    // default: 
                endcase
            end
            
        end
        // default: 
    endcase
end



// Sequential CKTS
always @(posedge clk or posedge rst) begin
    if(rst) begin
        state_r <= 0;
        start_r <= start_w;
        counter_r <= 0;
        valid_r <= 0;
        out_r <= 0;
        P_iterator_r <= 0;
        round_counter_r <= 0;
        sum_r <= 0;
        out_iterator_r <= 0;
        for(i=0; i<2; i=i+1) begin
            Ans_r[i] <= 0;
            
        end
        for(i=0; i<16; i=i+1) begin
            data_r[i] <= 0;
        end
    end
    else begin
        state_r <= state_w;
        start_r <= start_w;
        counter_r <= counter_w;
        valid_r <= valid_w;
        out_r <= out_w;
        P_iterator_r <= P_iterator_w;
        round_counter_r <= round_counter_w;
        sum_r <= sum_w;
        out_iterator_r <= out_iterator_w;
        for(i=0; i<2; i=i+1) begin
            Ans_r[i] <= Ans_w[i];
        end
        for(i=0; i<16; i=i+1) begin
            data_r[i] <= data_w[i];
        end
    end
end

endmodule
