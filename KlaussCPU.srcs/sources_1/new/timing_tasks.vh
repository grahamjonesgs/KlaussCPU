// Delay execution input value *2^13 ticks
// On completion
// Increment PC by 2
// Increment r_SM_msg

task t_delay;
    input [31:0] i_timeout_fraction;
    begin
        if(r_timing_start==0) // first cycle of timing
        begin
            r_timeout_max <= i_timeout_fraction << 13;
            r_timeout_counter <= 0;
            r_timing_start <= 1;
        end // if first loop
        else
        begin
            if (r_timeout_counter >= r_timeout_max) begin
                r_timeout_counter <= 0;
                r_timing_start <= 0;
                r_SM <= OPCODE_REQUEST;
                r_PC <= r_PC + 8;
            end  // if(r_timeout_counter>=DELAY_TIME)
            else
            begin
                r_timeout_counter <= r_timeout_counter + 1;
                r_SM <= OPCODE_EXECUTE;  // redo loop on same opcode
            end  // else if(r_timeout_counter>=DELAY_TIME)
        end  // if subsequent loop
    end
endtask

// Will delay execution input value *2^13 ticks from reg value
// On completion
// Increment PC by 1
// Increment r_SM_msg
task t_delay_reg;
    reg [31:0] r_timeout_fraction;
    reg [ 3:0] reg_1;
    begin

        if(r_timing_start==0) // first cycle of timing
        begin
            r_timeout_fraction = r_reg_port_b;
            r_timeout_max <= r_timeout_fraction << 13;
            r_timeout_counter <= 0;
            r_timing_start <= 1;
        end // if first loop
        else
        begin
            if (r_timeout_counter >= r_timeout_max) begin
                r_timeout_counter <= 0;
                r_timing_start <= 0;
                r_SM <= OPCODE_REQUEST;
                r_PC <= r_PC + 4;
            end  // if(r_timeout_counter>=DELAY_TIME)
            else
            begin
                r_timeout_counter <= r_timeout_counter + 1;
                r_SM <= OPCODE_EXECUTE;  // redo loop on same opcode
            end  // else if(r_timeout_counter>=DELAY_TIME)
        end  // if subsequent loop
    end
endtask
