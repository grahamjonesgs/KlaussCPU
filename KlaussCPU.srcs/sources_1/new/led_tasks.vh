// Set LED signal status
// On completion
// Increment PC 2
// Increment r_SM_msg

task t_led_value;
   input [31:0] i_state;
   begin
      o_led <= i_state[15:0];
      r_SM  <= OPCODE_REQUEST;
      r_PC  <= r_PC + 8;
   end
endtask

// Set LED signal status from register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_led_reg;
   begin
      o_led <= r_reg_port_b[15:0];
      r_SM  <= OPCODE_REQUEST;
      r_PC  <= r_PC + 4;
   end
endtask

// Set LED RGB1 signal status
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_led_rgb1_value;
   input [31:0] i_state;
   begin
      r_RGB_LED_1 <= i_state[11:0];
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 8;
   end
endtask

// Set LED RGB1 signal status from register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_led_rgb1_reg;
   begin
      r_RGB_LED_1 <= r_reg_port_b[11:0];
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 4;
   end
endtask

// Set LED RGB2 signal status
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_led_rgb2_value;
   input [31:0] i_state;
   begin
      r_RGB_LED_2 <= i_state[11:0];
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 8;
   end
endtask

// Set LED RGB2 signal status from register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_led_rgb2_reg;
   begin
      r_RGB_LED_2 <= r_reg_port_b[11:0];
      r_SM <= OPCODE_REQUEST;
      r_PC <= r_PC + 4;
   end
endtask

// Put switch status into register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_get_switch_reg;
   begin
      r_writeback_value <= {r_reg_port_b[31:16], i_switch};
      r_writeback_reg <= r_reg_2;
      r_SM <= WRITEBACK;
      r_PC <= r_PC + 4;
   end
endtask




