// Set LED signal status
// On completion
// Increment PC 2
// Increment r_SM_msg

task t_led_value;
   input [31:0] i_state;
   begin
      st.led <= i_state[15:0];
      st.SM  <= OPCODE_REQUEST;
      st.PC  <= st.PC + 8;
   end
endtask

// Set LED signal status from register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_led_reg;
   begin
      st.led <= r_reg_port_b[15:0];
      st.SM  <= OPCODE_REQUEST;
      st.PC  <= st.PC + 4;
   end
endtask

// Set LED RGB1 signal status
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_led_rgb1_value;
   input [31:0] i_state;
   begin
      st.RGB_LED_1 <= i_state[11:0];
      st.SM <= OPCODE_REQUEST;
      st.PC <= st.PC + 8;
   end
endtask

// Set LED RGB1 signal status from register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_led_rgb1_reg;
   begin
      st.RGB_LED_1 <= r_reg_port_b[11:0];
      st.SM <= OPCODE_REQUEST;
      st.PC <= st.PC + 4;
   end
endtask

// Set LED RGB2 signal status
// On completion
// Increment PC 2
// Increment r_SM_msg
task t_led_rgb2_value;
   input [31:0] i_state;
   begin
      st.RGB_LED_2 <= i_state[11:0];
      st.SM <= OPCODE_REQUEST;
      st.PC <= st.PC + 8;
   end
endtask

// Set LED RGB2 signal status from register
// On completion
// Increment PC 1
// Increment r_SM_msg
task t_led_rgb2_reg;
   begin
      st.RGB_LED_2 <= r_reg_port_b[11:0];
      st.SM <= OPCODE_REQUEST;
      st.PC <= st.PC + 4;
   end
endtask

// SWR - Read switch inputs into register.
// rd[15:0] = switch inputs (i_switch); rd[31:16] preserved from rs2[31:16];
// rd[63:32] = 0.  NOTE: not a clean zero-extension — the source register's
// upper-half bits [31:16] are carried through.
// 1-word, PC += 4.
task t_get_switch_reg;
   begin
      st.wb.value <= {r_reg_port_b[31:16], i_switch};
      st.wb.rd <= st.reg_2;
      st.SM <= WRITEBACK;
      st.PC <= st.PC + 4;
   end
endtask




