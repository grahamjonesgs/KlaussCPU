`timescale 1ns / 1ps
module RGB_LED (
    input             i_sysclk,     // 100 Mhz clock source on Basys 3 FPGA
    input      [11:0] LED1,         // Number to display
    input      [11:0] LED2,         // Number to display
    output reg [ 2:0] o_LED_RGB_1,
    output reg [ 2:0] o_LED_RGB_2
);


   reg [18:0] r_counter;

   // Max 25% duty cycle to dim LED's
   always @(posedge i_sysclk) begin
      r_counter <= r_counter + 1;
      o_LED_RGB_1[2] = r_counter[18:15] < LED1[3:0] ? 1'b1 : 1'b0;  // b
      o_LED_RGB_1[1] = r_counter[18:15] < LED1[7:4] ? 1'b1 : 1'b0;  // g
      o_LED_RGB_1[0] = r_counter[18:15] < LED1[11:8] ? 1'b1 : 1'b0;  // r
      o_LED_RGB_2[2] = r_counter[18:15] < LED2[3:0] ? 1'b1 : 1'b0;
      o_LED_RGB_2[1] = r_counter[18:15] < LED2[7:4] ? 1'b1 : 1'b0;
      o_LED_RGB_2[0] = r_counter[18:15] < LED2[11:8] ? 1'b1 : 1'b0;
   end




endmodule
