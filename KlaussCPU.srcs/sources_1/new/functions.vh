function [3:0] return_hex_from_ascii;
   input [7:0] ascii;
   begin
      case (ascii)
         8'h30:   return_hex_from_ascii = 4'h0;
         8'h31:   return_hex_from_ascii = 4'h1;
         8'h32:   return_hex_from_ascii = 4'h2;
         8'h33:   return_hex_from_ascii = 4'h3;
         8'h34:   return_hex_from_ascii = 4'h4;
         8'h35:   return_hex_from_ascii = 4'h5;
         8'h36:   return_hex_from_ascii = 4'h6;
         8'h37:   return_hex_from_ascii = 4'h7;
         8'h38:   return_hex_from_ascii = 4'h8;
         8'h39:   return_hex_from_ascii = 4'h9;
         8'h41:   return_hex_from_ascii = 4'hA;
         8'h42:   return_hex_from_ascii = 4'hB;
         8'h43:   return_hex_from_ascii = 4'hC;
         8'h44:   return_hex_from_ascii = 4'hD;
         8'h45:   return_hex_from_ascii = 4'hE;
         8'h46:   return_hex_from_ascii = 4'hF;
         default: return_hex_from_ascii = 4'h0;
      endcase
   end
endfunction

function [7:0] return_ascii_from_hex;
   input [3:0] hex;
   begin
      case (hex)
         4'h0: return_ascii_from_hex = 8'h30;
         4'h1: return_ascii_from_hex = 8'h31;
         4'h2: return_ascii_from_hex = 8'h32;
         4'h3: return_ascii_from_hex = 8'h33;
         4'h4: return_ascii_from_hex = 8'h34;
         4'h5: return_ascii_from_hex = 8'h35;
         4'h6: return_ascii_from_hex = 8'h36;
         4'h7: return_ascii_from_hex = 8'h37;
         4'h8: return_ascii_from_hex = 8'h38;
         4'h9: return_ascii_from_hex = 8'h39;
         4'hA: return_ascii_from_hex = 8'h41;
         4'hB: return_ascii_from_hex = 8'h42;
         4'hC: return_ascii_from_hex = 8'h43;
         4'hD: return_ascii_from_hex = 8'h44;
         4'hE: return_ascii_from_hex = 8'h45;
         4'hF: return_ascii_from_hex = 8'h46;
         default: return_ascii_from_hex = 8'h3F;
      endcase
   end
endfunction
