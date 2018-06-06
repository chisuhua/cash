module GCD(
  input wire clk,
  input wire reset,
  input wire io_in_valid,
  input wire[31:0] io_in_data,
  output wire io_in_ready,
  output wire io_out_valid,
  output wire[15:0] io_out_data
);
  reg[15:0] reg23; // gcd.cpp(16)
  reg[15:0] reg29; // gcd.cpp(16)
  reg reg34; // gcd.cpp(17)
  wire eq37;
  wire and39;
  wire sel43; // gcd.cpp(21)
  wire[15:0] sel44; // gcd.cpp(21)
  wire[15:0] sel45; // gcd.cpp(21)
  wire gt46;
  wire[15:0] sub48;
  wire[15:0] sel50; // gcd.cpp(28)
  wire[15:0] sel51; // gcd.cpp(27)
  wire[15:0] sel52; // gcd.cpp(28)
  wire and53;
  wire sel55; // gcd.cpp(36)
  wire eq58;
  wire and60;

  always @ (posedge clk) begin
    reg23 <= sel52;
  end
  always @ (posedge clk) begin
    reg29 <= sel51;
  end
  always @ (posedge clk) begin
    reg34 <= reset ? 1'h0 : sel55;
  end
  assign eq37 = 1'h0 == reg34;
  assign and39 = io_in_valid & eq37;
  assign sel43 = and39 ? 1'h1 : reg34;
  assign sel44 = and39 ? io_in_data[15:0] : reg23;
  assign sel45 = and39 ? io_in_data[31:16] : reg29;
  assign gt46 = reg23 > reg29;
  assign sub48 = reg29 - reg23;
  assign sel50 = gt46 ? reg23 : sub48;
  assign sel51 = reg34 ? sel50 : sel45;
  assign sel52 = and53 ? reg29 : sel44;
  assign and53 = reg34 & gt46;
  assign sel55 = and60 ? 1'h0 : sel43;
  assign eq58 = 16'h0 == reg29;
  assign and60 = eq58 & reg34;

  assign io_in_ready = eq37;
  assign io_out_valid = and60;
  assign io_out_data = reg23;

endmodule
