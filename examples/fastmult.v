`include "cash.v"

module FastMult(
    input wire[3:0] io_lhs,
    input wire[3:0] io_rhs,
    output wire[7:0] io_out
);
    wire[3:0] io_lhs2; // #1 fastmult.cpp(7)
    wire[3:0] io_rhs5; // #2 fastmult.cpp(7)
    wire[7:0] io_out8; // #3 fastmult.cpp(7)
    reg[7:0] mem10[0:255];
    wire[3:0] proxy12; // #4 /home/blaise/dev/cash/src/bit.h(1004)
    wire[7:0] proxy14; // #5 fastmult.cpp(21)
    wire[3:0] proxy15; // #6 /home/blaise/dev/cash/src/bit.h(1004)
    wire[7:0] proxy17; // #7 fastmult.cpp(21)
    wire[7:0] sll20;
    wire[7:0] or22;
    wire[7:0] proxy23; // #10 unknown(0)

    assign io_lhs2 = io_lhs;
    assign io_rhs5 = io_rhs;
    assign io_out8 = mem10[proxy23];
    initial begin
        mem10[0] = 8'h0;
        mem10[1] = 8'h0;
        mem10[2] = 8'h0;
        mem10[3] = 8'h0;
        mem10[4] = 8'h0;
        mem10[5] = 8'h0;
        mem10[6] = 8'h0;
        mem10[7] = 8'h0;
        mem10[8] = 8'h0;
        mem10[9] = 8'h0;
        mem10[10] = 8'h0;
        mem10[11] = 8'h0;
        mem10[12] = 8'h0;
        mem10[13] = 8'h0;
        mem10[14] = 8'h0;
        mem10[15] = 8'h0;
        mem10[16] = 8'h0;
        mem10[17] = 8'h1;
        mem10[18] = 8'h2;
        mem10[19] = 8'h3;
        mem10[20] = 8'h4;
        mem10[21] = 8'h5;
        mem10[22] = 8'h6;
        mem10[23] = 8'h7;
        mem10[24] = 8'h8;
        mem10[25] = 8'h9;
        mem10[26] = 8'ha;
        mem10[27] = 8'hb;
        mem10[28] = 8'hc;
        mem10[29] = 8'hd;
        mem10[30] = 8'he;
        mem10[31] = 8'hf;
        mem10[32] = 8'h0;
        mem10[33] = 8'h2;
        mem10[34] = 8'h4;
        mem10[35] = 8'h6;
        mem10[36] = 8'h8;
        mem10[37] = 8'ha;
        mem10[38] = 8'hc;
        mem10[39] = 8'he;
        mem10[40] = 8'h10;
        mem10[41] = 8'h12;
        mem10[42] = 8'h14;
        mem10[43] = 8'h16;
        mem10[44] = 8'h18;
        mem10[45] = 8'h1a;
        mem10[46] = 8'h1c;
        mem10[47] = 8'h1e;
        mem10[48] = 8'h0;
        mem10[49] = 8'h3;
        mem10[50] = 8'h6;
        mem10[51] = 8'h9;
        mem10[52] = 8'hc;
        mem10[53] = 8'hf;
        mem10[54] = 8'h12;
        mem10[55] = 8'h15;
        mem10[56] = 8'h18;
        mem10[57] = 8'h1b;
        mem10[58] = 8'h1e;
        mem10[59] = 8'h21;
        mem10[60] = 8'h24;
        mem10[61] = 8'h27;
        mem10[62] = 8'h2a;
        mem10[63] = 8'h2d;
        mem10[64] = 8'h0;
        mem10[65] = 8'h4;
        mem10[66] = 8'h8;
        mem10[67] = 8'hc;
        mem10[68] = 8'h10;
        mem10[69] = 8'h14;
        mem10[70] = 8'h18;
        mem10[71] = 8'h1c;
        mem10[72] = 8'h20;
        mem10[73] = 8'h24;
        mem10[74] = 8'h28;
        mem10[75] = 8'h2c;
        mem10[76] = 8'h30;
        mem10[77] = 8'h34;
        mem10[78] = 8'h38;
        mem10[79] = 8'h3c;
        mem10[80] = 8'h0;
        mem10[81] = 8'h5;
        mem10[82] = 8'ha;
        mem10[83] = 8'hf;
        mem10[84] = 8'h14;
        mem10[85] = 8'h19;
        mem10[86] = 8'h1e;
        mem10[87] = 8'h23;
        mem10[88] = 8'h28;
        mem10[89] = 8'h2d;
        mem10[90] = 8'h32;
        mem10[91] = 8'h37;
        mem10[92] = 8'h3c;
        mem10[93] = 8'h41;
        mem10[94] = 8'h46;
        mem10[95] = 8'h4b;
        mem10[96] = 8'h0;
        mem10[97] = 8'h6;
        mem10[98] = 8'hc;
        mem10[99] = 8'h12;
        mem10[100] = 8'h18;
        mem10[101] = 8'h1e;
        mem10[102] = 8'h24;
        mem10[103] = 8'h2a;
        mem10[104] = 8'h30;
        mem10[105] = 8'h36;
        mem10[106] = 8'h3c;
        mem10[107] = 8'h42;
        mem10[108] = 8'h48;
        mem10[109] = 8'h4e;
        mem10[110] = 8'h54;
        mem10[111] = 8'h5a;
        mem10[112] = 8'h0;
        mem10[113] = 8'h7;
        mem10[114] = 8'he;
        mem10[115] = 8'h15;
        mem10[116] = 8'h1c;
        mem10[117] = 8'h23;
        mem10[118] = 8'h2a;
        mem10[119] = 8'h31;
        mem10[120] = 8'h38;
        mem10[121] = 8'h3f;
        mem10[122] = 8'h46;
        mem10[123] = 8'h4d;
        mem10[124] = 8'h54;
        mem10[125] = 8'h5b;
        mem10[126] = 8'h62;
        mem10[127] = 8'h69;
        mem10[128] = 8'h0;
        mem10[129] = 8'h8;
        mem10[130] = 8'h10;
        mem10[131] = 8'h18;
        mem10[132] = 8'h20;
        mem10[133] = 8'h28;
        mem10[134] = 8'h30;
        mem10[135] = 8'h38;
        mem10[136] = 8'h40;
        mem10[137] = 8'h48;
        mem10[138] = 8'h50;
        mem10[139] = 8'h58;
        mem10[140] = 8'h60;
        mem10[141] = 8'h68;
        mem10[142] = 8'h70;
        mem10[143] = 8'h78;
        mem10[144] = 8'h0;
        mem10[145] = 8'h9;
        mem10[146] = 8'h12;
        mem10[147] = 8'h1b;
        mem10[148] = 8'h24;
        mem10[149] = 8'h2d;
        mem10[150] = 8'h36;
        mem10[151] = 8'h3f;
        mem10[152] = 8'h48;
        mem10[153] = 8'h51;
        mem10[154] = 8'h5a;
        mem10[155] = 8'h63;
        mem10[156] = 8'h6c;
        mem10[157] = 8'h75;
        mem10[158] = 8'h7e;
        mem10[159] = 8'h87;
        mem10[160] = 8'h0;
        mem10[161] = 8'ha;
        mem10[162] = 8'h14;
        mem10[163] = 8'h1e;
        mem10[164] = 8'h28;
        mem10[165] = 8'h32;
        mem10[166] = 8'h3c;
        mem10[167] = 8'h46;
        mem10[168] = 8'h50;
        mem10[169] = 8'h5a;
        mem10[170] = 8'h64;
        mem10[171] = 8'h6e;
        mem10[172] = 8'h78;
        mem10[173] = 8'h82;
        mem10[174] = 8'h8c;
        mem10[175] = 8'h96;
        mem10[176] = 8'h0;
        mem10[177] = 8'hb;
        mem10[178] = 8'h16;
        mem10[179] = 8'h21;
        mem10[180] = 8'h2c;
        mem10[181] = 8'h37;
        mem10[182] = 8'h42;
        mem10[183] = 8'h4d;
        mem10[184] = 8'h58;
        mem10[185] = 8'h63;
        mem10[186] = 8'h6e;
        mem10[187] = 8'h79;
        mem10[188] = 8'h84;
        mem10[189] = 8'h8f;
        mem10[190] = 8'h9a;
        mem10[191] = 8'ha5;
        mem10[192] = 8'h0;
        mem10[193] = 8'hc;
        mem10[194] = 8'h18;
        mem10[195] = 8'h24;
        mem10[196] = 8'h30;
        mem10[197] = 8'h3c;
        mem10[198] = 8'h48;
        mem10[199] = 8'h54;
        mem10[200] = 8'h60;
        mem10[201] = 8'h6c;
        mem10[202] = 8'h78;
        mem10[203] = 8'h84;
        mem10[204] = 8'h90;
        mem10[205] = 8'h9c;
        mem10[206] = 8'ha8;
        mem10[207] = 8'hb4;
        mem10[208] = 8'h0;
        mem10[209] = 8'hd;
        mem10[210] = 8'h1a;
        mem10[211] = 8'h27;
        mem10[212] = 8'h34;
        mem10[213] = 8'h41;
        mem10[214] = 8'h4e;
        mem10[215] = 8'h5b;
        mem10[216] = 8'h68;
        mem10[217] = 8'h75;
        mem10[218] = 8'h82;
        mem10[219] = 8'h8f;
        mem10[220] = 8'h9c;
        mem10[221] = 8'ha9;
        mem10[222] = 8'hb6;
        mem10[223] = 8'hc3;
        mem10[224] = 8'h0;
        mem10[225] = 8'he;
        mem10[226] = 8'h1c;
        mem10[227] = 8'h2a;
        mem10[228] = 8'h38;
        mem10[229] = 8'h46;
        mem10[230] = 8'h54;
        mem10[231] = 8'h62;
        mem10[232] = 8'h70;
        mem10[233] = 8'h7e;
        mem10[234] = 8'h8c;
        mem10[235] = 8'h9a;
        mem10[236] = 8'ha8;
        mem10[237] = 8'hb6;
        mem10[238] = 8'hc4;
        mem10[239] = 8'hd2;
        mem10[240] = 8'h0;
        mem10[241] = 8'hf;
        mem10[242] = 8'h1e;
        mem10[243] = 8'h2d;
        mem10[244] = 8'h3c;
        mem10[245] = 8'h4b;
        mem10[246] = 8'h5a;
        mem10[247] = 8'h69;
        mem10[248] = 8'h78;
        mem10[249] = 8'h87;
        mem10[250] = 8'h96;
        mem10[251] = 8'ha5;
        mem10[252] = 8'hb4;
        mem10[253] = 8'hc3;
        mem10[254] = 8'hd2;
        mem10[255] = 8'he1;
    end
    assign proxy12 = 4'h0;
    assign proxy14 = {proxy12, io_rhs5};
    assign proxy15 = 4'h0;
    assign proxy17 = {proxy15, io_lhs2};
    assign sll20 = proxy17 << 8'h4;
    assign or22 = sll20 | proxy14;
    assign proxy23 = or22;

    assign io_out = io_out8;

endmodule
