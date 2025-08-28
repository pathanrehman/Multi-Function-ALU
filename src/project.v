/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_example (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // ALU Operation codes (5 bits for 28 operations)
    wire [4:0] alu_op = ui_in[4:0];
    wire load_operand = ui_in[5];
    wire operand_sel = ui_in[6];
    wire output_sel = ui_in[7];
    
    // Internal registers for 32-bit operands
    reg [31:0] operand_a;
    reg [31:0] operand_b;
    
    // ALU outputs
    wire [31:0] alu_result;
    wire zero_flag, carry_flag, overflow_flag, negative_flag;
    
    // Load operands from input (8 bits at a time)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            operand_a <= 32'b0;
            operand_b <= 32'b0;
        end else if (load_operand) begin
            if (operand_sel == 1'b0) begin
                // Load operand A (shift in 8 bits)
                operand_a <= {operand_a[23:0], uio_in};
            end else begin
                // Load operand B (shift in 8 bits)
                operand_b <= {operand_b[23:0], uio_in};
            end
        end
    end
    
    // Instantiate the 32-bit Multi-Function ALU
    alu_32bit alu_inst (
        .a(operand_a),
        .b(operand_b),
        .op(alu_op),
        .result(alu_result),
        .zero(zero_flag),
        .carry(carry_flag),
        .overflow(overflow_flag),
        .negative(negative_flag)
    );
    
    // Output selection
    reg [7:0] output_data;
    always @(*) begin
        case (output_sel)
            1'b0: output_data = alu_result[7:0];   // Lower 8 bits of result
            1'b1: output_data = {4'b0, zero_flag, carry_flag, overflow_flag, negative_flag}; // Status flags
        endcase
    end
    
    // Output assignments
    assign uo_out = output_data;
    assign uio_out = alu_result[15:8]; // Next 8 bits of result
    assign uio_oe = 8'hFF; // All uio pins as outputs
    
    // Unused input
    wire _unused = &{ena, 1'b0};

endmodule

// 32-bit Multi-Function ALU Module
module alu_32bit (
    input [31:0] a,
    input [31:0] b,
    input [4:0] op,
    output reg [31:0] result,
    output zero,
    output carry,
    output overflow,
    output negative
);

    // Internal signals
    wire [31:0] add_result, sub_result;
    wire [31:0] mult_result_low;
    wire [31:0] div_result, mod_result;
    wire [31:0] shift_result;
    wire add_carry, sub_carry, add_overflow, sub_overflow;
    
    // Carry Lookahead Adder instantiation
    cla_32bit adder (
        .a(a),
        .b(b),
        .cin(1'b0),
        .sum(add_result),
        .cout(add_carry),
        .overflow(add_overflow)
    );
    
    // Subtractor (using 2's complement)
    wire [31:0] b_complement = ~b + 1'b1;
    cla_32bit subtractor (
        .a(a),
        .b(b_complement),
        .cin(1'b0),
        .sum(sub_result),
        .cout(sub_carry),
        .overflow(sub_overflow)
    );
    
    // Simple multiplier (partial implementation for lower 32 bits)
    assign mult_result_low = a[15:0] * b[15:0];
    
    // Simple divider (basic implementation)
    assign div_result = (b != 0) ? (a / b) : 32'hFFFFFFFF;
    assign mod_result = (b != 0) ? (a % b) : 32'h00000000;
    
    // Barrel shifter
    barrel_shifter shifter (
        .data_in(a),
        .shift_amount(b[4:0]),
        .shift_type(op[1:0]), // 00=left, 01=right logical, 10=right arithmetic
        .data_out(shift_result)
    );
    
    // ALU operation selection
    always @(*) begin
        case (op)
            // Arithmetic operations
            5'b00000: result = add_result;              // ADD
            5'b00001: result = sub_result;              // SUB
            5'b00010: result = mult_result_low;         // MUL (lower 32 bits)
            5'b00011: result = div_result;              // DIV
            5'b00100: result = a + 1;                   // INC
            5'b00101: result = a - 1;                   // DEC
            5'b00110: result = mod_result;              // MOD
            5'b00111: result = -a;                      // NEG
            5'b01000: result = (a > b) ? a : b;         // MAX
            5'b01001: result = (a < b) ? a : b;         // MIN
            5'b01010: result = a + b + 1;               // ADC (add with carry)
            5'b01011: result = a - b - 1;               // SBB (subtract with borrow)
            5'b01100: result = {a[15:0], 16'b0};        // SWAP bytes
            5'b01101: result = {16'b0, a[31:16]};       // SWAP high/low
            5'b01110: result = a * a;                   // SQR (square)
            5'b01111: result = a;                       // PASS A
            
            // Logical operations
            5'b10000: result = a & b;                   // AND
            5'b10001: result = a | b;                   // OR
            5'b10010: result = a ^ b;                   // XOR
            5'b10011: result = ~a;                      // NOT
            5'b10100: result = ~(a & b);                // NAND
            5'b10101: result = ~(a | b);                // NOR
            5'b10110: result = ~(a ^ b);                // XNOR
            5'b10111: result = a & ~b;                  // AND NOT
            5'b11000: result = shift_result;            // Shift operations
            5'b11001: result = {a[30:0], a[31]};        // ROL (rotate left)
            5'b11010: result = {a[0], a[31:1]};         // ROR (rotate right)
            5'b11011: result = {a[0], a[31:1]};         // RCR (rotate right through carry)
            
            default: result = 32'b0;
        endcase
    end
    
    // Status flags
    assign zero = (result == 32'b0);
    assign negative = result[31];
    assign carry = (op == 5'b00000) ? add_carry : 
                   (op == 5'b00001) ? sub_carry : 1'b0;
    assign overflow = (op == 5'b00000) ? add_overflow : 
                      (op == 5'b00001) ? sub_overflow : 1'b0;

endmodule

// 32-bit Carry Lookahead Adder
module cla_32bit (
    input [31:0] a,
    input [31:0] b,
    input cin,
    output [31:0] sum,
    output cout,
    output overflow
);

    wire [7:0] c;
    wire [7:0] g_group, p_group;
    
    assign c[0] = cin;
    
    // Generate 4-bit CLA blocks
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : cla_blocks
            cla_4bit cla_block (
                .a(a[i*4+3:i*4]),
                .b(b[i*4+3:i*4]),
                .cin(c[i]),
                .sum(sum[i*4+3:i*4]),
                .cout(c[i+1]),
                .g(g_group[i]),
                .p(p_group[i])
            );
        end
    endgenerate
    
    assign cout = c[8];
    assign overflow = c[8] ^ c[7];

endmodule

// 4-bit Carry Lookahead Adder
module cla_4bit (
    input [3:0] a,
    input [3:0] b,
    input cin,
    output [3:0] sum,
    output cout,
    output g,
    output p
);

    wire [3:0] gi, pi;
    wire [3:0] c;
    
    // Generate and propagate for each bit
    assign gi = a & b;
    assign pi = a ^ b;
    
    // Carry calculations
    assign c[0] = cin;
    assign c[1] = gi[0] | (pi[0] & c[0]);
    assign c[2] = gi[1] | (pi[1] & gi[0]) | (pi[1] & pi[0] & c[0]);
    assign c[3] = gi[2] | (pi[2] & gi[1]) | (pi[2] & pi[1] & gi[0]) | 
                  (pi[2] & pi[1] & pi[0] & c[0]);
    assign cout = gi[3] | (pi[3] & gi[2]) | (pi[3] & pi[2] & gi[1]) | 
                  (pi[3] & pi[2] & pi[1] & gi[0]) | (pi[3] & pi[2] & pi[1] & pi[0] & c[0]);
    
    // Sum calculation
    assign sum = pi ^ c;
    
    // Group generate and propagate
    assign g = gi[3] | (pi[3] & gi[2]) | (pi[3] & pi[2] & gi[1]) | (pi[3] & pi[2] & pi[1] & gi[0]);
    assign p = pi[3] & pi[2] & pi[1] & pi[0];

endmodule

// Barrel Shifter
module barrel_shifter (
    input [31:0] data_in,
    input [4:0] shift_amount,
    input [1:0] shift_type, // 00=left, 01=right logical, 10=right arithmetic
    output reg [31:0] data_out
);

    always @(*) begin
        case (shift_type)
            2'b00: data_out = data_in << shift_amount;                    // Left shift
            2'b01: data_out = data_in >> shift_amount;                    // Right logical shift
            2'b10: data_out = $signed(data_in) >>> shift_amount;         // Right arithmetic shift
            2'b11: data_out = data_in;                                    // No shift
            default: data_out = data_in;
        endcase
    end

endmodule
