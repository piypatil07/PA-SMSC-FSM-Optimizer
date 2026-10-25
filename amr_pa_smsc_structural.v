`timescale 1ns/1ps

// =====================================================================
// POWER-AWARE STRUCTURAL MIXED STATE CODE (PA-SMSC) FSM
// Target: 32-State Autonomous Mobile Robot (AMR) Controller
// Architecture: Explicit 2-Level LUT Structure (Graph-Based Partitioned)
// =====================================================================
module amr_pa_smsc_fsm (
    input  wire        clk,
    input  wire        rst,
    input  wire [12:1] x,      // 12 Inputs (1-indexed for CSV match)
    output wire [12:1] y       // 12 Outputs (1-indexed for CSV match)
);

    // -------------------------------------------------------------------------
    // 1. ENCODING PARAMETERS
    // -------------------------------------------------------------------------
    parameter RV  = 4;   // Class Code bits (K=9)
    parameter RS  = 3;   // Max Partial State Code bits
    parameter REC = 7;   // Total Mixed State Code bits (RV + RS)
    parameter K   = 9;   // Number of Classes

    // -------------------------------------------------------------------------
    // 2. STATE DEFINITIONS (7-bit Final MSC)
    // -------------------------------------------------------------------------
    // Class 0
    localparam [REC-1:0] a17 = 7'b0000_000;
    localparam [REC-1:0] a24 = 7'b0000_001;
    localparam [REC-1:0] a5  = 7'b0000_010;
    localparam [REC-1:0] a8  = 7'b0000_011;
    // Class 1
    localparam [REC-1:0] a14 = 7'b0001_010;
    localparam [REC-1:0] a19 = 7'b0001_001;
    localparam [REC-1:0] a20 = 7'b0001_100;
    localparam [REC-1:0] a21 = 7'b0001_000;
    localparam [REC-1:0] a25 = 7'b0001_111;
    localparam [REC-1:0] a30 = 7'b0001_011;
    localparam [REC-1:0] a31 = 7'b0001_101;
    localparam [REC-1:0] a32 = 7'b0001_110;
    // Class 2
    localparam [REC-1:0] a10 = 7'b0010_011;
    localparam [REC-1:0] a26 = 7'b0010_010;
    localparam [REC-1:0] a4  = 7'b0010_000;
    localparam [REC-1:0] a6  = 7'b0010_001;
    // Class 3 (Contains Initial State a1)
    localparam [REC-1:0] a1  = 7'b0011_000; 
    localparam [REC-1:0] a22 = 7'b0011_001;
    localparam [REC-1:0] a7  = 7'b0011_011;
    localparam [REC-1:0] a9  = 7'b0011_010;
    // Class 4
    localparam [REC-1:0] a11 = 7'b0100_000;
    localparam [REC-1:0] a13 = 7'b0100_001;
    localparam [REC-1:0] a3  = 7'b0100_011;
    // Class 5
    localparam [REC-1:0] a16 = 7'b0101_001;
    localparam [REC-1:0] a23 = 7'b0101_000;
    // Class 6
    localparam [REC-1:0] a12 = 7'b0110_010;
    localparam [REC-1:0] a15 = 7'b0110_011;
    localparam [REC-1:0] a27 = 7'b0110_001;
    localparam [REC-1:0] a29 = 7'b0110_000;
    // Class 7
    localparam [REC-1:0] a18 = 7'b0111_001;
    localparam [REC-1:0] a28 = 7'b0111_000;
    // Class 8
    localparam [REC-1:0] a2  = 7'b1000_000;

    // -------------------------------------------------------------------------
    // 3. STATE REGISTER & INTERNAL SIGNALS (DECLARED UPFRONT)
    // -------------------------------------------------------------------------
    reg [REC-1:0] cs, ns; 
    reg [12:1] y_comb;    
    reg [12:1] y_next_comb; // FIX: Moved declaration up to prevent "after use" error

    wire [RV-1:0]  class_code   = cs[REC-1 : RS]; 
    wire [RS-1:0]  partial_code = cs[RS-1 : 0];   

    // Level 1 Partial Function Registers
    reg [REC-1:0] D0_partial, D1_partial, D2_partial, D3_partial, D4_partial;
    reg [REC-1:0] D5_partial, D6_partial, D7_partial, D8_partial;
    reg [12:1]    Y0_partial, Y1_partial, Y2_partial, Y3_partial, Y4_partial;
    reg [12:1]    Y5_partial, Y6_partial, Y7_partial, Y8_partial;

    // Sequential Logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cs     <= a1;       
            y_comb <= 12'b0;
        end else begin
            cs     <= ns;
            y_comb <= y_next_comb; 
        end
    end

    // =========================================================================
    // LEVEL 1: PARTIAL FUNCTION GENERATORS (Auto-Generated)
    // =========================================================================

    // --- LEVEL 1, CLASS 0 ---
    always @(*) begin
        D0_partial = 7'b0; Y0_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a17
                if (!x[7]) begin D0_partial = 7'b0000000; Y0_partial = 12'b100100000011; end
                else if (x[7]) begin D0_partial = 7'b1000000; Y0_partial = 12'b100000000011; end
                else begin D0_partial = 7'b0000000; Y0_partial = 12'b0; end
            end
            3'b001: begin // State a24
                if (x[10]) begin D0_partial = 7'b0111000; Y0_partial = 12'b100100000000; end
                else if (!x[10]) begin D0_partial = 7'b0000001; Y0_partial = 12'b100100000000; end
                else begin D0_partial = 7'b0000001; Y0_partial = 12'b0; end
            end
            3'b010: begin // State a5
                if (x[2]) begin D0_partial = 7'b0000010; Y0_partial = 12'b100000000011; end
                else if (!x[2]) begin D0_partial = 7'b1000000; Y0_partial = 12'b100000000011; end
                else begin D0_partial = 7'b0000010; Y0_partial = 12'b0; end
            end
            3'b011: begin // State a8
                if (x[12]) begin D0_partial = 7'b0011010; Y0_partial = 12'b100100000100; end
                else if (!x[12]) begin D0_partial = 7'b0011011; Y0_partial = 12'b100000000010; end
                else begin D0_partial = 7'b0000011; Y0_partial = 12'b0; end
            end
            default: begin D0_partial = 7'b0; Y0_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 1 ---
    always @(*) begin
        D1_partial = 7'b0; Y1_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a21
                if (!x[11]) begin D1_partial = 7'b0001100; Y1_partial = 12'b100000001000; end
                else begin D1_partial = 7'b0001000; Y1_partial = 12'b0; end
            end
            3'b001: begin // State a19
                if (x[11]) begin D1_partial = 7'b0001000; Y1_partial = 12'b100100001000; end
                else begin D1_partial = 7'b0001001; Y1_partial = 12'b0; end
            end
            3'b010: begin // State a14
                if (!x[1]) begin D1_partial = 7'b0001010; Y1_partial = 12'b110001001000; end
                else if (x[11]) begin D1_partial = 7'b0001100; Y1_partial = 12'b110000001000; end
                else begin D1_partial = 7'b0001010; Y1_partial = 12'b0; end
            end
            3'b011: begin // State a30
                if (!x[6]) begin D1_partial = 7'b0011001; Y1_partial = 12'b100000000000; end
                else begin D1_partial = 7'b0001011; Y1_partial = 12'b0; end
            end
            3'b100: begin // State a20
                if (x[11]) begin D1_partial = 7'b0011000; Y1_partial = 12'b100000000000; end
                else begin D1_partial = 7'b0001100; Y1_partial = 12'b0; end
            end
            3'b101: begin // State a31
                if (x[11]) begin D1_partial = 7'b0000001; Y1_partial = 12'b100010101000; end
                else begin D1_partial = 7'b0001101; Y1_partial = 12'b0; end
            end
            3'b110: begin // State a32
                if (x[11]) begin D1_partial = 7'b0011000; Y1_partial = 12'b100100000000; end
                else begin D1_partial = 7'b0001110; Y1_partial = 12'b0; end
            end
            3'b111: begin // State a25
                if (!x[6]) begin D1_partial = 7'b0010010; Y1_partial = 12'b100000010000; end
                else begin D1_partial = 7'b0001111; Y1_partial = 12'b0; end
            end
            default: begin D1_partial = 7'b0; Y1_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 2 ---
    always @(*) begin
        D2_partial = 7'b0; Y2_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a4
                if (x[2]) begin D2_partial = 7'b0010000; Y2_partial = 12'b100000000011; end
                else if (!x[2]) begin D2_partial = 7'b1000000; Y2_partial = 12'b100000000011; end
                else begin D2_partial = 7'b0010000; Y2_partial = 12'b0; end
            end
            3'b001: begin // State a6
                if (x[1] && x[2]) begin D2_partial = 7'b1000000; Y2_partial = 12'b100100000011; end
                else begin D2_partial = 7'b0010001; Y2_partial = 12'b0; end
            end
            3'b010: begin // State a26
                if (x[11]) begin D2_partial = 7'b0101000; Y2_partial = 12'b100000000000; end
                else begin D2_partial = 7'b0010010; Y2_partial = 12'b0; end
            end
            3'b011: begin // State a10
                if (x[10]) begin D2_partial = 7'b0100000; Y2_partial = 12'b100000000011; end
                else begin D2_partial = 7'b0010011; Y2_partial = 12'b0; end
            end
            default: begin D2_partial = 7'b0; Y2_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 3 ---
    always @(*) begin
        D3_partial = 7'b0; Y3_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a1 (Initial)
                if (x[8] && x[9]) begin D3_partial = 7'b1000000; Y3_partial = 12'b100000000011; end
                else if (!x[8] && !x[9]) begin D3_partial = 7'b0011000; Y3_partial = 12'b100000000000; end
                else begin D3_partial = 7'b0011000; Y3_partial = 12'b0; end
            end
            3'b001: begin // State a22
                if (x[11]) begin D3_partial = 7'b0110001; Y3_partial = 12'b100000000000; end
                else begin D3_partial = 7'b0011001; Y3_partial = 12'b0; end
            end
            3'b010: begin // State a9
                if (x[11]) begin D3_partial = 7'b0001111; Y3_partial = 12'b100100010000; end
                else begin D3_partial = 7'b0011010; Y3_partial = 12'b0; end
            end
            3'b011: begin // State a7
                if (x[12]) begin D3_partial = 7'b0000011; Y3_partial = 12'b100100000010; end
                else if (!x[12]) begin D3_partial = 7'b0011011; Y3_partial = 12'b100000000010; end
                else begin D3_partial = 7'b0011011; Y3_partial = 12'b0; end
            end
            default: begin D3_partial = 7'b0; Y3_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 4 ---
    always @(*) begin
        D4_partial = 7'b0; Y4_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a11
                if (x[10]) begin D4_partial = 7'b0100000; Y4_partial = 12'b100000000011; end
                else if (!x[10]) begin D4_partial = 7'b0011000; Y4_partial = 12'b100000000000; end
                else begin D4_partial = 7'b0100000; Y4_partial = 12'b0; end
            end
            3'b001: begin // State a13
                if (!x[3]) begin D4_partial = 7'b1000000; Y4_partial = 12'b100000000011; end
                else if (x[3]) begin D4_partial = 7'b0100001; Y4_partial = 12'b100000001100; end
                else begin D4_partial = 7'b0100001; Y4_partial = 12'b0; end
            end
            3'b011: begin // State a3
                if (x[1] && x[2]) begin D4_partial = 7'b1000000; Y4_partial = 12'b100000000011; end
                else if (x[3]) begin D4_partial = 7'b0100001; Y4_partial = 12'b100000001100; end
                else begin D4_partial = 7'b0100011; Y4_partial = 12'b0; end
            end
            default: begin D4_partial = 7'b0; Y4_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 5 ---
    always @(*) begin
        D5_partial = 7'b0; Y5_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a23
                if (x[8] && x[9]) begin D5_partial = 7'b0110001; Y5_partial = 12'b100000000000; end
                else if (x[6]) begin D5_partial = 7'b0001111; Y5_partial = 12'b100000010000; end
                else begin D5_partial = 7'b0101000; Y5_partial = 12'b0; end
            end
            3'b001: begin // State a16
                if (x[6]) begin D5_partial = 7'b0101001; Y5_partial = 12'b110000011000; end
                else if (x[11]) begin D5_partial = 7'b0001111; Y5_partial = 12'b110000010000; end
                else begin D5_partial = 7'b0101001; Y5_partial = 12'b0; end
            end
            default: begin D5_partial = 7'b0; Y5_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 6 ---
    always @(*) begin
        D6_partial = 7'b0; Y6_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a29
                if (x[4] && x[10]) begin D6_partial = 7'b1000000; Y6_partial = 12'b100100000011; end
                else begin D6_partial = 7'b0110000; Y6_partial = 12'b0; end
            end
            3'b001: begin // State a27
                if (x[4] && x[10]) begin D6_partial = 7'b0110000; Y6_partial = 12'b100100000000; end
                else begin D6_partial = 7'b0110001; Y6_partial = 12'b0; end
            end
            3'b010: begin // State a12
                if (x[3]) begin D6_partial = 7'b0110010; Y6_partial = 12'b110000001100; end
                else if (x[11]) begin D6_partial = 7'b0001100; Y6_partial = 12'b110000001000; end
                else begin D6_partial = 7'b0110010; Y6_partial = 12'b0; end
            end
            3'b011: begin // State a15
                if (!x[10]) begin D6_partial = 7'b0110011; Y6_partial = 12'b110010000100; end
                else if (x[11]) begin D6_partial = 7'b0001100; Y6_partial = 12'b110000000100; end
                else begin D6_partial = 7'b0110011; Y6_partial = 12'b0; end
            end
            default: begin D6_partial = 7'b0; Y6_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 7 ---
    always @(*) begin
        D7_partial = 7'b0; Y7_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a28
                if (x[4] && x[10]) begin D7_partial = 7'b0110000; Y7_partial = 12'b100100000000; end
                else begin D7_partial = 7'b0111000; Y7_partial = 12'b0; end
            end
            3'b001: begin // State a18
                if (!x[4]) begin D7_partial = 7'b0111001; Y7_partial = 12'b110001000100; end
                else if (x[11]) begin D7_partial = 7'b0001100; Y7_partial = 12'b110000000100; end
                else begin D7_partial = 7'b0111001; Y7_partial = 12'b0; end
            end
            default: begin D7_partial = 7'b0; Y7_partial = 12'b0; end
        endcase
    end

    // --- LEVEL 1, CLASS 8 ---
    always @(*) begin
        D8_partial = 7'b0; Y8_partial = 12'b0;
        case (partial_code)
            3'b000: begin // State a2
                if (x[1] && x[2]) begin D8_partial = 7'b1000000; Y8_partial = 12'b100000000011; end
                else if (!x[1]) begin D8_partial = 7'b0100011; Y8_partial = 12'b100000000100; end
                else if (!x[8] && !x[9]) begin D8_partial = 7'b0101000; Y8_partial = 12'b100000000000; end
                else begin D8_partial = 7'b1000000; Y8_partial = 12'b0; end
            end
            default: begin D8_partial = 7'b0; Y8_partial = 12'b0; end
        endcase
    end

    // =========================================================================
    // LEVEL 2: FINAL VALUE GENERATOR / MULTIPLEXER
    // =========================================================================
    // FIX: We assign 'ns' directly here instead of using 'ns_internal' + assign
    always @(*) begin
        ns = 7'b0;
        y_next_comb = 12'b0;
        
        case (class_code)
            4'b0000: begin ns = D0_partial; y_next_comb = Y0_partial; end
            4'b0001: begin ns = D1_partial; y_next_comb = Y1_partial; end
            4'b0010: begin ns = D2_partial; y_next_comb = Y2_partial; end
            4'b0011: begin ns = D3_partial; y_next_comb = Y3_partial; end
            4'b0100: begin ns = D4_partial; y_next_comb = Y4_partial; end
            4'b0101: begin ns = D5_partial; y_next_comb = Y5_partial; end
            4'b0110: begin ns = D6_partial; y_next_comb = Y6_partial; end
            4'b0111: begin ns = D7_partial; y_next_comb = Y7_partial; end
            4'b1000: begin ns = D8_partial; y_next_comb = Y8_partial; end
            default: begin ns = a1; y_next_comb = 12'b0; end // Safe fallback
        endcase
    end

    // FIX: Removed 'assign ns = ns_internal;' because ns is a reg driven by always@(*)
    assign y  = y_comb;

endmodule