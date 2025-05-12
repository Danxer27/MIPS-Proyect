//Daniel Joel Corona Espinoza
//Yahir Efren Borboa Quintero
module DataPath(
    input CLK,
    output [31:0] DS
);

//Cables
wire [31:0] instruction;

wire cDemux, cWe, cRe, cZf, cBRWe, cBranch, cRegDst, cAddPcMuxo, cAluSrc;
wire [2:0] cALU_op;
wire [3:0] C4_Sel;  // Declarar C4_Sel
wire [31:0] C1_RD1, C2_RD2, C3_AluRes, cDW, CS, cAluMux;

// Eliminar señales duplicadas o innecesarias
wire [31:0] pcOut, sumOut, cSignex, cAddMuxo, cAddPc, cShiftAdd;

wire [4:0] BancMuxOut;

//Instancias

PC pc(.clk(CLK), .dirIn(cAddPc), .dirOut(pcOut));
Alud sum(.dir(pcOut), .dirOut(sumOut));
Memory mem(.dir(pcOut), .instructionOut(instruction));

SignExtend Signex(
    .signIn(instruction[15:0]),
    //output
    .signOut(cSignex)
);

AddAR ALuAdd(
    .addIn1(sumOut),
    .addIn2(cShiftAdd),
    //output
    .addOut(cAddMuxo)
);

Mux AddPCMuxo(
    .dmx(cAddPcMuxo),
    .data1In(sumOut),
    .data2In(cAddMuxo),
    //output
    .dataOut(cAddPc)
);

AndBr cand(
    .andIn1(cBranch),
    .andIn2(cZf),
    //output
    .andOut(cAddPcMuxo)
);

ShifLeft SL(
    .shiftIn(cSignex),
    .shiftOut(cShiftAdd)
);

//
Controller Control(//Inputs
    .Op(instruction[31:26]), 
    //Ouputs
    .Demuxo(cDemux), 
    .WeMD(cWe), 
    .ReMD(cRe),
    .BRWe(cBRWe),
    .ALU_op(cALU_op),
    .regDst(cRegDst),
    .Branch(cBranch),
    .aluSrc(cAluSrc)
);

Alu_Control AluController(
    .funct(instruction[5:0]),
    .ALUop(cALU_op),
    //Output:
    .sel(C4_Sel)
);

MuxBanc BancMux(
    .dmx(cRegDst),
    .data1In(instruction[20:16]),
    .data2In(instruction[15:11]),
    //output
    .dataOut(BancMuxOut)
);

BancRegister Banco( //Inputs
    .RA1(instruction[25:21]), 
    .RA2(instruction[20:16]), 
    .WA(BancMuxOut),
    .DW(cDW),
    .WE(cBRWe),
    //Outputs
    .data1Out(C1_RD1), 
    .data2Out(C2_RD2)
);

Mux Muxalu (
    .dmx(cAluSrc),
    .data1In(C2_RD2),
    .data2In(cSignex),
    //output:
    .dataOut(cAluMux)
);

ALU Aluru(
    .A(C1_RD1),
    .B(cAluMux),
    .ALU_OP(C4_Sel),
    //Ouputs 
    .R(C3_AluRes),
    .Zero_Flag(cZf)
);

MemoryData MemData(
    .WEn(cWe),
    .REn(cRe),
    .dataIn(C2_RD2),
    .addres(C3_AluRes),
    //Output
    .dataOut(CS)
);

Demuxo Demux(
    .dmx(cDemux),
    .dataAluIn(C3_AluRes),
    .dataMemIn(CS),
    //OutPut:
    .dataOut(cDW)
);

assign DS = cDW;

endmodule

//Controlador
module Controller(
    input [5:0] Op,
    output reg Demuxo, WeMD, ReMD, BRWe, regDst, Branch, aluSrc,
    output reg [2:0] ALU_op
);

always @(*) begin
    case (Op)
        6'b000000: begin   // R-type
            ALU_op = 3'b010;
            Demuxo = 1'b1;      // ALU result to register
            BRWe = 1'b1;        // Write to register
            WeMD = 1'b0;        // Don't write to memory
            ReMD = 1'b0;        // Don't read from memory
            regDst = 1'b1;      // Destination is rd
            Branch = 1'b0;      // No branch
            aluSrc = 1'b0;      // Second operand from register
        end

        6'b000100: begin  // BEQ
            ALU_op = 3'b001;    // Subtract for comparison
            Demuxo = 1'bx;      // Don't care (not writing to register)
            BRWe = 1'b0;        // Don't write to register
            WeMD = 1'b0;        // Don't write to memory
            ReMD = 1'b0;        // Don't read from memory
            regDst = 1'bx;      // Don't care
            Branch = 1'b1;      // This is a branch
            aluSrc = 1'b0;      // Use register for comparison
        end

        6'b100011: begin  // LW - Load Word
            ALU_op = 3'b000;    // Add for address calculation
            Demuxo = 1'b0;      // Memory result to register
            BRWe = 1'b1;        // Write to register
            WeMD = 1'b0;        // Don't write to memory
            ReMD = 1'b1;        // Read from memory
            regDst = 1'b0;      // Destination is rt
            Branch = 1'b0;      // No branch
            aluSrc = 1'b1;      // Use immediate value
        end

        6'b101011: begin  // SW - Store Word
            ALU_op = 3'b000;    // Add for address calculation
            Demuxo = 1'bx;      // Don't care (not writing to register)
            BRWe = 1'b0;        // Don't write to register
            WeMD = 1'b1;        // Write to memory
            ReMD = 1'b0;        // Don't read from memory
            regDst = 1'bx;      // Don't care
            Branch = 1'b0;      // No branch
            aluSrc = 1'b1;      // Use immediate value
        end

        6'b001101: begin  // ORI
            ALU_op = 3'b100;    // OR operation
            Demuxo = 1'b1;      // ALU result to register
            BRWe = 1'b1;        // Write to register
            WeMD = 1'b0;        // Don't write to memory
            ReMD = 1'b0;        // Don't read from memory
            regDst = 1'b0;      // Destination is rt
            Branch = 1'b0;      // No branch
            aluSrc = 1'b1;      // Use immediate value
        end

        6'b001100: begin  // ANDI (cambiar de ADDI)
            ALU_op = 3'b101;    // AND operation
            Demuxo = 1'b1;      // ALU result to register
            BRWe = 1'b1;        // Write to register
            WeMD = 1'b0;        // Don't write to memory
            ReMD = 1'b0;        // Don't read from memory
            regDst = 1'b0;      // Destination is rt
            Branch = 1'b0;      // No branch
            aluSrc = 1'b1;      // Use immediate value
        end

        6'b001000: begin  // ADDI (cambiar código)
            ALU_op = 3'b000;    // Add operation
            Demuxo = 1'b1;      // ALU result to register
            BRWe = 1'b1;        // Write to register
            WeMD = 1'b0;        // Don't write to memory
            ReMD = 1'b0;        // Don't read from memory
            regDst = 1'b0;      // Destination is rt
            Branch = 1'b0;      // No branch
            aluSrc = 1'b1;      // Use immediate value
        end

        6'b001010: begin  // SLTI
            ALU_op = 3'b011;    // SLT operation
            Demuxo = 1'b1;      // ALU result to register
            BRWe = 1'b1;        // Write to register
            WeMD = 1'b0;        // Don't write to memory
            ReMD = 1'b0;        // Don't read from memory
            regDst = 1'b0;      // Destination is rt
            Branch = 1'b0;      // No branch
            aluSrc = 1'b1;      // Use immediate value
        end

        default: begin
            ALU_op = 3'b000;
            Demuxo = 1'b0;
            BRWe = 1'b0;
            WeMD = 1'b0;
            ReMD = 1'b0;
            regDst = 1'b0;
            Branch = 1'b0;
            aluSrc = 1'b0;
        end

    endcase
end
endmodule

//### ALU CONTROL
module Alu_Control(
    input [5:0] funct,
    input [2:0] ALUop,
    output reg [3:0] sel
);

always @(*) begin
    case (ALUop)
        3'b000:
            sel = 4'b0010;  // ADD
        3'b001:
            sel = 4'b0110;  // SUB
        3'b010:
            case (funct) 
                6'b100000: sel = 4'b0010;  // ADD
                6'b100010: sel = 4'b0110;  // SUB
                6'b100100: sel = 4'b0000;  // AND
                6'b100101: sel = 4'b0001;  // OR
                6'b101010: sel = 4'b0111;  // SLT
                default:   sel = 4'b0010;  // Default ADD
            endcase
        3'b011: sel = 4'b0111;  // SLT
        3'b100: sel = 4'b0001;  // OR
        3'b101: sel = 4'b0000;  // AND
        default: sel = 4'b0010; // Default ADD
    endcase
end
endmodule

//Banco de Registros
module BancRegister( 
    input [4:0] RA1, RA2, WA,
    input [31:0] DW,
    input WE,
    output reg [31:0] data1Out, data2Out
);

reg [31:0] mem [0:31];

initial begin
    $readmemb("data", mem);
end

always @ (*) begin
    data1Out = mem[RA1];
    data2Out = mem[RA2];
end

// Para evitar problemas combinacionales, la escritura debe ser en un bloque separado
// pero sin reloj puede causar problemas de síntesis
always @ (*) begin
    if(WE) begin
        mem[WA] = DW;
    end
end
endmodule

module Mux(
    input dmx,
    input [31:0] data1In, data2In,
    output reg [31:0] dataOut 
);

always @(*) begin
    case (dmx)
        1'b0: dataOut = data1In;               
        1'b1: dataOut = data2In;        
    endcase
end
endmodule

module MuxBanc(
    input dmx,
    input [4:0] data1In, data2In,
    output reg [4:0] dataOut   
);

always @(*) begin
    case (dmx)
        1'b0: dataOut = data1In;               
        1'b1: dataOut = data2In;        
    endcase
end
endmodule

//Demultiplexor Mem / Alu => Registers
module Demuxo(
    input dmx,
    input [31:0] dataAluIn, dataMemIn,
    output reg [31:0] dataOut
);

always @(*) begin
    case (dmx)
        1'b0: dataOut = dataMemIn;               
        1'b1: dataOut = dataAluIn;        
    endcase
end
endmodule

//Memoria de Datos
module MemoryData(
    input WEn, REn,
    input [31:0] dataIn, 
    input [31:0] addres,
    output reg [31:0] dataOut
);

reg [31:0] mem [0:127];

always @ (*) begin
    if(REn) begin
        dataOut = mem[addres];
    end else begin
        dataOut = 32'bz;  // Alta impedancia cuando no se lee
    end
end

// Escritura combinacional (problema potencial)
always @ (*) begin
    if(WEn) begin
        mem[addres] = dataIn;
    end
end
endmodule

//ALU
module ALU (
    input [31:0] A,
    input [31:0] B,
    input [3:0] ALU_OP,
    output reg [31:0] R,
    output reg Zero_Flag
);

always @(*) begin
    case (ALU_OP)
        4'b0000: R = A & B;        
        4'b0001: R = A | B;        
        4'b0010: R = A + B;       
        4'b0110: R = A - B;        
        4'b0111: R = (A < B) ? 32'd1 : 32'd0;
        4'b1100: R = ~(A | B);        
        default: R = A;      
    endcase
    
    Zero_Flag = (R == 32'd0) ? 1'b1 : 1'b0;
end

endmodule

// ########################
// FETCH CYCLE MODULES #######################

//Program Counter
module PC (
    input clk,
    input [31:0] dirIn,
    output reg [31:0] dirOut
);
    
initial begin
    dirOut = 32'b0;
end

always @(posedge clk) begin
    dirOut <= dirIn;
end
endmodule

//Sumador de direcciones
module Alud(
    input [31:0] dir,
    output reg [31:0] dirOut 
);

always @(*) begin
    dirOut = dir + 4;
end
endmodule

//Memoria de Instrucciones
module Memory(
    input [31:0] dir,
    output reg [31:0] instructionOut
);

reg [7:0] insmem [0:1023];

initial begin
    $readmemb("insmem", insmem);
end

always @(*) begin
    instructionOut = {insmem[dir], insmem[dir+1], insmem[dir+2], insmem[dir+3]};
end
endmodule

// ###################
module SignExtend (
    input [15:0] signIn,
    output reg [31:0] signOut
);

always @(*) begin
    signOut = {{16{signIn[15]}}, signIn};
end
endmodule

//
module ShifLeft(
    input [31:0] shiftIn,
    output reg [31:0] shiftOut
);

always @(*) begin
    shiftOut = shiftIn << 2;
end
endmodule

//
module AddAR(
    input [31:0] addIn1, addIn2,
    output reg [31:0] addOut
);

always @(*) begin
    addOut = addIn1 + addIn2;
end
endmodule

module AndBr(
    input andIn1, andIn2,
    output reg andOut
);

always @(*) begin
    andOut = andIn1 && andIn2;
end
endmodule