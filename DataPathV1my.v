//Daniel Joel Corona Espinoza
//Yahir Efren Borboa Quintero
module DataPath(
    input CLK,
    output [31:0] DS
);

//Cables
wire [3:0] C4_Sel;
wire cPCSrc;
wire [31:0] pcOut, cAddPc, cShiftAdd, cAluMux;

// CABLES BUFFERS

// IF Buffer Conections
wire [31:0] cIF_fetchAdder, cIF_Instruction_Memory; 
// ID Buffer Conections
wire [31:0] cID_fetchAdder, cID_Instruction_Memory, cID_readData1, cID_readData2, cID_signExtend;
wire [4:0] cID_ins1, cID_ins2;
wire cID_WB, cID_M, cID_EX1, cID_EX2, cID_memWrite, cID_memRead, cID_memToReg;
wire [2:0] cID_AluOp;
// EX Buffer Conections
wire cEX_WB, cEX_M, cEX_RegDst, cEX_AluSrc, cEX_memWrite, cEX_memRead, cEX_memToReg; 
wire [2:0] cEX_AluOp;
wire [4:0] cEX_bmux1, cEX_bmux2;
wire [31:0]  cEX_EX, cEX_fetchAdder, cEX_rData1, cEX_rData2, cEX_signExtend;
    // Salidas de EX -> Mem
wire cEX_ZF;
wire [31:0] cEX_fetchAdderRes, cEX_AluResult, cEX_memWriteData;
wire [4:0] cEX_bancMuxResult;
// MEM Buffer Conections -> Entradas a mem
wire cMEM_WB, cMEM_M, cMEM_ZF, cMEM_memWrite, cMEM_memRead, cMEM_memToReg;
wire [4:0] cMEM_bancMuxResult;
wire [31:0] cMEM_memAdress, cMEM_memWriteData, cMEM_readData, cMEM_addPcMux;
// WB Buffer conections
wire cWB_memToReg, cWB_RegWrite;
wire [31:0] cWB_memMux1, cWB_memMux2, cWB_BancWriteData;
wire [4:0] cWB_WR;

wire cID_Jump, cEX_Jump, cMEM_Jump;                   
wire [31:0] cID_JumpAddr, cEX_JumpAddr, cMEM_JumpAddr, cPcAdd, cPCnext;


//Instancias

PC pc(
    .clk(CLK),
    .dirIn(cPCnext),
    .dirOut(pcOut)
);

Alud sum(
    .dir(pcOut),
    .dirOut(cIF_fetchAdder)
);

Memory mem(
    .dir(pcOut),
    .instructionOut(cIF_Instruction_Memory)
);

Mux AddPCMuxo(
    .dmx(cPCSrc),
    .data1In(cIF_fetchAdder),
    .data2In(cMEM_addPcMux),
    //output
    .dataOut(cPcAdd)
);

JumpAddress jump_calc (
    .PC_plus_4(cID_fetchAdder),      
    .instr_index(cID_Instruction_Memory[25:0]), 
    .jump_addr(cID_JumpAddr)
);

MuxJump pc_mux (
    .PC_plus_4(cPcAd),      
    .jump_addr(cMEM_JumpAddr),
    .Jump(cJump),                   
    .next_PC(cPCnext) 
);


// #### BUFFER 1 #####

IFID Buffer1(
    //Inputs
    .clk(CLK),
    .inFCAdd(cIF_fetchAdder),
    .inMem(cIF_Instruction_Memory),
    //outputs
    .outFCAdd(cID_fetchAdder),
    .outInsMem(cID_Instruction_Memory)
);

    // BUFFER 1 EXIT

Controller Control(//Inputs
    .Op(cID_Instruction_Memory[31:26]), 
    //Ouputs
    .Demuxo(cID_memToReg), 
    .WeMD(cID_memWrite), 
    .ReMD(cID_memRead),
    .BRWe(cWB_RegWrite),
    .ALU_op(cID_AluOp),
    .regDst(cID_EX1),
    .Branch(cID_M),
    .aluSrc(cID_EX2),
    .jump(cID_Jump)
);

BancRegister Banco( //Inputs
    .RA1(cID_Instruction_Memory[25:21]), 
    .RA2(cID_Instruction_Memory[20:16]), 
    .WA(cWB_WR),
    .DW(cWB_BancWriteData),
    .WE(cWB_RegWrite),
    .CLK(CLK),
    //Outputs
    .data1Out(cID_readData1), 
    .data2Out(cID_readData2)
);

SignExtend Signex(
    .signIn(cID_Instruction_Memory[15:0]),
    //output
    .signOut(cID_signExtend)
);


// #### BUFFER 2 #####

IDEX Buffer2( //inputs
    .clk(CLK),
    .WBIn(cID_WB),
    .Min(cID_M),
    .EX1(cID_EX1),
    .EX2(cID_EX2),
    .ExAluOp(cID_AluOp),
    .memWriteEnIn(cID_memWrite),
    .memReadEnIn(cID_memRead),
    .AddResultIn(cID_fetchAdder),
    .readData1In(cID_readData1),
    .readData2In(cID_readData2),
    .signExtIn(cID_signExtend),
    .instructionMux1In(cID_Instruction_Memory[20:16]),
    .instructionMux2In(cID_Instruction_Memory[15:11]),
    .memToRegIn(cID_memToReg),
    .jumpAdd_in(cID_JumpAddr),
    .jumpIn(cID_Jump),
    //outputs
    .WBout(cEX_WB),
    .Mout(cEX_M),
    .AluOp(cEX_AluOp),
    .RegDst(cEX_RegDst),
    .AluSrc(cEX_AluSrc),
    .memReadEnOut(cEX_memRead),
    .memWriteEnOut(cEX_memWrite),
    .AddResultOut(cEX_fetchAdder),
    .readData1Out(cEX_rData1),
    .readData2Out(cEX_rData2),
    .signExtOut(cEX_signExtend),
    .instructionMux1Out(cEX_bmux1),
    .instructionMux2out(cEX_bmux2),
    .memToRegOut(cEX_memToReg),
    .jumpAdd_out(cEX_JumpAddr),
    .jumpOut(cEX_Jump)
); 

    // BUFFER 2 EXIT

ShifLeft SL(
    .shiftIn(cEX_signExtend),
    .shiftOut(cShiftAdd)
);

AddAR ALuAdd(
    .addIn1(cEX_fetchAdder),
    .addIn2(cShiftAdd),
    //output
    .addOut(cEX_fetchAdderRes)
);

Mux Muxalu (
    .dmx(cEX_AluSrc),
    .data1In(cEX_rData2),
    .data2In(cEX_signExtend),
    //output:
    .dataOut(cAluMux)
);

ALU Alulu(
    .A(cEX_rData1),
    .B(cAluMux),
    .ALU_OP(C4_Sel),
    //Ouputs 
    .R(cEX_AluResult),
    .Zero_Flag(cEX_ZF)
);

Alu_Control AluController(
    .funct(cEX_signExtend[5:0]),
    .ALUop(cEX_AluOp),
    //Output:
    .sel(C4_Sel)
);

MuxBanc BancMux(
    .dmx(cEX_RegDst),
    .data1In(cEX_bmux1),
    .data2In(cEX_bmux2),
    //output
    .dataOut(cEX_bancMuxResult)
);

 // #### BUFFER 3 #####

EXMEM Buffer3( //inputs
    .clk(CLK),
    .WBin(cEX_WB),
    .Min(cEX_M),
    .ZFin(cEX_ZF),
    .memWriteEnIn(cEX_memWrite),
    .memReadEnIn(cEX_memRead),
    .memToRegIn(cEX_memToReg),
    .AddResIn(cEX_fetchAdderRes),
    .AluResultIn(cEX_AluResult),
    .readData2In(cEX_rData2),
    .BancMuxIn(cEX_bancMuxResult),
    .jumpAdd_in(cEX_JumpAddr),
    .jumpIn(cEX_Jump),
    //outputs
    .WBout(cMEM_WB),
    .Mout(cMEM_M),
    .ZFout(cMEM_ZF),
    .memWriteEnOut(cMEM_memWrite),
    .memReadEnOut(cMEM_memRead),
    .memToRegOut(cMEM_memToReg),
    .memAddress(cMEM_memAdress),
    .memWData(cMEM_memWriteData),
    .AddResOut(cMEM_addPcMux),
    .BancMuxOut(cMEM_bancMuxResult),
    .jumpAdd_out(cMEM_JumpAddr),
    .jumpOut(cMEM_Jump)
);

    // BUFFER 3 EXIT

AndBr cand(
    .andIn1(cMEM_M),
    .andIn2(cMEM_ZF),
    //output
    .andOut(cPCSrc)
);

MemoryData MemData(
    .clk(CLK),
    .WEn(cMEM_memWrite),
    .REn(cMEM_memRead),
    .addres(cMEM_memAdress),
    .dataIn(cMEM_memWriteData),
    //Output
    .dataOut(cMEM_readData)
);


// #### BUFFER 4 #####
MEMWB Buffer4(
    .clk(CLK),
    .WBin(cMEM_WB),
    .memToRegIn(cMEM_memToReg),
    .readDataIn(cMEM_readData),
    .aluDataIn(cMEM_memAdress),
    .bancMuxIn(cMEM_bancMuxResult),
    //output
    .WBout(cWB_RegWrite),
    .memToRegOut(cWB_memToReg),
    .mux1out(cWB_memMux1),
    .mux2out(cWB_memMux2),
    .bancMuxOut(cWB_WR)
);

    // BUFFER 4 EXIR
Demuxo Demux(
    .dmx(cWB_memToReg),
    .dataMemIn(cWB_memMux1),
    .dataAluIn(cWB_memMux2),
    //OutPut:
    .dataOut(cWB_BancWriteData)
);

assign DS = cWB_BancWriteData;

//############# MODULOS #################

endmodule

//Controlador
module Controller(
    input [5:0] Op,
    output reg Demuxo, WeMD, ReMD, BRWe, regDst, Branch, aluSrc, jump,
    output reg [2:0] ALU_op
);

always @(*) begin
    case (Op)
        6'b000000: begin   // R     
            ALU_op = 3'b010;
            Demuxo = 1'b1;
            BRWe = 1'b1;   
            WeMD = 1'b0;   
            ReMD = 1'b0;
            regDst = 1'b1;
            Branch = 1'b0;
            aluSrc = 1'b0;
            jump = 1'b0;
        end

        6'b000100: begin  //  BEQ
            ALU_op = 3'b001;
            Demuxo = 1'b0;
            BRWe = 1'b0;   
            WeMD = 1'b0;   
            ReMD = 1'b0;
            regDst = 1'b0;
            Branch = 1'b1;
            aluSrc = 1'b0;
            jump = 1'b0;
        end

       6'b001101: begin  // ORI 
            ALU_op = 3'b100;
            Demuxo = 1'b1;
            BRWe = 1'b1;   
            WeMD = 1'b0;   
            ReMD = 1'b0;
            regDst = 1'b0;
            Branch = 1'b0;
            aluSrc = 1'b1;
            jump = 1'b0;
        end
        
        6'b001000: begin  // ADDI 
            ALU_op = 3'b010; 
            Demuxo = 1'b1;
            BRWe = 1'b1;
            WeMD = 1'b0;
            ReMD = 1'b0;
            regDst = 1'b0;
            Branch = 1'b0;
            aluSrc = 1'b1;
            jump = 1'b0;
        end

        6'b001100: begin  // ANDI 
            ALU_op = 3'b101;
            Demuxo = 1'b1;
            BRWe = 1'b1;
            WeMD = 1'b0;
            ReMD = 1'b0;
            regDst = 1'b0;
            Branch = 1'b0;
            aluSrc = 1'b1;
            jump = 1'b0;
        end

        6'b101011: begin  // SW 
            ALU_op = 3'b010; 
            Demuxo = 1'bx;
            BRWe = 1'b0;
            WeMD = 1'b1;
            ReMD = 1'b0;
            regDst = 1'bx;
            Branch = 1'b0;
            aluSrc = 1'b1;
            jump = 1'b0;
        end

        6'b100011: begin  // LW 
            ALU_op = 3'b010;  
            Demuxo = 1'b0;
            BRWe = 1'b1;
            WeMD = 1'b0;
            ReMD = 1'b1;
            regDst = 1'b0;
            Branch = 1'b0;
            aluSrc = 1'b1;
            jump = 1'b0;
        end

        6'b001010: begin // SLTI
            ALU_op = 3'b011;  
            regDst = 1'b0;
            Demuxo = 1'b1;
            BRWe = 1'b1;
            ReMD = 1'b0;
            WeMD = 1'b0;
            Branch = 1'b0;
            aluSrc = 1'b1;
            jump = 1'b0;
        end

        6'b000010: begin // J
            ALU_op = 3'b000;  
            regDst = 1'b0;
            Demuxo = 1'b0;
            BRWe = 1'b0;
            ReMD = 1'b0;
            WeMD = 1'b0;
            Branch = 1'b0;
            aluSrc = 1'b0;
            jump = 1'b1;
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
            sel = 4'b0010;
        3'b001:
            sel = 4'b0110;
        3'b010:
            case (funct) 
            6'b100000: sel = 4'b0010;
            6'b100010: sel = 4'b0110;
            6'b100100: sel = 4'b0000;
            6'b100101: sel = 4'b0001;
            6'b101010: sel = 4'b0111;
            default:   sel = 4'b0010;
            endcase
        3'b011: sel = 4'b0111; // SLT
        3'b100: sel = 4'b0001; // OR
        3'b101: sel = 4'b0000; // AND
    endcase
end
endmodule

//Banco de Registros
module BancRegister( 
    input [4:0] RA1, RA2, WA,
    input [31:0] DW,
    input WE,
    input CLK,
    output reg [31:0] data1Out, data2Out
);

reg [31:0] mem [0:31];

initial begin
    $readmemb("data", mem);
end

always @(posedge CLK) begin
    if(WE) begin
        mem[WA] <= DW;
    end
end

always @(*) begin
    data1Out = mem[RA1];
    data2Out = mem[RA2];
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
    output reg [31:0] dataOut //outAlu, outMem    
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
    input clk,
    output reg [31:0] dataOut
);

reg [31:0] mem [0:127];

always @(posedge clk) begin
    if(WEn) begin
        mem[addres] <= dataIn;
    end
end

always @(*) begin
    if(REn) begin
        dataOut = mem[addres];
    end else begin
        dataOut = 32'b0;
    end
end
endmodule

//Alu
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
// FETCH CICLE MODULES #######################

//Direccionaor
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

//Memoria
module Memory(
    input [31:0] dir,
    output reg [31:0] instructionOut
);

reg [7:0] insmem [0:1023];

initial begin
    $readmemb("insmem", insmem);
end

always @ (*) begin
    instructionOut = {insmem[dir], insmem[dir+1], insmem[dir+2], insmem[dir+3]};
end
endmodule

// ###################
module SignExtend (
    input [15:0] signIn,
    output reg [31:0] signOut
);

always @ (*) begin
    signOut = {{16{signIn[15]}}, signIn};
end
endmodule

//
module ShifLeft(
    input [31:0] shiftIn,
    output reg [31:0] shiftOut
);

always @ (*) begin
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

always @ (*) begin
    if(andIn1 && andIn2) begin
        andOut = 1'b1;
    end else begin
         andOut = 1'b0;
    end
end
endmodule

// TIPO J

module JumpAddress (
    input [31:0] PC_plus_4,
    input [25:0] instr_index,
    output reg [31:0] jump_addr
);
always @(*) begin
    jump_addr = {PC_plus_4[31:28], instr_index, 2'b00};
end
endmodule

module MuxJump (
    input [31:0] PC_plus_4, jump_addr,          
    input Jump,                      
    output reg[31:0] next_PC            
);
always @(*) begin
    next_PC = Jump ? jump_addr : PC_plus_4; 
end
endmodule

// module ShiftLeft (
//     input [25:0] in,     
//     output [27:0] out      
// );
// always @(*) begin
//     out = in << 2;
// end
// endmodule


// ###### BUFFERS ###################

    //BUFFER IF/ID
module IFID(
    input clk,
    input [31:0] inFCAdd, inMem,
    output reg [31:0] outFCAdd, outInsMem
);

always @(posedge clk) begin
    outFCAdd <= inFCAdd;
    outInsMem <= inMem;
end
endmodule

 // BUFFER ID/EX
module IDEX(
    input clk, WBIn, Min, memWriteEnIn, memReadEnIn, EX1, EX2, memToRegIn, jumpIn,
    input [2:0] ExAluOp,
    input [31:0] AddResultIn, readData1In, readData2In, signExtIn, jumpAdd_in,
    input [4:0] instructionMux1In, instructionMux2In,
    output reg WBout, Mout, memWriteEnOut, memReadEnOut, memToRegOut, RegDst, AluSrc, jumpOut,
    output reg [2:0] AluOp, 
    output reg [31:0] AddResultOut, readData1Out, readData2Out, signExtOut, jumpAdd_out,
    output reg [4:0] instructionMux1Out, instructionMux2out
);

always @(posedge clk) begin
    WBout <= WBIn;
    Mout <= Min;
    AluOp <= ExAluOp;
    RegDst <= EX1;
    AluSrc <= EX2;
    AddResultOut <= AddResultIn;
    readData1Out <= readData1In;
    readData2Out <= readData2In;
    signExtOut <= signExtIn;
    instructionMux1Out <= instructionMux1In;
    instructionMux2out <= instructionMux2In;
    memWriteEnOut <= memWriteEnIn;
    memReadEnOut <= memReadEnIn;
    memToRegOut <= memToRegIn;
    jumpAdd_out <= jumpAdd_in;
    jumpOut <= jumpIn;
end
endmodule

    // BUFFER EX/MEM
module EXMEM(
    input WBin, Min, ZFin, clk, memWriteEnIn, memReadEnIn, memToRegIn, jumpIn,
    input [31:0] AluResultIn, readData2In, AddResIn, jumpAdd_in,
    input [4:0] BancMuxIn,
    output reg WBout, Mout, ZFout, memWriteEnOut, memReadEnOut, memToRegOut, jumpOut,
    output reg [31:0] memAddress, memWData, AddResOut, jumpAdd_out,
    output reg [4:0] BancMuxOut
);

always @(posedge clk) begin
    WBout <= WBin;
    Mout <= Min;
    AddResOut <= AddResIn;
    ZFout <= ZFin;
    memAddress <= AluResultIn;
    memWData <= readData2In;
    BancMuxOut <= BancMuxIn;
    memWriteEnOut <= memWriteEnIn;
    memReadEnOut <= memReadEnIn;
    memToRegOut <= memToRegIn;
    jumpAdd_out <= jumpAdd_in;
    jumpOut <= jumpIn;
end
endmodule

    // BUFFER MEM/WB
module MEMWB(
    input WBin, clk, memToRegIn,
    input [31:0] readDataIn, aluDataIn,
    input [4:0] bancMuxIn,
    output reg WBout, memToRegOut,
    output reg [31:0] mux1out, mux2out, 
    output reg [4:0] bancMuxOut
);

always @(posedge clk) begin
    WBout <= WBin;
    mux1out <= readDataIn;
    mux2out <= aluDataIn;
    bancMuxOut <= bancMuxIn;
    memToRegOut <= memToRegIn;
end
endmodule