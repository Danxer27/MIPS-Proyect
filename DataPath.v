
//Daniel Joel Corona Espinoza
//Yahir Efren Borboa Quintero
module DataPath(
    input CLK,
    output [31:0] DS
);

//Cables
wire [31:0] instruction;

wire cDemux, cWe, cRe, cZf, cBRWe, cBranch, cRegDst, cAddPcMuxo;
wire [2:0] cALU_op;
wire cData_Out;
wire [3:0] C4_Sel;
wire [31:0] C1_RD1, C2_RD2, C3_AluRes, cDW, CS;

wire clock;
wire [31:0] pcIn, pcOut, sumOut, istruction, cSignex, cAddMuxo, cAddPc;

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
    .addIn1(cSignex),
    .addIn2(sumOut),
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
    .Branch(cBranch)
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
    .data1Out(C2_RD2), 
    .data2Out(C1_RD1)
);

ALU Alulu(
    .A(C1_RD1),
    .B(C2_RD2),
    .ALU_OP(C4_Sel),
    //Ouputs 
    .R(C3_AluRes),
    .Zero_Flag(cZf)
);

MemoryData MemData(
    .WEn(cWe),
    .REn(cRe),
    .dataIn(C1_RD1),
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

// always @(posedge CLK) begin
//     DS <= cData_Out;
// end
assign DS = cDW;

endmodule

//Controlador
module Controller(
    input [5:0] Op,
    output reg Demuxo, WeMD, ReMD, BRWe, regDst, Branch,
    output reg [2:0] ALU_op
);

always @(*) begin
    case (Op)
        6'b000000: begin       
            ALU_op = 3'b010;
            Demuxo = 1'b1;
            BRWe = 1'b1;   
            WeMD = 1'b0;   
            ReMD = 1'b0;
            regDst = 1'b1;
            Branch = 1'b0;
        end

        6'b000100: begin       
            ALU_op = 3'b001;
            Demuxo = 1'b1;
            BRWe = 1'b1;   
            WeMD = 1'b1;   
            ReMD = 1'b1;
            regDst = 1'b1;
            Branch = 1'b1;
        end

         6'b000010: begin       
            ALU_op = 3'b010;
            Demuxo = 1'b1;
            BRWe = 1'b1;   
            WeMD = 1'b1;   
            ReMD = 1'b1;
            regDst = 1'b1;
            Branch = 1'b1;
        end
        
        //Aqui van tipo I y J cuando se necesitan
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
            6'b101000: sel = 4'b0000;
            6'b100101: sel = 4'b0001;
            6'b101010: sel = 4'b0111;
            endcase
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
    if(WE) begin
        mem[WA] = DW;
    end
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
    output reg [31:0] dataOut
);

reg [31:0] mem [0:127];

always @ (*) begin
    if(WEn) begin
        mem[addres] = dataIn;
    end
    if(REn) begin
        dataOut = mem[addres];
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