module top(clk, reset);
input logic clk, reset;
logic [31:0] instr, imm, npc_sum1, npc_sum2, data_mem_out, rd2_data, rd1_data, alu_out; // 
outputs of all big modules
logic [31:0] rd1_ff_data, rd2_ff_data, pc_ff_out, instr_next, alu_ff_out, data_ff_out; // all ff 
outputs
logic [31:0] pc_mux_out, rd2_mux_out, data_mux_out; // mux outputs
logic [1:0] alu_op; // control signal
logic branch, mem_r, mem_w, mem_reg, alu_b, wr_en, data_w, branch_take, zero;
 logic [3:0] alu_control;
 control control_inst (
.opcode(instr_next[6:0]), .branch(branch), .alu_op(alu_op), .func3(instr_next[14:12]), 
.func7(instr_next[31:25]), .zero(zero),
.mem_r(mem_r), .mem_w(mem_w), .mem_reg(mem_reg), .alu_b(alu_b), 
.wr_en(wr_en), .data_w(data_w), .alu_control(alu_control), .branch_take(branch_take));
 
 adder1 a1(.a(pc_ff_out), .b(32'd4), .result(npc_sum1));
 
 adder1 a2(.a(pc_ff_out), .b(imm), .result(npc_sum2));
 
 flipflop pc2 (.clk(clk), .reset(reset), .en(1'b1), .next(pc_mux_out), .out(pc_ff_out));
 
 flipflop instr1(.clk(clk), .reset(reset), .en(1'b1), .next(instr), .out(instr_next));
 
 flipflop alu_a2(.clk(clk), .reset(reset), .en(1'b1), .next(rd1_data), .out(rd1_ff_data));
 
 flipflop alu_b2(.clk(clk), .reset(reset), .en(1'b1), .next(rd2_data), .out(rd2_ff_data));
 
 flipflop alu_out2(.clk(clk), .reset(reset), .en(1'b1), .next(alu_out), .out(alu_ff_out));
 
 flipflop data_mem (.clk(clk), .reset(reset), .en(1'b1), .next(data_mem_out), .out(data_ff_out));
 
 multiplexer2 pc4(.clk(clk), .a(npc_sum1), .b(npc_sum2), .sel(branch_take), .out(pc_mux_out));
 
 multiplexer2 alu_b1(.clk(clk), .a(rd2_ff_data), .b(imm), .sel(alu_b), .out(rd2_mux_out));
 
 multiplexer2 data_mux(.clk(clk), .a(alu_ff_out), .b(data_ff_out), .sel(data_w), .out(data_mux_out));
 
 instruction_memory instruction_memory_inst (
 .addr(pc_ff_out[7:0]),
 .instruction(instr)
 );
 register register_inst (
 .clk(clk),
 .read1_data(rd1_data),
 .read2_data(rd2_data),
 .input_d(data_mux_out),
 .read1(instr_next[19:15]),
 .read2(instr_next[24:20]),
 .write(instr_next[11:7]),
 .write_en(wr_en) 
 );
 
 imm_generation imm_generation_inst (
 .imm(imm),
 .instr(instr_next)
 );
 
 ALU alu_inst (
 .a(rd1_ff_data),
 .b(rd2_mux_out),
 .result(alu_out),
 .zero(zero),
 .alu_control(alu_control)
 );
 data_Memory data_memory_inst (
 .clk(clk),
 .addr(alu_ff_out[7:0]),
 .data_in(rd2_ff_data),
 .w(mem_w),
 .out(data_mem_out), 
.read(mem_r), 
.func3(instr_next[14:12])
 );
endmodule
module top_testbench();
logic clk, reset;
 // Instantiate the top-level module
 top dut (
.clk, .reset);
 
initial
begin
reset <= 1; #5; reset <= 0;
end
// generate clock to sequence tests
always
begin
clk <= 1; #5; clk <= 0; #5;
end
endmodule




module instruction_memory(addr, instruction);
	input logic [7:0] addr;
	output logic [31:0] instruction;
  logic [31:0] mem [0:255];
  
    initial 
	 begin
	 $readmemh("data.txt", mem); 
  end
  
    assign instruction = mem[addr];



module data_Memory(clk, addr, data_in, w, out, read, func3);
	input logic clk, w, read;
	input logic [31:0] data_in;
	input logic [7:0] addr;
	input logic [2:0] func3;
	output logic [31:0] out;
	
	logic [7:0] mem [0:255];
	
	initial begin
		$readmemh("data_mem.txt", mem);
	end
	
	initial begin
		out = 0;
	end	

	always @(posedge clk) begin
		if (w) begin
			if (func3 == 3'b000) 
				mem[addr] <= data_in[7:0];
			else if (func3 == 3'b001) begin	
				mem[addr] <= data_in[7:0];
				mem[addr - 1] <= data_in[15:8];
			end
			else if (func3 == 3'b010) begin
				mem[addr] <= data_in[7:0];
				mem[addr - 1] <= data_in[15:8];	
				mem[addr - 2] <= data_in[23:16];
				mem[addr - 3] <= data_in[31:24];
			end	
		end
		else if (read) begin
			if (func3 == 3'b000) 
				out[7:0] <= mem[addr]; 
			else if (func3 == 3'b001) begin	
				out[7:0] <= mem[addr];
				out[15:8] <= mem[addr - 1];
			end
			else if (func3 == 3'b010) begin
				out[7:0] <= mem[addr];
				out[15:8] <= mem[addr - 1];
				out[23:16] <= mem[addr - 2];
				out[31:24] <= mem[addr - 3];
			end
		end
		else
			out <= {mem[addr + 3], mem[addr + 2], mem[addr + 1], mem[addr]};
	end	
endmodule 




module register(clk, read1_data, read2_data, read1, read2, write, write_en, input_d);
	input logic clk, write_en;
	input logic [4:0] read1, read2, write;
	input logic [31:0] input_d;
	output logic [31:0] read1_data, read2_data;
	
	logic [31:0] register_arr [31:0];
	
	integer i;
	initial begin
		for (i = 0; i < 32; i = i + 1)
			register_arr[i] = i;
	end	
		
	always @(posedge clk) begin
		if (write_en) begin 
			register_arr[write] = input_d;
		end	
	end
	
	assign read1_data = register_arr[read1];
	assign read2_data = register_arr[read2];
	
endmodule 




module alu_decoder(func7, func3, alu_op, alu_control, opcode);
	input logic [2:0] func3;
	input logic [1:0] alu_op;
	input logic [6:0] opcode, func7;
	output logic [3:0] alu_control;
	
	always_comb begin
		case(alu_op)  // for i type and r type
			2'b00: 
				case(opcode)
					7'b0010111: alu_control = 4'd0; //AUIPC
					7'b1101111: alu_control = 4'd0; // JALR
					7'b1100111: alu_control = 4'd0; // JAL
					7'b0000011: alu_control = 4'd0;
					7'b0100011: alu_control = 4'd0;
					default: alu_control = 4'd0;
				endcase	
			2'b01: 
				case(func3) //branching
						3'b000: alu_control = 4'd1;
						3'b001: alu_control = 4'd1;
						3'b100: alu_control = 4'd2;
						3'b101: alu_control = 4'd2;
						3'b110: alu_control = 4'd3;
						3'b111: alu_control = 4'd3;
						default: alu_control = 4'd1;
				endcase
			2'b10: begin
							case({func7, func3}) // r and i type
							10'b0000000000: alu_control = 4'd0; //ADD
							10'b0100000000:	alu_control = 4'd1; //SUB
							10'b0000000001:	alu_control = 4'd8; //SLL
							10'b0000000010:	alu_control = 4'd2; //SLT
							10'b0000000011:	alu_control = 4'd3; //SLTU
							10'b0000000100:	alu_control = 4'd4; //XOR
							10'b0000000101:	alu_control = 4'd7; //SRL
							10'b0100000101:	alu_control = 4'd9; //SRA
							10'b0000000110:	alu_control = 4'd5; //OR
							10'b0000000111:	alu_control = 4'd6; //AND
							default: alu_control = 4'd0;
						endcase		
			  end
			2'b11: alu_control = 4'd10; //LUI 
			default: alu_control = 4'd0;
		endcase
	end
	
endmodule	




module adder1(a, b, result);
	input logic[31:0] a, b;
	output logic [31:0] result;
	
	assign result = a + b;
endmodule	




module ALU (a, b, result, zero, alu_control);
	input logic [31:0] a, b;
	input logic [3:0] alu_control;
	output logic [31:0] result; 
	output logic zero;
	
  always_comb begin  // R type
		case (alu_control)
					  4'd0: result = a + b;    // ADD
					  4'd1: result = a - b;    // SUB
					  4'd2: result = ($signed(a) < $signed(b)) ? 1 : 0;    // SLT
					  4'd3: result = (a < b) ? 1 : 0;    // SLTU
					  4'd4: result = a ^ b;    // XOR
					  4'd5: result = a | b;    // OR
					  4'd6: result = a & b;   // AND
					  4'd7: result = a >> b[4:0];   // SRL
					  4'd8: result = a << b[4:0];   // SLL
					  4'd9: result = ($signed(a)) >>> b[4:0];   // SRA
					  4'd10: result = b;
					  default: result = 0;            // ADD	
		endcase
	end

   assign zero = (result == 32'd0);
	
endmodule	




module branch_taken (zero_res, func3, branch_take, alu_op, opcode);
	input logic zero_res;
	input logic [2:0] func3;
      input logic [1:0] alu_op;
	input logic [6:0] opcode;
	output logic branch_take;
	
	always_comb begin
	   if (opcode == 7'b1101111 || opcode == 7'b1100111)
		branch_take = 1;
	   else if (alu_op == 2'b01) begin
		case(func3)
			3'b000: branch_take = zero_res;
			3'b001: branch_take = !zero_res;
			3'b100: branch_take = !zero_res;
			3'b101: branch_take = zero_res;
			3'b110: branch_take = !zero_res;
			3'b111: branch_take = zero_res;
			default: branch_take = 1'bx;
		endcase
	   end
	   else
		branch_take = 0;
	end
endmodule	




module main_decode(op, branch, alu_op, mem_r, mem_w, mem_reg, alu_b, wr_en, data_w);
	input logic [6:0] op;
	output logic [1:0] alu_op;
	output logic mem_r, mem_w, mem_reg, alu_b, wr_en, branch, data_w;
	
	
	parameter LW = 7'b0000011;
	parameter SW = 7'b0100011;
	parameter R_type = 7'b0110011;
	parameter B_type = 7'b1100011;
	parameter I_type = 7'b0010011;
	parameter jal = 7'b1101111;
	parameter Jalr = 7'b1100111;
	parameter LUI = 7'b0110111;
	parameter AUIPC = 7'b0010111;
	
	always_comb begin
		case(op)
			R_type: begin
				mem_r = 0;
				branch = 0;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 0;
			end
			I_type: begin
				mem_r = 0;
				branch = 0;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 1;
				wr_en = 1;
				data_w = 0;
				end
			B_type: begin
				mem_r = 0;
				branch = 1;
				alu_op = 2'b01;
				mem_reg = 1'bx;
				mem_w = 0;
				alu_b = 0;
				wr_en = 0;
				data_w = 0;
				end
			LW: begin 
				mem_r = 1;
				branch = 0;
				alu_op = 2'b00;
				mem_reg = 1;
				mem_w = 0;
				alu_b = 1;
				wr_en = 1;
				data_w = 0;
				end
			SW: begin
				mem_r = 0;
				branch = 0;
				alu_op = 2'b00;
				mem_reg = 1'bx;
				mem_w = 1;
				alu_b = 1;
				wr_en = 0;
				data_w = 0;
				end
			jal: begin
				mem_r = 0;
				branch = 1;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 1;
				end
			Jalr: begin 
				mem_r = 0;
				branch = 1;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 1;
				end
			LUI: begin 
				mem_r = 0;
				branch = 0;
				alu_op = 2'b11;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 0;
				end
			AUIPC: begin
				mem_r = 0;
				branch = 0;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 0;
				end
			default: begin
				mem_r = 1'bx;
				branch = 1'bx;
				alu_op = 2'bx;
				mem_reg = 1'bx;
				mem_w = 1'bx;
				alu_b = 1'bx;
				wr_en = 1'bx;
				data_w = 1'bx;
				end	
		endcase
	end

endmodule 





module control(opcode, branch, branch_take, alu_op, func3, func7, zero, mem_r, mem_w, mem_reg, alu_b, wr_en, data_w, alu_control), data_w;
	input logic [6:0] opcode, func7;
	input logic zero;
	inp [2:0] func3;
	output logic alu_b;
	output logic [1:0] alu_op, data_w;
	output logic mem_r, mem_w, mem_reg, wr_en, branch, branch_take;
	output logic [3:0] alu_control;
		
	main_decode m1(.op(opcode), .branch(branch), .alu_op(alu_op), .mem_r(mem_r), .mem_w(mem_w), .mem_reg(mem_reg), .alu_b(alu_b), .wr_en(wr_en), .data_w(data_w));
	
	alu_decoder a1(.func7(func7), .func3(func3), .alu_op(alu_op), .opcode(opcode), .alu_control(alu_control));
	
	branch_taken b1(.zero_res(zero), .func3(func3), .branch_take(branch_take), .alu_op(alu_op), .opcode(opcode));
	
endmodule




module imm_generation(imm, instr);
	input logic [31:0] instr; 
	output logic [31:0] imm;
	
   parameter LW = 7'b0000011;
	parameter SW = 7'b0100011;
	parameter R_type = 7'b0110011;
	parameter B_type = 7'b1100011;
	parameter I_type = 7'b0010011;
	parameter jal = 7'b1101111;
	parameter Jalr = 7'b1100111;
	parameter LUI = 7'b0110111;
	parameter AUIPC = 7'b0010111;
	
	always_comb begin // try and add u format seperately
		case(instr[6:0])
			I_type: // I type
				imm = {{21{instr[31]}}, instr[30:25], instr[24:21], instr[20]};
			LW: 
				imm = {{21{instr[31]}}, instr[30:25], instr[24:21], instr[20]};
			Jalr:
				imm = {{21{instr[31]}}, instr[30:25], instr[24:21], instr[20]};
			SW:	// S type
				imm = {{21{instr[31]}}, instr[30:25], instr[11:8], instr[7]}; 
			B_type:	// B type
				imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
			jal:	// J type
				imm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:25], instr[24:21], 1'b0};
			LUI:	// U type
				imm = {{1{instr[31]}}, instr[30:20], instr[19:12], 12'b0};
			AUIPC:
				imm = {{1{instr[31]}}, instr[30:20], instr[19:12], 12'b0};
			default: begin imm = {{21{instr[31]}}, instr[30:25], instr[24:21], instr[20]}; // I type
			end
		endcase	
	end	
endmodule 





module flipflop (clk, reset, en, next, out);
	input logic clk, reset, en;
	input logic [31:0] next;
	output logic [31:0] out;
	
	always_ff @(posedge clk) begin
		if (reset) 
			out <= 0;
		else if (en)
			out <= next;
	end
endmodule




module multiplexer2(clk, a, b, sel, out);
	input logic clk;
	input logic [31:0] a, b;
	input logic sel;
	output logic [31:0] out;
	
	assign out = sel ? b : a;
	
endmodule	

