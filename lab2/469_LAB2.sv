module top(clk, reset);
	input logic clk, reset;
	logic [31:0] instr, imm, npc_sum1, npc_sum2, data_mem_out, rd2_data, rd1_data, alu_out; // outputs of all  big modules
	logic [31:0] pc_mux_out, rd2_mux_out, pc_ff_out; // mux outputs
	logic [1:0] alu_op, data_w; // control signal
	logic branch, mem_r, mem_w, mem_reg, alu_b, wr_en, branch_take, zero;
   logic [3:0]	alu_control;
	logic [31:0]  instr_D, pc_D, pc_4_D, pc_4_ex, pc_4_m, pc_4_w; // IF_ID signals
	logic [31:0] rd1_ex, rd2_ex, pc_ex, imm_ex; // ID_EX
   logic  branch_ex, mem_r_ex, mem_w_ex, mem_reg_ex, alu_b_ex, wr_en_ex; // ID_EX
   logic [1:0] alu_op_ex, data_w_m, data_w_w; // ID_EX
	wire [1:0] data_w_ex;
   // MEM_EX
	logic [4:0] rs1_ex, rs2_ex, rd_ex;
	logic wr_en_m, mem_r_m, mem_w_m; // MEM_EX
	logic [31:0] alu_res_m, rd2_m; // MEM_EX
	logic wr_en_w; // WB_MEM
	logic [31:0] alu_res_w, data_out_w, add_sum_m, mux_3_a, mux_3_b, mux_3_c; // WB_MEM
	logic [4:0] rd_m, rd_w;
	logic [1:0] forward_a_ex, forward_b_ex;
	logic stall, flush_d, flush_ex;
	
	
		hazard h1(.rs1_ex(rs1_ex), .rd_m(rd_m), .mem_reg_m(wr_en_m), .mem_reg_w(wr_en_w), .rs2_ex(rs2_ex), .rd_w(rd_w), .branch_take(branch_take), 
		.data_w_ex(data_w_ex), .rs1_d(instr_D[19:15]), .rs2_d(instr_D[24:20]), .rd_ex(rd_ex), .stall(stall), .flush_d(flush_d), .flush_ex(flush_ex), 
		.forward_a_ex(forward_a_ex), .forward_b_ex(forward_b_ex));

	  control control_inst (
	  .opcode(instr_D[6:0]), .branch(branch), .alu_op(alu_op), .func3(instr_D[14:12]), .func7(instr_D[31:25]), .zero(zero),
	  .mem_r(mem_r), .mem_w(mem_w), .mem_reg(mem_reg), .alu_b(alu_b), 
	  .wr_en(wr_en), .data_w(data_w), .alu_control(alu_control), .branch_take(branch_take));
	 
	  flipflop pc2 (.clk(clk), .reset(reset), .en(stall), .next(pc_mux_out), .out(pc_ff_out));
	  
	  adder1 a1(.a(pc_ff_out), .b(32'd4), .result(npc_sum1));
	  
	  adder1 a2(.a(pc_ex), .b(imm_ex), .result(npc_sum2));
	  
	  mux_3 alu_a_1(.a(rd1_ex), .b(mux_3_c), .c(alu_res_m), .sel(forward_a_ex), .out1(mux_3_a));
	  
	  mux_3 alu_b_1(.a(rd2_ex), .b(mux_3_c), .c(alu_res_m), .sel(forward_b_ex), .out1(mux_3_b));
	  
	  IF_ID  i1(.clk(clk), .reset(reset), .pc_F(pc_ff_out), .instr_F(instr), .instr_D(instr_D), .pc_D(pc_D), .en(stall), .branch_taken(flush_d), .pc_4_F(npc_sum1), .pc_4_D(pc_4_D));
	  
	  ID_EX  i2(.clk(clk), .reset(reset), .branch(branch), .alu_op(alu_op), .mem_r(mem_r), .mem_w(mem_w), .mem_reg(mem_reg), .alu_b(alu_b), 
	  .wr_en(wr_en), .data_w(data_w), .rd1(rd1_data), .rd2(rd2_data), .pc(pc_D), .imm(imm), .rd1_ex(rd1_ex), .rd2_ex(rd2_ex), .pc_ex(pc_ex), .pc_4_d(pc_4_D), .pc_4_ex(pc_4_ex),
	  .imm_ex(imm_ex), .branch_ex(branch_ex), .alu_op_ex(alu_op_ex), .mem_r_ex(mem_r_ex), .mem_w_ex(mem_w_ex), .mem_reg_ex(mem_reg_ex), .alu_b_ex(alu_b_ex), .flush(flush_ex),
	  .wr_en_ex(wr_en_ex), .data_w_ex(data_w_ex), .rs1_d(instr_D[19:15]), .rs2_d(instr_D[24:20]), .rd_d(instr_D[11:7]), .rs1_ex(rs1_ex), .rs2_ex(rs2_ex), .rd_ex(rd_ex));
	  
	  MEM_EX m1(.clk(clk), .reset(reset), .data_w_ex(data_w_ex), .wr_en_ex(wr_en_ex), .mem_r_ex(mem_r_ex), .mem_w_ex(mem_w_ex), .alu_res(alu_out), .rd2_ex(rd2_ex), .pc_4_ex(pc_4_ex), .pc_4_m(pc_4_m),
	  .data_w_m(data_w_m), .wr_en_m(wr_en_m), .mem_r_m(mem_r_m), .mem_w_m(mem_w_m), .alu_res_m(alu_res_m), .rd2_m(rd2_m), .rd_m(rd_m), .rd_ex(rd_ex));
	  
	  WB_MEM w1(.clk(clk), .reset(reset), .data_w_m(data_w_m), .wr_en_m(wr_en_m), .alu_res_m(alu_res_m), .data_out(data_mem_out), .data_w_w(data_w_w),
	  .wr_en_w(wr_en_w), .alu_res_w(alu_res_w), .data_out_w(data_out_w), .rd_m(rd_m), .rd_w(rd_w), .pc_4_m(pc_4_m), .pc_4_w(pc_4_w));
	  
	  multiplexer2 pc4(.clk(clk), .a(npc_sum1), .b(npc_sum2), .sel(branch_take), .out(pc_mux_out));
	  
	  multiplexer2 alu_b1(.clk(clk), .a(mux_3_b), .b(imm_ex), .sel(alu_b_ex), .out(rd2_mux_out));
	  
	  mux_3 data_mux(.a(alu_res_w), .b(data_out_w), .c(pc_4_w), .sel(data_w_w), .out1(mux_3_c));
	  
	  instruction_memory instruction_memory_inst (
		 .addr(pc_ff_out[7:0]),
		 .instruction(instr)
	  );

	  register register_inst (
		 .clk(clk),
		 .read1_data(rd1_data),
		 .read2_data(rd2_data),
		 .input_d(mux_3_c),
		 .read1(instr_D[19:15]),
		 .read2(instr_D[24:20]),
		 .write(instr_D[11:7]),
		 .write_en(wr_en) 
	  );
	  
	  imm_generation imm_generation_inst (
		 .imm(imm),
		 .instr(instr_D)
	  );
	  
	  ALU alu_inst (
		 .a(mux_3_a),
		 .b(rd2_mux_out),
		 .result(alu_out),
		 .zero(zero),
		 .alu_control(alu_control)
	  );
	  data_Memory data_memory_inst (
		 .clk(clk),
		 .addr(alu_res_m[7:0]),
		 .data_in(rd2_m),
		 .w(mem_w),
		 .out(data_mem_out), 
		 .read(mem_r_m), 
		 .func3(instr_D[14:12])
	  );
endmodule

module top_testbench();
	 logic clk, reset;
  // Instantiate the top-level module
  top dut (
.clk, .reset);
  
 initial
 begin
 reset = 1; #5; reset = 0;
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
	output logic [1:0] alu_op, data_w;
	output logic mem_r, mem_w, mem_reg, alu_b, wr_en, branch;
	
	
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
				data_w = 2'b00;
			end
			I_type: begin
				mem_r = 0;
				branch = 0;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 1;
				wr_en = 1;
				data_w = 2'b00;
				end
			B_type: begin
				mem_r = 0;
				branch = 1;
				alu_op = 2'b01;
				mem_reg = 1'bx;
				mem_w = 0;
				alu_b = 0;
				wr_en = 0;
				data_w = 2'b00;
				end
			LW: begin 
				mem_r = 1;
				branch = 0;
				alu_op = 2'b00;
				mem_reg = 1;
				mem_w = 0;
				alu_b = 1;
				wr_en = 1;
				data_w = 2'b01;
				end
			SW: begin
				mem_r = 0;
				branch = 0;
				alu_op = 2'b00;
				mem_reg = 1'bx;
				mem_w = 1;
				alu_b = 1;
				wr_en = 0;
				data_w = 2'b00;
				end
			jal: begin
				mem_r = 0;
				branch = 1;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 2'b10;
				end
			Jalr: begin 
				mem_r = 0;
				branch = 1;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 2'b10;
				end
			LUI: begin 
				mem_r = 0;
				branch = 0;
				alu_op = 2'b11;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 2'b00;
				end
			AUIPC: begin
				mem_r = 0;
				branch = 0;
				alu_op = 2'b10;
				mem_reg = 0;
				mem_w = 0;
				alu_b = 0;
				wr_en = 1;
				data_w = 2'b00;
				end
			default: begin
				mem_r = 1'bx;
				branch = 1'bx;
				alu_op = 2'bx;
				mem_reg = 1'bx;
				mem_w = 1'bx;
				alu_b = 1'bx;
				wr_en = 1'bx;
				data_w = 2'bx;
				end	
		endcase
	end

endmodule 




module control(opcode, branch, branch_take, alu_op, func3, func7, zero, mem_r, mem_w, mem_reg, alu_b, wr_en, data_w, alu_control);
	input logic [6:0] opcode, func7;
	input logic zero;
	input logic [2:0] func3;
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





module hazard(rs1_ex, rd_m, mem_reg_m, mem_reg_w, rs2_ex, rd_w, branch_take, data_w_ex, rs1_d, rs2_d, rd_ex, stall, flush_d, flush_ex, forward_a_ex, forward_b_ex);
	input logic mem_reg_m, mem_reg_w, branch_take;
	input logic [1:0] data_w_ex;
	input logic [4:0] rd_m, rs1_ex, rs2_ex, rd_w, rs1_d, rs2_d, rd_ex;
	output logic [1:0] forward_a_ex, forward_b_ex;
	output logic stall, flush_d, flush_ex;
	
	always @(*) begin
		if (((rs1_ex == rd_m) & mem_reg_m) & (rs1_ex != 0))
			forward_a_ex = 2'b10;
		else if (((rs1_ex == rd_w) & mem_reg_w) & (rs1_ex != 0))
			forward_a_ex = 2'b01;
		else 
			forward_a_ex = 2'b00;
	end
	
	always @(*) begin
		if (((rs2_ex == rd_m) & mem_reg_m) & (rs2_ex != 0))
			forward_b_ex = 2'b10;
		else if (((rs2_ex == rd_w) & mem_reg_w) & (rs2_ex != 0))
			forward_b_ex = 2'b01;
		else 
			forward_b_ex = 2'b00;
	end
	
	assign stall = (data_w_ex == 1) & ((rs1_d == rd_ex) | (rs2_d == rd_ex) & branch_take);
	assign flush_d = branch_take;
	assign flush_ex = branch_take | stall;
endmodule 




// Instruction Fetch Unit
module IF_ID(clk, reset, pc_F, instr_F, instr_D, pc_D, branch_taken, en, pc_4_F, pc_4_D);
	input logic clk;
	input logic reset, en, branch_taken;
	input logic [31:0] pc_F, instr_F, pc_4_F; // input program counter
	output logic [31:0] instr_D; // output instruction
	output logic [31:0] pc_D, pc_4_D;
	always @(posedge clk) begin
		if (reset || branch_taken) begin
			instr_D <= 0;
			pc_D <= 0;
			pc_4_D <= 0;
		end
		else if (!en) begin
			pc_D <= pc_F;
			instr_D <= instr_F;
			pc_4_D <= pc_4_F;
		end
	end	
endmodule




module ID_EX(clk, reset, branch, alu_op, mem_r, mem_w, mem_reg, alu_b, wr_en, data_w, rd1, rd2, rd1_ex,
rd2_ex, pc, pc_ex, imm, imm_ex, branch_ex, alu_op_ex, mem_r_ex, mem_w_ex, rs1_d, rs2_d, rd_d, rs1_ex, rs2_ex, rd_ex, pc_4_d, pc_4_ex,
mem_reg_ex, alu_b_ex, wr_en_ex, data_w_ex, flush);

input logic clk, reset, branch, mem_r, mem_w, mem_reg, alu_b, wr_en, flush;
input logic [1:0] alu_op, data_w;
input logic [31:0] rd1, rd2, pc, imm, pc_4_d;
input logic [4:0] rs1_d, rs2_d, rd_d;
output logic  branch_ex, mem_r_ex, mem_w_ex, mem_reg_ex, alu_b_ex, wr_en_ex;
output logic [1:0] alu_op_ex, data_w_ex;
output logic [31:0] rd1_ex, rd2_ex, pc_ex, imm_ex, pc_4_ex;
output logic [4:0] rs1_ex, rs2_ex, rd_ex;


always @(posedge clk) begin
	if (reset || flush) begin
		alu_op_ex <= 2'bx;
		branch_ex <= 0;
		mem_r_ex <= 0;
		mem_w_ex <= 0;
		mem_reg_ex <= 0;
		alu_b_ex <= 0;
		wr_en_ex <= 0;
		data_w_ex <= 0;
		rd1_ex <= 0;
		rd2_ex <= 0;
		pc_ex <= 0;
		imm_ex <= 0;
		rs1_ex <= 0;
		rs2_ex <= 0;
		rd_ex <= 0;
		pc_4_ex <= 0;
	end
	else begin 
		alu_op_ex <= alu_op;
		branch_ex <= branch;
		mem_r_ex <= mem_r;
		mem_w_ex <= mem_w;
		mem_reg_ex <= mem_reg;
		alu_b_ex <= alu_b;
		wr_en_ex <= wr_en;
		data_w_ex <= data_w;
		rd1_ex <= rd1;
		rd2_ex <= rd2;
		pc_ex <= pc;
		imm_ex <= imm;
		rs1_ex <= rs1_d;
		rs2_ex <= rs2_d;
		rd_ex <= rd_d;
		pc_4_ex <= pc_4_d;
	end
end
endmodule 




module MEM_EX(clk, reset, data_w_ex, wr_en_ex, mem_r_ex, mem_w_ex, data_w_m, wr_en_m, mem_r_m, mem_w_m, rd_ex, rd_m, pc_4_ex, pc_4_m,
 alu_res, rd2_ex, alu_res_m, rd2_m);

 input logic clk, reset, wr_en_ex, mem_r_ex, mem_w_ex;
 input logic [1:0] data_w_ex;
 input logic [31:0] alu_res, rd2_ex, pc_4_ex;
 input logic [4:0] rd_ex;
 output logic wr_en_m, mem_r_m, mem_w_m;
 output logic [1:0] data_w_m;
 output logic [4:0] rd_m;
 output logic [31:0] alu_res_m, rd2_m, pc_4_m;
 
 always @(posedge clk) begin
	if (reset) begin
		data_w_m <= 0;
		wr_en_m <= 0;
		mem_r_m <= 0;
		mem_w_m <= 0;
		alu_res_m <= 0;
		rd2_m <= 0;
		rd_m <= 0;
		pc_4_m <= 0;
	end
	else begin 
		data_w_m <= data_w_ex;
		wr_en_m <= wr_en_ex;
		mem_r_m <= mem_r_ex;
		mem_w_m <= mem_w_ex;
		alu_res_m <= alu_res;
		rd2_m <= rd2_ex;
		rd_m <= rd_ex;
		pc_4_m <= pc_4_ex;
	end	
end	
endmodule 




module WB_MEM(clk, reset, data_w_m, wr_en_m, alu_res_m, data_out, data_w_w, wr_en_w, alu_res_w, data_out_w, rd_m, rd_w, pc_4_m, pc_4_w);
	input logic clk, reset, wr_en_m;
	input logic [1:0] data_w_m;
	input logic [31:0] alu_res_m, data_out, pc_4_m;
	input logic [4:0] rd_m;
	output logic wr_en_w;
	output logic [1:0] data_w_w;
	output logic [31:0] alu_res_w, data_out_w, pc_4_w;
	output logic [4:0] rd_w;
	
	always @(posedge clk)  begin
		if (reset) begin
			wr_en_w <= 0;
			rd_w <= 0;
			data_w_w <= 2'bxx;
			alu_res_w <= 0;
			data_out_w <= 0;
			pc_4_w <= 0;
		end
		else begin 
			wr_en_w <= wr_en_m;
			rd_w <= rd_m;
			data_w_w <= data_w_m;
			alu_res_w <= alu_res_m;
			data_out_w <= data_out;	
			pc_4_w <= pc_4_m;
		end	
	end	
endmodule 




module multiplexer2(clk, a, b, sel, out);
	input logic clk;
	input logic [31:0] a, b;
	input logic sel;
	output logic [31:0] out;
	
	assign out = sel ? b : a;
	
endmodule	




module mux_3(a,b,c,sel,out1);

        input logic [31:0] a,b,c;
        input logic [1:0] sel;
        output logic [31:0] out1;

        always_latch @(*)
           if (sel== 2'b00)
                  out1 = a;
           else if (sel == 2'b01)
                  out1 = b;
           else if (sel == 2'b10)
                  out1 = c;
endmodule 
