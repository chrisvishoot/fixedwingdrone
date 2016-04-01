module RPI_Handshake(RPI_Ack, Mode);
	input [0:0] RPI_Ack;
	output [0:0] Mode;
	wire [0:0] Mode;
	assign Mode= RPI_Ack;
endmodule