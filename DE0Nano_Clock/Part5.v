// TCES 330, Spring 2015
// Date: May 4, 2015
// Author: Rahul Deshpande, David Rainey
//	
//	This module  uses a 1Hz counter to 
// count to display the 'H E L L O' message through
// the seven  segment displays of the board.

module Part5 (CLOCK_50, LED);
/*  Define the Inputs */
	input [0:0] CLOCK_50;
	output [2:0] LED;
	wire [2:0] S;
/* Define the outputs. */	
//	output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7;
/* Construct the 17-bit vector to pass to Part6 */
//		wire [2:0] U, V, W, X, Y, D0, D1, D2gt, S;
/* construct H E L L O	*/
/*		assign U = 3'b011;	
		assign V = 3'b000;
		assign W = 3'b001;
		assign X= 3'b010;
		assign Y = 3'b010;
/* Obtain the count */
		MyCustomCounter my_Counter (CLOCK_50, S);
		assign LED = S;
/* Invoke from Lab One............ */
//		part6 part6_From_Lab_One ({S, Y, X, W, V, U}, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7);
endmodule