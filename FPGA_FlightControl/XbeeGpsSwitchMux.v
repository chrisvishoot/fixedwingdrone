module XbeeGpsSwitchMux(
	XbeeGpsSelect,
	RPITx, GpsTx, XbeeTx,
	RPIRx, GpsRx, XbeeRx
);

// Inputs
// XbeeOrGpsSwitch - Signal from Raspberry PI indicating whether it wants to communicate with the Xbee or the GPS hardware  using  UART.
// RPITx - Tx from Raspberry PI for UART.
// GpsTx - Tx from  GPS module for UART.
// XbeeTx - Tx from Xbee module for UART.
	input [0:0] XbeeGpsSelect,RPITx, GpsTx, XbeeTx;
	output [0:0] RPIRx, GpsRx, XbeeRx;
   reg[0:0]  XbeeRx, GpsRx;
	
// Implement a 2-1 MUX
	assign RPIRx = (~XbeeGpsSelect & GpsTx) | (XbeeGpsSelect & XbeeTx);
	always @(XbeeGpsSelect)
	begin
		if(XbeeGpsSelect)
		begin
			XbeeRx = RPITx;
		end
		else
		begin
			GpsRx = RPITx;
		end
	end
endmodule