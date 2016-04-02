module FlightControl (
// Inputs
	Rec_Rudder, Rec_Elevator, Rec_Aileron, Rec_Throttle, AUXONE, AUXTWO, 
	SD_Rudder, SD_Elevator, SD_Aileron , SD_Throttle, SD_Pan, SD_Tilt,
	RPIAckCruise, RPIAckAuto,
	XbeeGpsSelect,
	RPITx, GpsTx, XbeeTx,
// Rec_Rudder - Rudder from Receiver ( used for Pan under Auto=0, Cruise = 1 and Auto = 1, Cruise = 1)
// Rec_Elevator - Elevator from Receiver
// Rec_Aileron - Aileron from Receiver
// Rec_Throttle - Throttle from Receiver (used for Tilt under Auto=0, Cruise = 1 and Auto = 1, Cruise = 1)
// AUXONE  - Aux one for  turning on Cruise Control
// AUXTWO - For turning on Autonomous  Mode.
// SD_Rudder -  Rudder from Servo Driver 
// SD_Elevator - Elevator from Servo Driver 
// SD_Aileron - Aileron from Servo Driver
// SD_Throttle - Throttle from Servo Driver 
// SD_Pan -  Pan
// SD_Tilt - Tilt
// RPIAckCruise - An Acknowledge from Raspberry Pi signal for Cruise Control Mode
// RPIAckAuto - An Acknowledge from Raspberry PI signal  for Autonomous Control
// XbeeOrGpsSwitch - Signal from Raspberry PI indicating whether it wants to communicate with the Xbee or the GPS hardware  using  UART 
// RPITx - Tx from Raspberry PI for UART.
// GpsTx - Tx from  GPS module for UART.
// XbeeTx - Tx from Xbee module for UART

// Outputs
	RudderOutput, ElevatorOutput, AileronOutput, ThrottleOutput, PanOutput, TiltOutput,
	RPIReqCruise, RPIReqAuto,
	RPIRx, GpsRx, XbeeRx
// RPIReqCruise - Send Request to Raspberry PI indicating UAV is under Cruise Control.
// RPIReqAuto - Send Request to Raspberry PI indicating UAV is under Autonomous mode.
// RPIRx - Rx pin  from Raspberry PI
// GpsRx - Rx pin of GPS  hardware.
// XbeeRx - Rx pin of Xbee hardware.
	);
	input [0:0]  Rec_Rudder, Rec_Elevator, Rec_Aileron, Rec_Throttle,
				    SD_Rudder, SD_Elevator, SD_Aileron, SD_Throttle, SD_Pan, SD_Tilt,
					 AUXONE, AUXTWO,
					 RPIAckCruise, RPIAckAuto, XbeeGpsSelect,
					 RPITx, GpsTx, XbeeTx;
	output[0:0]	 RudderOutput, ElevatorOutput, AileronOutput, ThrottleOutput, PanOutput, TiltOutput,
					 RPIReqCruise, RPIReqAuto,
					 RPIRx, GpsRx, XbeeRx;
	
	// Used to indicate which mode UAV is operating.
	wire [0:0] Critical, Auto; //  Cannot assign a reg  through output of another module.
	// CC - Cruise Control Mode.
	// Auto - Autonomous Mode.
	
	// Name the instances to eliminate the 'instance not named' warning.
	
	RPI_Handshake handshake_inst_one(RPIAckCruise, Critical);
	RPI_Handshake handshake_inst_two(RPIAckAuto, Auto);
	
	XbeeGpsSwitch xbeegpsswitch_inst_one(XbeeGpsSelect,
		RPITx, GpsTx, XbeeTx,
		RPIRx, GpsRx, XbeeRx);
		
	AssignOutputsFromMode assignOutputsFromMode_inst_one(
		Auto, Critical,
		Rec_Rudder, Rec_Elevator, Rec_Aileron, Rec_Throttle, Rec_Rudder, Rec_Throttle,
		SD_Rudder, SD_Elevator, SD_Aileron, SD_Throttle, SD_Pan, SD_Tilt,
		RudderOutput, ElevatorOutput, AileronOutput, ThrottleOutput, PanOutput, TiltOutput
	);
endmodule