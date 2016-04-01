module AssignOutputsFromMode(
	Auto, Critical,
	Rec_Rudder, Rec_Elevator, Rec_Aileron, Rec_Throttle, Rec_Pan, Rec_Tilt,
	SD_Rudder, SD_Elevator, SD_Aileron, SD_Throttle, SD_Pan, SD_Tilt,
	Rudder, Elevator, Aileron, Throttle, Pan, Tilt
);
// Inputs
// Mode - Constitutes the four possible values for Auto and Critical from 
// 		 top Level Module.
	input [0:0] Auto, Critical; 
	input [0:0] Rec_Rudder, Rec_Elevator, Rec_Aileron, Rec_Throttle, Rec_Pan, Rec_Tilt,
				   SD_Rudder, SD_Elevator, SD_Aileron, SD_Throttle, SD_Pan, SD_Tilt;
	
	output [0:0] Rudder, Elevator, Aileron, Throttle, Pan, Tilt;
// Always assign outputs  when the combination changes.

	 assign Rudder = (~Auto & ~Critical & Rec_Rudder)|((Auto | Critical)& SD_Rudder);
	 assign Elevator= (~Auto & Rec_Elevator)|(Auto & SD_Elevator);
	 assign Aileron= (~Auto & Rec_Aileron)|(Auto & SD_Aileron);
	 assign Throttle = (~Auto & ~Critical & Rec_Throttle)|((Auto | Critical)& SD_Throttle);
	 assign Pan = (~Auto & Critical & Rec_Pan)|((Auto| ~Critical) & SD_Pan);
	 assign Tilt = (~Auto & Critical & Rec_Tilt)|((Auto| ~Critical) & SD_Tilt);
	 
endmodule