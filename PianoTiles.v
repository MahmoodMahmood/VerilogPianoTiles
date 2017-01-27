`timescale 1ns / 1ns // `timescale time_unit/time_precision

module PianoTiles
	(
		HEX0,
		HEX1,
		HEX2,
		HEX3,
		HEX4,
		HEX5,
		LEDR,
		CLOCK_50,						//	On Board 50 MHz 
		KEY, 
		SW,
		// Your inputs and outputs here
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B   						//	VGA Blue[9:0]
	);

	input			CLOCK_50;				//	50 MHz
	input [3:0] KEY;//This input is used to click on each column
	input [9:0] SW;//Input for reset
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	output	[6:0]	HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
	output	[9:0]	LEDR;
	wire resetn;
	assign resetn = SW[9];//This is the input to resetn
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.

	reg [2:0] colour;
	reg [7:0] x;
	reg [6:0] y;
	reg writeEn;
	wire writeEn1, writeEn2, writeEn3, writeEn4;
	wire green1, green2, green3, green4;
	wire [3:0] shift_r, set, FPSTick, random, move;
	wire [119:0] LUTY1, LUTY2, LUTY3, LUTY4, LUTY12, LUTY22, LUTY32, LUTY42;
	wire [31:0] out1, out2, out3, out4;
	reg [1:0] selectColumn;
	assign random[0] = out1[0];
	assign random[1] = out2[0];
	assign random[2] = out3[0];
	assign random[3] = out4[0];
	reg [3:0] start;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(~resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
		
		//This makes a call to grab the details from the gameover RAM module
		module GameOver g1(
			address,
			clock,
			data,
			wren,
			q
		);
		
		//Call the counter to control speed of blocks
		counter c1(.Clock(CLOCK_50), .resetn(SW[0]), .enable(FPSTick));
		
		//Instantiate all the modules needed to draw each column (see document for more details)
		
		//This function draws the tiles using the look up table generated
		tile t1(.Clock(CLOCK_50), .start(draw[0]), .writeEn(writeEn1), .shift_r(shift_r[0]), .set(set[0]), .resetn(SW[0]), .LUTY(LUTY1), .count2(count1), .move(move[0]), .done(done[0]));
		//Datapath to choose what process to be executed 
		datapath d1(.Clock(CLOCK_50), .shift_r(shift_r[0]), .resetn(resetn), .value_x(x1), .value_y(y1), .colour(colour1), .startx(startx1), .starty(starty1), .set(set[0]), .clear(clear[0]), .move(move[0]));
		//Create and shift y value lookup table to represent position of boxes
		LookUpTable l1(.CLOCK_50(CLOCK_50), .random(random[0]), .LUTY(LUTY1), .pause(0), .resetn(SW[0]), .Shift(Shift[0]), .in(in1), .LUTY2(LUTY12));
		//This function generates random numbers
		RandomNumberGenerator r1 (.out(out1), .CLOCK_50(CLOCK_50), .rst(SW[0]));
		
		tile t2(.Clock(CLOCK_50), .start(draw[1]), .writeEn(writeEn2), .shift_r(shift_r[1]), .set(set[1]), .resetn(SW[0]), .LUTY(LUTY2), .count2(count2), .move(move[1]), .done(done[1]));
		datapath d2(.Clock(CLOCK_50), .shift_r(shift_r[1]), .resetn(resetn), .value_x(x2), .value_y(y2), .colour(colour2), .startx(startx2), .starty(starty2), .set(set[1]), .clear(clear[1]), .move(move[1]));
		LookUpTable l2(.CLOCK_50(CLOCK_50), .random(random[1]), .LUTY(LUTY2), .pause(0), .resetn(SW[0]), .Shift(Shift[1]), .in(in2), .LUTY2(LUTY22));
		RandomNumberGenerator r2 (.out(out2), .CLOCK_50(CLOCK_50), .rst(SW[0]));
		
		tile t3(.Clock(CLOCK_50), .start(draw[2]), .writeEn(writeEn3), .shift_r(shift_r[2]), .set(set[2]), .resetn(SW[0]), .LUTY(LUTY3), .count2(count3), .move(move[2]), .done(done[2]));
		datapath d3(.Clock(CLOCK_50), .shift_r(shift_r[2]), .resetn(resetn), .value_x(x3), .value_y(y3), .colour(colour3), .startx(startx3), .starty(starty3), .set(set[2]), .clear(clear[2]), .move(move[2]));
		LookUpTable l3(.CLOCK_50(CLOCK_50), .random(random[2]), .LUTY(LUTY3), .pause(0), .resetn(SW[0]), .Shift(Shift[2]), .in(in3), .LUTY2(LUTY32));
		RandomNumberGenerator r3 (.out(out3), .CLOCK_50(CLOCK_50), .rst(SW[0]));
		
		tile t4(.Clock(CLOCK_50), .start(draw[3]), .writeEn(writeEn4), .shift_r(shift_r[3]), .set(set[3]), .resetn(SW[0]), .LUTY(LUTY4), .count2(count4), .move(move[3]), .done(done[3]));
		datapath d4(.Clock(CLOCK_50), .shift_r(shift_r[3]), .resetn(resetn), .value_x(x4), .value_y(y4), .colour(colour4), .startx(startx4), .starty(starty4), .set(set[3]), .clear(clear[3]), .move(move[3]));
		LookUpTable l4(.CLOCK_50(CLOCK_50), .random(random[3]), .LUTY(LUTY4), .pause(0), .resetn(SW[0]), .Shift(Shift[3]), .in(in4), .LUTY2(LUTY42));
		RandomNumberGenerator r4 (.out(out4), .CLOCK_50(CLOCK_50), .rst(SW[0]));
		
		//Declare all necessary variables
		wire [3:0] done;
		reg [3:0] Shift, draw, clear;
		reg [9:0] current, next;
		wire [7:0] x1, x2, x3, x4, startx1, startx2, startx3, startx4;
		wire [6:0] y1, y2, y3, y4, starty1, starty2, starty3, starty4;
		wire [6:0] count1, count2, count3, count4;
		wire [2:0] colour1, colour2, colour3, colour4;
		wire in1, in2, in3, in4;
		reg [15:0] score1, score2, score3, score4;
		wire [15:0] overallscore;
		
		//Overall score is just the sum of the scores from
		assign overallscore = score1 + score2 + score3 + score4;
		//Key input
		assign in1 = ~KEY[3];
		assign in2 = ~KEY[2];
		assign in3 = ~KEY[1];
		assign in4 = ~KEY[0];
		//X position of each block (seperated by 30 pixels)
		assign startx1 = 8'd10;
		assign startx2 = 8'd40;
		assign startx3 = 8'd70;
		assign startx4 = 8'd100;
		//Y start position of each block depends on the position in the lookup table
		assign starty1 = count1;
		assign starty2 = count2;
		assign starty3 = count3;
		assign starty4 = count4;
		
		//This is practically a 5 to 1 MUX that decides what column to draw
		always@(*)
		begin 
			if (selectColumn == 2'b00)
			begin
				x = x1;//Set VGA input to column 1 x
				y = y1;//Set VGA input to column 1 y
				colour = colour1;//Set VGA input to column 1 colour
				writeEn = writeEn1;//Set VGA writeEnable to Column1 writeEnable
			end
			else if (selectColumn == 2'b01)
			begin
				x = x2;
				y = y2;
				colour = colour2;
				writeEn = writeEn2;
			end
			else if (selectColumn == 2'b10)
			begin
				x = x3;
				y = y3;
				colour = colour3;
				writeEn = writeEn3;
			end
			else if (selectColumn == 2'b11)
			begin
				x = x4;
				y = y4;
				colour = colour4;
				writeEn = writeEn4;
			end
		end 
		localparam WAIT = 10'd0, 
					  COL1LUT = 10'd9,
					  COL2LUT = 10'd10,
					  COL3LUT = 10'd11,
					  COL4LUT = 10'd12,
					  COL1DRAW = 10'd1,
					  COL1WAIT = 10'd2,
					  COL2DRAW = 10'd3,
					  COL2WAIT = 10'd4,
					  COL3DRAW = 10'd5,
					  COL3WAIT = 10'd6,
					  COL4DRAW = 10'd7,
					  COL4WAIT = 10'd8,
					  COL1CLEAR = 10'd13,
					  COL1CLEARWAIT = 10'd14,
					  COL2CLEAR = 10'd15,
					  COL2CLEARWAIT = 10'd16,
					  COL3CLEAR = 10'd17,
					  COL3CLEARWAIT = 10'd18,
					  COL4CLEAR = 10'd19,
					  COL4CLEARWAIT = 10'd20,
					  IDLE = 10'd21,
					  DEAD = 10'd22;
		//8 bit counter to count and display fps ticks (useful for debugging)	  
		reg [7:0] FPScounter;
		always@(posedge FPSTick)
		begin
			if (SW[0] == 0) begin//Check for reset
				FPScounter <= 0;
			end
			else begin//Keep incrementing counter
				FPScounter <= FPScounter + 1'b1;
			end
		end
		//HEX output
		hex_decoder(FPScounter[3:0], HEX0);
		hex_decoder(FPScounter[7:4], HEX1);
		hex_decoder(overallscore[3:0], HEX2);
		hex_decoder(overallscore[7:4], HEX3);
		hex_decoder(overallscore[11:8], HEX4);
		hex_decoder(overallscore[15:12], HEX5);
		
		//Choose next state in the column selection FSM
		always@(*)
		begin
			case(current)
					  WAIT: begin next = SW[0] ? IDLE : WAIT; end
					  IDLE: begin next = FPSTick ? COL1CLEAR : IDLE; end
					  COL1LUT: begin next = COL2LUT; end
					  COL2LUT: begin next = COL3LUT; end
					  COL3LUT: begin next = COL4LUT; end
					  COL4LUT: begin next = COL1DRAW; end
					  COL1DRAW: begin next = COL1WAIT; end
					  COL1WAIT: begin next = done[0] ? COL2DRAW : COL1WAIT; end
					  COL2DRAW: begin next = COL2WAIT; end
					  COL2WAIT: begin next = done[1] ? COL3DRAW : COL2WAIT; end
					  COL3DRAW: begin next = COL3WAIT; end
					  COL3WAIT: begin next = done[2] ? COL4DRAW : COL3WAIT; end
					  COL4DRAW: begin next = COL4WAIT; end 
					  COL4WAIT: begin next = done[3] ? IDLE : COL4WAIT; end
					  
					  COL1CLEAR: begin next = COL1CLEARWAIT; end
					  COL1CLEARWAIT: begin next = done[0] ? COL2CLEAR : COL1CLEARWAIT; end
					  COL2CLEAR: begin next = COL2CLEARWAIT; end
					  COL2CLEARWAIT: begin next = done[1] ? COL3CLEAR : COL2CLEARWAIT; end
					  COL3CLEAR: begin next = COL3CLEARWAIT; end
					  COL3CLEARWAIT: begin next = done[2] ? COL4CLEAR : COL3CLEARWAIT; end
					  COL4CLEAR: begin next = COL4CLEARWAIT; end
					  COL4CLEARWAIT: begin next = done[3] ? COL1LUT : COL4CLEARWAIT; end
					  DEAD: next = DEAD;
					  default: begin next = WAIT; end
			endcase
		end
		
		//Depending on the current state, choose the necessary signals to activate
		always@(posedge CLOCK_50)
		begin
		//Instantiate every control signals to 0
		Shift[3:0] = 4'd0; draw[3:0] = 4'd0; clear[3:0] = 4'd0; selectColumn[1:0] = 2'd0;
			case(current)
				IDLE: begin Shift[3:0] = 4'd0; draw[3:0] = 4'd0; clear[3:0] = 4'd0; end
				//Let the shift cpntrol signal be 1 
				COL1LUT: begin Shift[0] = 1'b1; end
				COL2LUT: begin Shift[1] = 1'b1; end
				COL3LUT: begin Shift[2] = 1'b1; end
				COL4LUT: begin Shift[3] = 1'b1; end
				
				//Let the draw control signal be 1 and select appropriate column
				COL1DRAW: begin draw[0] = 1'b1; selectColumn = 2'b00; end
				//Wait after the column is drawn
				COL1WAIT: begin draw[0] = 1'b0; selectColumn = 2'b00; end
				COL2DRAW: begin draw[1] = 1'b1; selectColumn = 2'b01; end
				COL2WAIT: begin draw[1] = 1'b0; selectColumn = 2'b01; end
				COL3DRAW: begin draw[2] = 1'b1; selectColumn = 2'b10; end
				COL3WAIT: begin draw[2] = 1'b0; selectColumn = 2'b10; end
				COL4DRAW: begin draw[3] = 1'b1; selectColumn = 2'b11; end
				COL4WAIT: begin draw[3] = 1'b0; selectColumn = 2'b11; end
				
				//Set the clear signal to 0 and draw signal to 1 while also selecting the appropriate column
				COL1CLEAR: begin clear[0] = 1'b1; draw[0] = 1'b1; selectColumn = 2'b00; end
				COL1CLEARWAIT: begin clear[0] = 1'b1; draw[0] = 1'b0; selectColumn = 2'b00; end
				COL2CLEAR: begin clear[1] = 1'b1; draw[1] = 1'b1; selectColumn = 2'b01; end
				COL2CLEARWAIT: begin clear[1] = 1'b1; draw[1] = 1'b0; selectColumn = 2'b01; end
				COL3CLEAR: begin clear[2] = 1'b1; draw[2] = 1'b1; selectColumn = 2'b10; end
				COL3CLEARWAIT: begin clear[2] = 1'b1; draw[2] = 1'b0; selectColumn = 2'b10; end
				COL4CLEAR: begin clear[3] = 1'b1; draw[3] = 1'b1; selectColumn = 2'b11; end
				COL4CLEARWAIT: begin clear[3] = 1'b1; draw[3] = 1'b0; selectColumn = 2'b11; end
				
			endcase
		end
		
		//Increment score whenever the key is released
		always@(negedge in1)
		begin
			if(SW[9])
				score1 = 10'd0;
			else
				score1 = score1 + 1'b1;
		end
		always@(negedge in2)
		begin
			if(SW[9])
				score2 = 10'd0;
			else
				score2 = score2 + 1'b1;
		end
		always@(negedge in3)
		begin
			if(SW[9])
				score3 = 10'd0;
			else
				score3 = score3 + 1'b1;
		end
		always@(negedge in4)
		begin
			if(SW[9])
				score4 = 10'd0;
			else
				score4 = score4 + 1'b1;
		end
		
		//Check if the user clicked on a black square
		//They will die if they do
		always@(posedge CLOCK_50)
		begin
			if(!SW[0])
				current <= WAIT;
			else if(in1 == 1'b1 && LUTY12[119] == 1'b0) //these extra if statements check if any key is pressed while there is no box on the bottom of the screen
				current <= DEAD;								  //if any of the statements becomes true, the game just keeps looping on the DEAD state which pretty much does nothing
			else if(in2 == 1'b1 && LUTY22[119] == 1'b0) //if you want you can create an FSM to draw a "You are dead" screen once the DEAD state is hit
				current <= DEAD;								
			else if(in3 == 1'b1 && LUTY32[119] == 1'b0)
				current <= DEAD;
			else if(in4 == 1'b1 && LUTY42[119] == 1'b0)
				current <= DEAD;
			else
            current <= next;
		end
endmodule

//This module draws the box based on input from the y position look up table (LUTY)
module tile (Clock, start, writeEn, shift_r, set, resetn, LUTY, count2, move, done);
	input start, Clock, resetn;
	input [119:0] LUTY;
	output reg writeEn, shift_r, set, move, done;
	wire [7:0] boxx;
	reg [3:0] current, next;
	reg [4:0] count;
	output reg [6:0] count2;
	
	localparam PLOT = 4'd1, SHIFT_R = 4'd2, CHECK = 4'd4, WAIT = 4'd5, SETPOS = 4'd7, IDLE2 = 4'd8, MOVELUT = 4'd9, DONE = 4'd10;
	assign boxx = 8'd10;

	//Decide what the next case is
	always@(*)
	begin
		case (current)
			//If stay on wait until start is trigered
			WAIT: begin next = start ? IDLE2 : WAIT; end
			//This is just to burn a clock cycle to get other stuff to set up
			IDLE2: begin next = CHECK; end
			//Spend 
			CHECK: begin 
						if(count2 < 7'd120)
						begin
							if(LUTY[count2] == 1'b1) //checks if the lookup table has a logic 1. If so draw one line of pixels
								next = SETPOS;
							else
								next = MOVELUT; //checks the next bit in the lookup table
						end
						else begin
							next = DONE;
						end
					 end
			SETPOS: begin next = PLOT; end
			PLOT: begin 
						if (count == boxx)
							next = MOVELUT;
						else 
							next = SHIFT_R; //shifts x values to draw the line
					end
			SHIFT_R: begin next = PLOT; end
			MOVELUT: begin next = CHECK; end
			DONE: begin next = WAIT; end //sends a done signal back to top level FSM
			default: begin next = WAIT; end
		endcase
	end
	
	//Change the control signal to perform the necessary tasks
	always@(posedge Clock)
   begin
		writeEn = 1'b0; shift_r = 1'b0; set = 1'b0; move = 1'b0; done = 1'b0;
        case(current)
					WAIT: begin done = 1'b0; end
					IDLE2: begin count2 = 7'd0; count = 5'd0; end
					SETPOS: begin set = 1'b1; end
					PLOT: begin writeEn = 1'b1; end
					SHIFT_R: begin count = count + 1; shift_r = 1'b1; writeEn = 1'b1; end//This shifts all the counter
					MOVELUT: begin count2 = count2 + 1; count = 5'd0; move = 1'b1; end//Move the LUT down and keep drawing
					DONE: begin done = 1'b1; count2 = 7'd0; end
		  endcase
	end 
	
	always@(posedge Clock)
    begin
        if(!resetn)
            current <= WAIT;
        else
            current <= next;
    end
endmodule

//This is a basic counter
module counter(Clock, resetn, enable);
	input Clock, resetn;
	output reg enable;
	reg [29:0] count, accelcount, offset;
	reg [29:0] offsetenable;
	wire [29:0] Tick;
	//Tick decides frames per second
	//Subtract the offsetenable in order to accelerate
	assign Tick = 30'd3000000 - offsetenable;
	
	//Every "TICK" clock edges we want to flash "enable"
	always@(posedge Clock)
	begin
			if(~resetn) begin
				count = 30'd0;
				enable = 1'b0; end
			else begin
				if(count == Tick) begin//Set enable to 1 if and only if count = tick which happens once every tick
					enable = 1'b1;
					count = 30'd0; end//Reset count when we flash enable
				else begin
					count = count + 1;
					enable = 1'b0; end
			end
	end
	
	//Every 10000000 clock cycles we want to increment acceleration
	always@(posedge Clock)
	begin
		if(~resetn)
		begin
			accelcount = 30'd0;
		end
		else if(accelcount == 30'd10000000)
			accelcount = 30'd0;
		else
			accelcount = accelcount + 1;
	end
	
	//Everytime we want to enable acceleration
	always@(posedge accelcount)
	begin
		if(~resetn)
		begin
			offset = 30'd0;
		end
		//As long as we haven't reached a cap of enabling acceleration 100 times
		else if(offset > 100)
		begin
			offset = offset;
		end
		
		//We keep incrementing acceleration offset
		else if(accelcount == 30'd1)
		begin
			offset = offset + 1;
		end
		
		else
		begin
			offset = offset;
		end
	end
	
	//Everytime we increment the acceleration offset
	always@(posedge offset)
	begin
		if(~resetn)
			offsetenable = 30'd0;
		else
			offsetenable = offset * 20000;//We make the offset  enable (final value) a multiple of 20000
	end
endmodule

//This is the datapath (see document for details)
module datapath(Clock, shift_r, resetn, value_x, value_y, colour, startx, starty, set, clear, move);
	 input Clock, shift_r, resetn, set, clear, move;
	 output reg [7:0] value_x;
	 output reg [6:0] value_y;
	 output reg [2:0] colour;
	 input [7:0] startx;
	 input [6:0] starty;

	 always@(posedge Clock)
		begin
				if(shift_r)//If we want to shift right we just increment the x value
					value_x = value_x + 1;
				if(set)//Reset the x and y value to initial x and y value
					begin value_x = startx; value_y = starty; end
				if(clear)//To clear, set colour to white 
					colour = 3'b000;
				if(~clear)//Set color to white if we aren't clearing
					colour = 3'b111;
				if(move)//Just moves the x value to initial
					value_x = startx;
		end
endmodule

//This module generates pseuodo random numbers
module RandomNumberGenerator (out, CLOCK_50, rst);
  output reg [30:0] out;
  input CLOCK_50, rst;
  wire feedback;
  
  //Basically returns a mashup of XOR gates between the least significant bits that depends on
  //those same bits themselves. Meaning we have a chaotic self induced chaotic system
  assign feedback = ~(((out[3] ^ out[2]) ^ (out[4] ^ out[5])) ^ 
							((out[6]^out[7])^(out[8]^out[9])) ^ 
							((out[10] ^ out[11]) ^ (out[12] ^ out[13])) ^
							((out[14] ^ out[15]) ^ (out[16] ^ out[17])));


	always @(posedge CLOCK_50)
	begin
		if (~rst)
			out = 32'b0;
		else
			out = {out[30:0],feedback};//Assign the least significant bit of out to be the random feedback bit
  end
endmodule

//This module outputs a lookup table for the y position of the blocks in each column
module LookUpTable (CLOCK_50, random, LUTY, pause, resetn, Shift, in, LUTY2);
	input CLOCK_50, random, pause, resetn, in;
	output reg [119:0] LUTY, LUTY2;
	input Shift;
	localparam LUTYsize = 10'd119;
	
	//Everytime we are asked to shift the table down
	//This counter counts how many shifts you have to wait until you can insert a new block (21 shifts)
	always@(posedge Shift)
	begin
		if(~resetn)
			counterDisable = 27'd0;
		else if(counterDisable == 27'd21)
			counterDisable = 27'd0;
		else 
			counterDisable = counterDisable + 1;
	end
	
	//Everytime we need to shift down
	always@(posedge Shift)
	begin
		if(~resetn) begin
			LUTY = 120'd0;
			LUTY2 = 120'd0;
			end
		//If its time to create a new block, we do that and set the block equal to the random input 
		else if(counterDisable == 27'd20) begin
		//We make 2 boxes, one for printing and one for judging score (win or lose)
			LUTY[0] = random;
			LUTY[1] = random;
			LUTY[2] = random;
			LUTY[3] = random;
			LUTY[4] = random;
			LUTY[5] = random;
			LUTY[6] = random;
			LUTY[7] = random;
			LUTY[8] = random;
			LUTY[9] = random;
			LUTY[10] = random;
			LUTY[11] = random;
			LUTY[12] = random;
			LUTY[13] = random;
			LUTY[14] = random;
			LUTY[15] = random;
			LUTY[16] = random;
			LUTY[17] = random;
			LUTY[18] = random;
			LUTY[19] = random;
			
			LUTY2[0] = random;
			LUTY2[1] = random;
			LUTY2[2] = random;
			LUTY2[3] = random;
			LUTY2[4] = random;
			LUTY2[5] = random;
			LUTY2[6] = random;
			LUTY2[7] = random;
			LUTY2[8] = random;
			LUTY2[9] = random;
			LUTY2[10] = random;
			LUTY2[11] = random;
			LUTY2[12] = random;
			LUTY2[13] = random;
			LUTY2[14] = random;
			LUTY2[15] = random;
			LUTY2[16] = random;
			LUTY2[17] = random;
			LUTY2[18] = random;
			LUTY2[19] = random;
		end
		else begin
		//If we don't have to make a new block then we just shift our LUTY down
			LUTY = LUTY << 1;
			LUTY2 = LUTY2 << 1; end
		
		//Sooooo we tried to use a while and for loop but the internet told us not to (Anderson said no as well :( )
		//This beautiful display of programming prowess, dedication and shear beauty deletes a tile when it is clicked on
		if(in == 1'b1)
		begin
			if(LUTY[LUTYsize] == 1'b1)
			begin
				LUTY[LUTYsize] = 5'd0;
				if(LUTY[LUTYsize - 5'd1] == 1'b1)
				begin
					LUTY[LUTYsize - 5'd1] = 1'b0;
					if(LUTY[LUTYsize - 5'd2] == 1'b1)
					begin
						LUTY[LUTYsize - 5'd2] = 1'b0;
						if(LUTY[LUTYsize - 5'd3] == 1'b1)
						begin
							LUTY[LUTYsize - 5'd3] = 1'b0;
							if(LUTY[LUTYsize - 5'd4] == 1'b1)
							begin
								LUTY[LUTYsize - 5'd4] = 1'b0;
								if(LUTY[LUTYsize - 5'd5] == 1'b1)
								begin
									LUTY[LUTYsize - 5'd5] = 1'b0;
									if(LUTY[LUTYsize - 5'd6] == 1'b1)
									begin
										LUTY[LUTYsize - 5'd6] = 1'b0;
										if(LUTY[LUTYsize - 5'd7] == 1'b1)
										begin
											LUTY[LUTYsize - 5'd7] = 1'b0;
											if(LUTY[LUTYsize - 5'd8] == 1'b1)
											begin
												LUTY[LUTYsize - 5'd8] = 1'b0;
												if(LUTY[LUTYsize - 5'd9] == 1'b1)
												begin
													LUTY[LUTYsize - 5'd9] = 1'b0;
													if(LUTY[LUTYsize - 5'd10] == 1'b1)
													begin
														LUTY[LUTYsize - 5'd10] = 1'b0;
														if(LUTY[LUTYsize - 5'd11] == 1'b1)
														begin
															LUTY[LUTYsize - 5'd11] = 1'b0;
															if(LUTY[LUTYsize - 5'd12] == 1'b1)
															begin
																LUTY[LUTYsize - 5'd12] = 1'b0;
																if(LUTY[LUTYsize - 5'd13] == 1'b1)
																begin
																	LUTY[LUTYsize - 5'd13] = 1'b0;
																	if(LUTY[LUTYsize - 5'd14] == 1'b1)
																	begin
																		LUTY[LUTYsize - 5'd14] = 1'b0;
																		if(LUTY[LUTYsize - 5'd15] == 1'b1)
																		begin
																			LUTY[LUTYsize - 5'd15] = 1'b0;
																			if(LUTY[LUTYsize - 5'd16] == 1'b1)
																			begin
																				LUTY[LUTYsize - 5'd16] = 1'b0;
																				if(LUTY[LUTYsize - 5'd17] == 1'b1)
																				begin
																					LUTY[LUTYsize - 5'd17] = 1'b0;
																					if(LUTY[LUTYsize - 5'd18] == 1'b1)
																					begin
																						LUTY[LUTYsize - 5'd18] = 1'b0;
																						if(LUTY[LUTYsize - 5'd19] == 1'b1)
																						begin
																							LUTY[LUTYsize - 5'd19] = 1'b0;
																						end
																					end
																				end
																			end
																		end
																	end
																end
															end
														end
													end
												end
											end
										end
									end
								end
							end
						end
					end
				end
			end
		end
	end	
endmodule 

//Outputs values to hex display
module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule
