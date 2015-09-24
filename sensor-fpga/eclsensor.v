// Proximity sensor for Eclipse
// (c) 2015 Micah Elizabeth Scott

// For the KNJN Pluto board, with the Altera EP1K10TC100-3.

// This board has some discrete logic that lets us configure the FPGA
// using a UART at 115200 baud 8-N-1 inverted.

// It benefits us to blast out the data quickly since there isn't room
// to double-buffer, so the sensor results are reported at 921600 8-N-1.
// The Raspberry Pi needs a faster UART clock to handle this:
//   init_uart_clock=14745600
//   (https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=73673)

module eclsensor (clk, txd, rxd, ir_tx, ir_rx, led);

	parameter kClockHz = 25_000_000;	// Matches the oscillator on Pluto
	parameter kSerialInvert = 1;		// Needs to match the config logic on the Pluto
	parameter kBaudRate = 921600;		// Doesn't need to match the config logic
	parameter kModulationHz = 38000;	// Center frequency of the receiver
	parameter kBurstMicrosec = 2500;    // How long to drive the transmitter
	parameter kQuietMicrosec = 500;     // Additional time to wait between repeats, with receivers on
	parameter kTxCount = 12;
	parameter kRxCount = 20;
	parameter kRxTimerNybbles = 4;
	parameter kRxTimerBits = kRxTimerNybbles * 4;

	output reg [kTxCount-1:0] ir_tx;
	input [kRxCount-1:0] ir_rx;
	output led;
	input clk;
	output reg txd;
	input rxd;

	// Keep the onboard LED dark unless we need it for debug
	assign led = 0;

	// Counter for amplitude modulating the transmitters at the frequency our receivers select for
	reg [8:0] modulation_timer = 0;
	reg modulation_state = 0;
	wire modulation_next = !modulation_state;
	wire modulation_edge = modulation_timer == kClockHz / kModulationHz / 2;
	always @(posedge clk) begin
		if (modulation_edge) begin
			modulation_timer <= 0;
			modulation_state <= modulation_next;
		end
		else
			modulation_timer <= modulation_timer + 1'b1;
	end

	// The per-transmitter state is tracked by counting modulation cycles.
	// The cycle timer resets after state_tx_end, and stays in reset until serial TX finishes.
	// This is necessary to give us time to shift out the collected counts, since we don't
	// have enough room to latch them all separately.

	reg [11:0] cycle_timer = 0;
	wire state_tx_modulating = cycle_timer < kBurstMicrosec * 2 * kModulationHz / 1000000;
	wire state_tx_end = modulation_edge && cycle_timer == (kBurstMicrosec + kQuietMicrosec) * 2 * kModulationHz / 1000000;
	always @(posedge clk)
		if (modulation_edge) begin
			if (text_busy || state_tx_end)
				cycle_timer <= 1'b0;
			else
				cycle_timer <= cycle_timer + 1'b1;
		end

	// Loop through all transmit channels, switching right after state_tx_end
	reg [3:0] current_tx = 0;
	wire [3:0] prev_tx = ~|current_tx ? kTxCount-1'b1 : current_tx-1'b1;
	wire state_tx_end_last = state_tx_end && current_tx == kTxCount-1;
	always @(posedge clk)
		if (state_tx_end_last)
			current_tx <= 0;
		else if (state_tx_end)
			current_tx <= current_tx + 1'b1;

	// Light up one transmitter at a time, with amplitude modulation, during state_tx_modulating
	always @(posedge clk)
		ir_tx <= (state_tx_modulating && !text_busy && modulation_state) << current_tx;

	// Input registers, synch the IR receivers
	reg [kRxCount-1:0] ir_rx_reg;
	always @(posedge clk)
		ir_rx_reg <= ir_rx;
		
	// The counters sum positive clock cycles on that channel, during the non-masked part of the cycle,
	// then shifts out nybbles at a time while we're in serial transmission.

	reg [kRxTimerBits-1:0] rx_counter[0:kRxCount-1];
	wire [kRxTimerBits-1:0] rx_counter_next[0:kRxCount-1];
	reg rx_nybble_strobe;  // input
	reg rx_clear;          // input
    wire [3:0] rx_nybble = rx_counter[0][kRxTimerBits-1:kRxTimerBits-4];
	genvar rx_i, bit_i;	
	generate
		for (rx_i = 0; rx_i < kRxCount; rx_i = rx_i + 1) begin: rx_i_block

			// Count LOW states in ir_rx_reg, masked when sending results.
			assign rx_counter_next[rx_i] = rx_counter[rx_i] + (!text_busy && !ir_rx_reg[rx_i]);

			// Combo counter and 4-lane shift register
			for (bit_i = 0; bit_i < kRxTimerBits; bit_i = bit_i + 1) begin: bit_i_block
				always @(posedge clk)
					rx_counter[rx_i][bit_i] <=
						rx_clear ? 1'b0 :
						rx_nybble_strobe ?
							(bit_i >= 4 ? rx_counter[rx_i][bit_i - 4] :
								(rx_i + 1 >= kRxCount) ? 4'hX : rx_counter[rx_i + 1][bit_i + kRxTimerBits - 4]) :
						rx_counter_next[rx_i][bit_i];
			end
		end
	endgenerate

	// Tiny baud rate generator
	reg [7:0] baud_timer = 0;
	reg baud_edge;
	always @(posedge clk)
		if (baud_timer == kClockHz / kBaudRate) begin
			baud_edge <= 1;
			baud_timer <= 0;
		end
		else begin
			baud_edge <= 0;
			baud_timer <= baud_timer + 1'b1;
		end

	// Tiny 8-N-1 asynchronous serial transmitter
	reg [9:0] async_tx_reg = 0;
	reg [3:0] async_tx_state = 0;
	reg async_tx_busy = 0;
	wire [7:0] async_tx_next_byte;     // Input
	wire async_tx_request;  		   // Input
	always @(posedge clk) begin
		if (baud_edge) begin
			txd <= kSerialInvert[0] ^ (async_tx_busy ? async_tx_reg[0] : 1'b1);
		end
		if (!async_tx_busy && async_tx_request) begin
			async_tx_reg <= { 1'b1, async_tx_next_byte, 1'b0 };
			async_tx_state <= 4'h0;
			async_tx_busy <= 1'b1;
		end		
		else if (async_tx_busy && baud_edge) begin
			async_tx_reg <= { 1'b1, async_tx_reg[9:1] };
			async_tx_state <= async_tx_state + 1'b1;
			async_tx_busy <= async_tx_state != 9;
		end
	end
	
	// Transmit data back as simple ASCII packets that compromise
	// between readability and compactness. Each TX is one line
	// ending with a "\n". The line begins with a capital letter
	// identifying the transmitter, followed by groups of lowercase
	// hex digits carrying the timings from each receiver.

	reg text_busy = 0;
	reg [6:0] text_counter = 7'hXX;

	assign async_tx_request = !async_tx_busy && text_busy;
	assign async_tx_next_byte =
		text_counter == 2 + kRxTimerNybbles * kRxCount ? "A" + {3'b0, prev_tx} :
		text_counter == 0 ? "\n" :
		(rx_nybble < 8'd10 ?
			"0" + {4'b0, rx_nybble} :
			("a" - 8'd10) + {4'b0, rx_nybble});
		
	always @(posedge clk) begin
		if (!text_busy) begin
			// Idle, wait for a TX cycle to end
			if (state_tx_end) begin
				text_busy <= 1'b1;
			end
		
			text_counter <= 2 + kRxTimerNybbles * kRxCount;
		
			// Important to leave the counters alone here, this is where sampling happens
			rx_nybble_strobe <= 0;
			rx_clear <= 0;
		end
		else if (text_counter != 0) begin
			// Text character
			if (async_tx_request) begin
				text_counter <= text_counter - 1'b1;
				rx_nybble_strobe <= 1;
			end
			else
				rx_nybble_strobe <= 0;
			rx_clear <= 0;
		end
 		else begin
			// End of line.
			if (async_tx_request) begin
				text_busy <= 0;
				rx_clear <= 1;
			end
			rx_nybble_strobe <= 0;
		end
	end

endmodule
