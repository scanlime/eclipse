// Proximity sensor for Eclipse
// (c) 2015 Micah Elizabeth Scott

// For the KNJN Pluto board, with the Altera EP1K10TC100-3.

module eclsensor (clk, txd, rxd, ir_tx, ir_rx, led);

	parameter kClockHz = 25_000_000;
	parameter kBaudRate = 115200;
	parameter kSerialInvert = 1;
	parameter kModulationHz = 38000;
	parameter kBurstMillis = 20;
	parameter kRepeatMillis = 30;
	parameter kRxMaskingMillis = 5;
	parameter kTxCount = 12;
	parameter kRxCount = 20;
	parameter kRxTimerNybbles = 5;
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
	reg [15:0] modulation_timer = 0;
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

	// The per-transmitter state is tracked by counting modulation cycles
	reg [15:0] cycle_timer = 0;
	wire state_tx_modulating = cycle_timer < kBurstMillis * 2 * kModulationHz / 1000;
	wire state_tx_end = cycle_timer >= kRepeatMillis * 2 * kModulationHz / 1000;
	wire state_rx_masking = cycle_timer < kRxMaskingMillis * 2 * kModulationHz / 1000;
	always @(posedge clk) if (modulation_edge)
		cycle_timer <= state_tx_end ? 1'b0 : cycle_timer + 1'b1;

	// Loop through all transmit channels, switching right after state_tx_end
	reg [3:0] current_tx = 0;
	wire state_tx_end_last = state_tx_end && current_tx == kTxCount-1;
	always @(posedge clk)
		if (state_tx_end_last)
			current_tx <= 0;
		else if (state_tx_end)
			current_tx <= current_tx + 1'b1;

	// Light up one transmitter at a time, with amplitude modulation, during state_tx_modulating
	always @(posedge clk)
		ir_tx <= (state_tx_modulating && modulation_state) << current_tx;
		
	// The counters sum positive clock cycles on that channel, during the non-masked part of the cycle,
	// then latch those values at the end of the cycle. Latched values are valid throughout the following cycle.
	reg [kRxTimerBits-1:0] rx_counter[0:kRxCount-1];
	reg [kRxTimerBits-1:0] rx_latch[0:kRxCount-1];	
	genvar rx_lat_n;
	generate
		for (rx_lat_n = 0; rx_lat_n < kRxCount; rx_lat_n = rx_lat_n + 1) begin: rx_lat
			always @(posedge clk) begin
				if (state_tx_end) begin
					rx_latch[rx_lat_n] <= rx_counter[rx_lat_n];
					rx_counter[rx_lat_n] <= 0;
				end
				else if (!state_rx_masking && !ir_rx[rx_lat_n]) begin
					// Non-masked pulse from detector
					rx_counter[rx_lat_n] <= rx_counter[rx_lat_n] + 1'b1;
				end
			end
		end
	endgenerate

	// Keep track of the previous transmitter, corresponding with rx_latch
	reg [3:0] latched_tx = 0;
	always @(posedge clk) if (state_tx_end)
		latched_tx <= current_tx;

	// Tiny baud rate generator
	reg [15:0] baud_timer = 0;
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
	reg [7:0] async_tx_next_byte = 0;  // Input
	reg async_tx_next_byte_ready = 0;  // Input
	always @(posedge clk) begin
		txd <= kSerialInvert[0] ^ (async_tx_busy ? async_tx_reg[0] : 1'b1);
		if (baud_edge) begin
			if (!async_tx_busy && async_tx_next_byte_ready) begin
				async_tx_reg <= { 1'b1, async_tx_next_byte, 1'b0 };
				async_tx_state <= 4'h0;
				async_tx_busy <= 1'b1;
			end
			else if (async_tx_busy) begin
				async_tx_reg <= { 1'b1, async_tx_reg[9:1] };
				async_tx_state <= async_tx_state + 1'b1;
				async_tx_busy <= async_tx_state != 9;
			end
		end
	end
	
	// Transmit data back as simple ASCII packets that compromise
	// between readability and compactness. Each TX is one line
	// ending with a "\n". The line begins with a lowercase letter
	// identifying the transmitter, followed by groups of capital
	// hex digits carrying the timings from each receiver.
	
	reg [1:0] text_state = 0;
	reg [2:0] text_digit = 0;
	reg [4:0] text_rx = 0;
	wire [kRxTimerBits-1:0] current_latch = rx_latch[text_rx];
	wire [3:0] current_nybble = current_latch[4*text_digit +: 4];
	wire [7:0] current_hex_digit = 5;//current_nybble < 10 ? "0" + current_nybble : ("A" - 8'd10) + current_nybble;

	always @(posedge clk) case (text_state)
	
		// Idle, wait for a TX cycle to end
		0: begin
			async_tx_next_byte_ready <= 0;
			if (state_tx_end)
				text_state <= 3;
		end

		// Send the TX identity letter
		1: if (async_tx_busy)
			async_tx_next_byte_ready <= 0;
		else begin
			async_tx_next_byte_ready <= 1;
			async_tx_next_byte <= "a" + latched_tx;
			text_state <= 2;
			text_digit <= 0;
			text_rx <= 0;
		end
	
		// Send RX timer digits until we run out
		2: if (async_tx_busy)
			async_tx_next_byte_ready <= 0;
		else begin
			async_tx_next_byte_ready <= 1;
			async_tx_next_byte <= current_hex_digit;

			if (text_digit + 1 != kRxTimerNybbles)
				text_digit <= text_digit + 1'b1;
			else begin
				text_digit <= 0;
				if (text_rx + 1 != kRxCount)
					text_rx <= text_rx + 1'b1;
				else begin
					text_rx <= 0;
					text_state <= 3;
				end
			end
		end

		// Newline
		3: if (async_tx_busy)
			async_tx_next_byte_ready <= 0;
		else begin
			async_tx_next_byte_ready <= 1;
			async_tx_next_byte <= "\n";
			text_state <= 0;
		end

	endcase

endmodule
