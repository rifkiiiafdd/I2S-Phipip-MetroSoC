// Passive checks shared by the full SoC fixture and the local APB RTL test.
// Only Philips I2S, master, stereo, 16-bit data/slots, POL=0, MSB first.
`timescale 1ns/1ps
module i2s_audio_serial_checker #(
    parameter realtime EXPECTED_PCLK_PERIOD_NS = 0.0
) (
    input wire pclk, reset_n, active,
    input wire [15:0] divider,
    input wire sck, ws, sd,
    output integer words_checked = 0,
    output integer sck_checks = 0,
    output integer ws_checks = 0,
    output integer frame_checks = 0
);
  integer cycle = 0, last_edge = 0, last_ws = 0, last_left = 0;
  integer rises = 0, bits_seen = 0;
  reg previous_active = 0, previous_sck = 0, previous_ws = 1;
  reg sampled_ws = 1, collecting = 0, edge_seen = 0, ws_seen = 0, left_seen = 0;
  reg [15:0] decoded = 0;
  realtime last_pclk_time = 0, pclk_period_ns = 0;
  realtime session_pclk_period_ns = 0;
  realtime sck_half_ns = 0, ws_half_ns = 0, frame_ns = 0;
  realtime last_edge_time = 0, last_ws_time = 0, last_left_time = 0;

  function automatic [15:0] sample(input integer index);
    sample = (32'h96ab ^ (index * 32'h137)) & 32'hffff;
  endfunction

  // Sample pins just after the PCLK NBA updates. This avoids a race between
  // SCK and WS generated on the same PCLK, while staying well before the
  // next PCLK on which this RTL's serializer changes SD.
  always @(posedge pclk) begin
    #0.001;
    cycle = cycle + 1;
    if (last_pclk_time != 0) pclk_period_ns = $realtime - last_pclk_time;
    last_pclk_time = $realtime;
    if (!reset_n) begin
      previous_active = 0;
    end else if (active) begin
      if ((^{sck, ws, sd, divider}) === 1'bx)
        $fatal(1, "I2S_AUDIO_FAIL: X/Z on serial pins or DIV: sck=%b ws=%b sd=%b DIV=%h active=%b reset_n=%b scope=%m",
               sck, ws, sd, divider, active, reset_n);
      if (EXPECTED_PCLK_PERIOD_NS > 0 &&
          (pclk_period_ns < EXPECTED_PCLK_PERIOD_NS - 0.002 ||
           pclk_period_ns > EXPECTED_PCLK_PERIOD_NS + 0.002))
        $fatal(1, "I2S_AUDIO_FAIL: PCLK period=%0.3f ns expected=%0.3f ns",
               pclk_period_ns, EXPECTED_PCLK_PERIOD_NS);
      if (!previous_active) begin
        session_pclk_period_ns = pclk_period_ns;
        words_checked = 0; sck_checks = 0; ws_checks = 0; frame_checks = 0;
        edge_seen = 0; ws_seen = 0; left_seen = 0;
        previous_sck = sck; previous_ws = ws; sampled_ws = ws;
        collecting = 0; bits_seen = 0; decoded = 0; rises = 0;
      end else begin
        if (pclk_period_ns < session_pclk_period_ns - 0.002 ||
            pclk_period_ns > session_pclk_period_ns + 0.002)
          $fatal(1, "I2S_AUDIO_FAIL: PCLK changed/gated during stream");
        if (sck != previous_sck) begin
          if (edge_seen) begin
            if (cycle - last_edge != int'(divider) + 1)
              $fatal(1, "I2S_AUDIO_FAIL: SCK half-period=%0d PCLK cycles expected=%0d",
                     cycle-last_edge, int'(divider)+1);
            sck_checks = sck_checks + 1;
            sck_half_ns = $realtime - last_edge_time;
          end
          edge_seen = 1; last_edge = cycle; last_edge_time = $realtime;
        end
        if (ws != previous_ws) begin
          if (!(previous_sck === 1'b1 && sck === 1'b0))
            $fatal(1, "I2S_AUDIO_FAIL: WS must change on falling SCK");
          if (ws_seen) begin
            if (cycle-last_ws != 32 * (int'(divider)+1) || rises != 16)
              $fatal(1, "I2S_AUDIO_FAIL: WS half-period cycles=%0d SCK rises=%0d expected=%0d/16",
                     cycle-last_ws, rises, 32*(int'(divider)+1));
            ws_checks = ws_checks + 1;
            ws_half_ns = $realtime-last_ws_time;
          end
          ws_seen = 1; rises = 0; last_ws = cycle; last_ws_time = $realtime;
          if (!ws) begin
            if (left_seen) begin
              if (cycle-last_left != 64 * (int'(divider)+1))
                $fatal(1, "I2S_AUDIO_FAIL: stereo frame period incorrect");
              frame_checks = frame_checks + 1;
              frame_ns = $realtime-last_left_time;
            end
            left_seen = 1; last_left = cycle; last_left_time = $realtime;
          end
        end
        if (sck && !previous_sck) begin
          rises = rises + 1;
          // First rising SCK after WS changes carries the previous LSB.
          // Capture that bit BEFORE starting the next channel's word.
          if (collecting && bits_seen < 16) begin
            decoded = {decoded[14:0], sd};
            bits_seen = bits_seen + 1;
            if (bits_seen == 16) begin
              if (decoded !== sample(words_checked))
                $fatal(1, "I2S_AUDIO_FAIL: TX serial word=%0d actual=%04h expected=%04h",
                       words_checked, decoded, sample(words_checked));
              words_checked = words_checked + 1;
            end
          end
          if (ws != sampled_ws) begin
            if (collecting && bits_seen != 16)
              $fatal(1, "I2S_AUDIO_FAIL: wrong bit count at channel boundary");
            if (!ws || collecting) begin
              if (ws !== (words_checked % 2 != 0))
                $fatal(1, "I2S_AUDIO_FAIL: stereo channel order is not left/right");
              collecting = 1; bits_seen = 0; decoded = 0;
            end
            sampled_ws = ws;
          end
        end
        previous_sck = sck; previous_ws = ws;
      end
      previous_active = 1;
    end else previous_active = 0;
  end
endmodule

module i2s_audio_test_monitor #(
    parameter realtime EXPECTED_PCLK_PERIOD_NS = 0.0
) (
    input wire pclk, reset_n,
    input wire psel, penable, pwrite, pready, pslverr,
    input wire [11:0] paddr,
    input wire [31:0] pwdata, prdata,
    input wire [3:0] pstrb,
    input wire sck, ws, sd,
    output reg done = 0
);
  reg active = 0;
  reg [15:0] divider = 0;
  integer started = 0, completed = 0, tx_written = 0, rx_read = 0;
  integer error_reads = 0, total_tx_checked = 0, total_rx_checked = 0;
  wire [31:0] words_checked, sck_checks, ws_checks, frame_checks;
  i2s_audio_serial_checker #(.EXPECTED_PCLK_PERIOD_NS(EXPECTED_PCLK_PERIOD_NS)) serial (
      .pclk(pclk), .reset_n(reset_n), .active(active), .divider(divider),
      .sck(sck), .ws(ws), .sd(sd), .words_checked(words_checked),
      .sck_checks(sck_checks), .ws_checks(ws_checks), .frame_checks(frame_checks)
  );
  function automatic [31:0] sample(input integer index);
    sample = (32'h96ab ^ (index * 32'h137)) & 32'hffff;
  endfunction

  always @(posedge pclk) begin
    if (!reset_n) begin
      active <= 0; done <= 0; divider = 0;
      started = 0; completed = 0; tx_written = 0; rx_read = 0;
      error_reads = 0; total_tx_checked = 0; total_rx_checked = 0;
    end else if (psel === 1'b1 && penable === 1'b1 && pready === 1'b1) begin
      if ((^{pwrite, paddr, pslverr}) === 1'bx || pslverr !== 1'b0)
        $fatal(1, "I2S_AUDIO_FAIL: APB error or X/Z control");
      if (pwrite && ((^pwdata) === 1'bx || pstrb !== 4'hf))
        $fatal(1, "I2S_AUDIO_FAIL: invalid write data/strobe");
      if (!pwrite && (^prdata) === 1'bx)
        $fatal(1, "I2S_AUDIO_FAIL: X/Z read data");
      if (pwrite) begin
        case (paddr)
          12'h000: begin
            if (pwdata == 32'h5008) begin
              if (active) $fatal(1, "I2S_AUDIO_FAIL: premature flush");
              tx_written = 0; rx_read = 0; error_reads = 0;
            end else if (pwdata == 32'h5001) begin
              if (active || started != completed || started >= 2 || tx_written != 48)
                $fatal(1, "I2S_AUDIO_FAIL: invalid start/prefill");
              if (divider != (started == 0 ? 31 : 63))
                $fatal(1, "I2S_AUDIO_FAIL: firmware/testbench divider contract mismatch");
              started = started + 1; active <= 1;
            end else if (pwdata == 32'h5000) begin
              if (active) begin
                if (rx_read != 200 || words_checked < 200 || tx_written < 200 ||
                    error_reads == 0 || ws_checks < 199 || frame_checks < 99 || sck_checks < 6000)
                  $fatal(1, "I2S_AUDIO_FAIL: premature stop TXpin=%0d RXcpu=%0d TXwrites=%0d",
                         words_checked, rx_read, tx_written);
                completed = completed + 1;
                total_tx_checked = total_tx_checked + words_checked;
                total_rx_checked = total_rx_checked + rx_read;
                $display("I2S_AUDIO_SESSION_PASS session=%0d DIV=%0d TXpin=%0d RXcpu=%0d SCKchecks=%0d WSchecks=%0d frames=%0d",
                         completed, divider, words_checked, rx_read, sck_checks, ws_checks, frame_checks);
                $display("I2S_AUDIO_TIMING PCLK_ns=%0.3f SCK_half_ns=%0.3f WS_half_ns=%0.3f frame_ns=%0.3f Fs_Hz=%0.6f",
                         serial.pclk_period_ns, serial.sck_half_ns, serial.ws_half_ns,
                         serial.frame_ns, 1.0e9/serial.frame_ns);
                active <= 0;
              end
            end else $fatal(1, "I2S_AUDIO_FAIL: unexpected CTRL=%h", pwdata);
          end
          12'h004: begin
            if (pwdata == 32'hdead) $fatal(1, "I2S_AUDIO_FAIL: firmware reported failure");
            if (pwdata == 32'h600d) begin
              if (active || completed != 2 || total_rx_checked != 400)
                $fatal(1, "I2S_AUDIO_FAIL: premature firmware completion marker");
              done <= 1;
            end else begin
              if (active || (pwdata != 31 && pwdata != 63))
                $fatal(1, "I2S_AUDIO_FAIL: unsupported DIV or change while active");
              divider = pwdata[15:0];
            end
          end
          12'h008: begin
            if (pwdata !== sample(tx_written) || tx_written >= 232)
              $fatal(1, "I2S_AUDIO_FAIL: TX FIFO pattern/order wrong index=%0d", tx_written);
            tx_written = tx_written + 1;
          end
          12'h01c: if (pwdata != 0) $fatal(1, "I2S_AUDIO_FAIL: polling test expects ERRIE=0");
          default: $fatal(1, "I2S_AUDIO_FAIL: unexpected write offset=%h", paddr);
        endcase
      end else begin
        case (paddr)
          12'h00c: begin
            if (!active || rx_read >= 200 || prdata !== sample(rx_read))
              $fatal(1, "I2S_AUDIO_FAIL: RX CPU word=%0d actual=%08h expected=%08h",
                     rx_read, prdata, sample(rx_read));
            rx_read = rx_read + 1;
          end
          12'h014: if (prdata[31:16] > 64 || prdata[15:0] > 64)
            $fatal(1, "I2S_AUDIO_FAIL: FIFO count outside 0..64");
          12'h018: begin
            if (prdata != 0) $fatal(1, "I2S_AUDIO_FAIL: peripheral ERROR=%h", prdata);
            error_reads = error_reads + 1;
          end
          default: $fatal(1, "I2S_AUDIO_FAIL: unexpected read offset=%h", paddr);
        endcase
      end
    end
  end
endmodule
