// Copyright (c) 2023-2024 Miao Yuchi <miaoyuchi@ict.ac.cn>
// i2s is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//             http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.


`include "edge_det.sv"
`include "i2s_define.sv"

module i2s_core (
    input  logic                       clk_i,
    input  logic                       rst_n_i,
    input  logic                       en_i,
    input  logic                       lsb_i,
    input  logic                       wm_i,
    input  logic [                1:0] fmt_i,
    input  logic [                1:0] chm_i,
    input  logic [                1:0] chl_i,
    output logic                       busy_o,
    output logic                       chd_o,
    input  logic                       tx_valid_i,
    output logic                       tx_ready_o,
    input  logic [`I2S_DATA_WIDTH-1:0] tx_data_i,
    output logic                       rx_valid_o,
    input  logic                       rx_ready_i,
    output logic [`I2S_DATA_WIDTH-1:0] rx_data_o,
    output logic                       tx_underrun_o,
    output logic                       rx_overrun_o,
    input  logic                       i2s_sck_i,
    input  logic                       i2s_sck_trg_i,
    input  logic                       i2s_ws_i,
    output logic                       i2s_sd_o,
    input  logic                       i2s_sd_i
);

  logic s_sck_re;
  logic s_ws_q, s_tx_synced_q, s_rx_synced_q, s_tx_active_q, s_rx_active_q;
  logic [31:0] s_tx_shift_q, s_rx_shift_q, s_rx_data_q;
  logic [31:0] s_rx_next, s_word_mask;
  logic [5:0] s_word_bits, s_tx_count_q, s_rx_count_q;
  logic s_rx_valid_q, s_ws_change, s_channel_selected, s_start_channel;

  edge_det_sync_re #(.DATA_WIDTH(1)) u_sck_edge_det_sync_re (
      .clk_i(clk_i), .rst_n_i(rst_n_i), .dat_i(i2s_sck_i), .re_o(s_sck_re)
  );

  assign s_word_bits = 6'd8 + {chl_i, 3'b000};
  assign s_word_mask = 32'hffffffff >> (32 - s_word_bits);
  assign s_ws_change = i2s_ws_i != s_ws_q;
  assign s_channel_selected = (chm_i == `I2S_CHM_STERO) ||
      (chm_i == `I2S_CHM_LEFT && !i2s_ws_i) ||
      (chm_i == `I2S_CHM_RIGHT && i2s_ws_i);
  // Stereo FIFO words begin with left. Right-only mode begins with right.
  assign s_start_channel = (chm_i == `I2S_CHM_RIGHT) ? i2s_ws_i : !i2s_ws_i;
  assign tx_ready_o = en_i && wm_i == `I2S_WM_NORM && s_sck_re &&
      s_ws_change && s_channel_selected && (s_tx_synced_q || s_start_channel);
  assign busy_o = en_i && s_tx_active_q;
  assign chd_o = s_ws_q;
  assign rx_valid_o = s_rx_valid_q;
  assign rx_data_o = s_rx_data_q;
  assign i2s_sd_o = wm_i == `I2S_WM_TEST ? i2s_sd_i :
      (lsb_i ? s_tx_shift_q[0] : s_tx_shift_q[s_word_bits - 1'b1]);

  always_comb begin
    if (lsb_i)
      s_rx_next = (s_rx_shift_q >> 1) |
          (32'(i2s_sd_i) << (s_word_bits - 1'b1));
    else
      s_rx_next = ((s_rx_shift_q << 1) | {31'b0, i2s_sd_i}) & s_word_mask;
  end

  always_ff @(posedge clk_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
      s_ws_q <= 0;
      s_tx_synced_q <= 0;
      s_rx_synced_q <= 0;
      s_tx_active_q <= 0;
      s_tx_shift_q <= 0;
      s_tx_count_q <= 0;
      s_rx_active_q <= 0;
      s_rx_shift_q <= 0;
      s_rx_count_q <= 0;
      s_rx_data_q <= 0;
      s_rx_valid_q <= 0;
      tx_underrun_o <= 0;
      rx_overrun_o <= 0;
    end else if (!en_i) begin
      s_ws_q <= i2s_ws_i;
      s_tx_synced_q <= 0;
      s_rx_synced_q <= 0;
      s_tx_active_q <= 0;
      s_tx_shift_q <= 0;
      s_tx_count_q <= 0;
      s_rx_active_q <= 0;
      s_rx_shift_q <= 0;
      s_rx_count_q <= 0;
      s_rx_valid_q <= 0;
      tx_underrun_o <= 0;
      rx_overrun_o <= 0;
    end else begin
      tx_underrun_o <= 0;
      rx_overrun_o <= 0;
      // Keep a completed receive word stable until the FIFO accepts it.
      if (s_rx_valid_q && rx_ready_i) s_rx_valid_q <= 0;
      if (!tx_valid_i && !s_tx_active_q) s_tx_synced_q <= 0;
      if (s_sck_re) begin
        // The first rising SCK with new WS still carries the previous LSB.
        // Finish the previous receive word before starting the new channel.
        if (s_rx_active_q && s_rx_count_q < s_word_bits) begin
          s_rx_shift_q <= s_rx_next;
          s_rx_count_q <= s_rx_count_q + 1'b1;
          if (s_rx_count_q == s_word_bits - 1'b1) begin
            s_rx_active_q <= 0;
            if (!s_rx_valid_q || rx_ready_i) begin
              s_rx_data_q <= s_rx_next;
              s_rx_valid_q <= 1;
            end else begin
              // Serial input cannot be stalled: preserve the pending word
              // and report that this newly completed word was dropped.
              rx_overrun_o <= 1;
            end
          end
        end

        if (s_ws_change) begin
          if (tx_ready_o && !tx_valid_i) tx_underrun_o <= 1;
          s_ws_q <= i2s_ws_i;
          if (tx_ready_o && tx_valid_i) s_tx_synced_q <= 1;
          else if (!tx_valid_i) s_tx_synced_q <= 0;
          if (s_start_channel) s_rx_synced_q <= 1;
          s_tx_count_q <= 0;
          s_tx_active_q <= tx_ready_o && tx_valid_i;
          // Present the new MSB after the delay-bit sampling edge.
          if (tx_ready_o && tx_valid_i)
            s_tx_shift_q <= (tx_data_i >> (32 - s_word_bits)) & s_word_mask;
          else
            s_tx_shift_q <= 0;
          s_rx_shift_q <= 0;
          s_rx_count_q <= 0;
          s_rx_active_q <= wm_i == `I2S_WM_NORM && s_channel_selected &&
              (s_rx_synced_q || s_start_channel);
        end else if (s_tx_active_q) begin
          s_tx_count_q <= s_tx_count_q + 1'b1;
          if (lsb_i) s_tx_shift_q <= s_tx_shift_q >> 1;
          else s_tx_shift_q <= (s_tx_shift_q << 1) & s_word_mask;
          if (s_tx_count_q == s_word_bits - 1'b1) s_tx_active_q <= 0;
        end
      end
    end
  end
endmodule
