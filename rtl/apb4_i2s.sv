// Copyright (c) 2023-2024 Miao Yuchi <miaoyuchi@ict.ac.cn>
// i2s is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//             http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

`include "register.sv"
`include "fifo.sv"
`include "i2s_define.sv"

module apb4_i2s #(
    parameter int FIFO_DEPTH = 64,
    // Defaults preserve the original interface-based regression. The M33
    // wrapper selects level interrupts and checked APB accesses.
    parameter bit LEVEL_IRQ = 1'b0,
    parameter bit STRICT_APB = 1'b0
) (
    apb4_if.slave apb4,
    i2s_if.dut    i2s,
    output logic tx_irq_o,
    output logic rx_irq_o,
    output logic error_irq_o
);

  localparam int LOG_FIFO_DEPTH = $clog2(FIFO_DEPTH);

  logic [3:0] s_apb4_addr;
  logic s_apb4_wr_hdshk, s_apb4_rd_hdshk;
  logic [`I2S_CTRL_WIDTH-1:0] s_i2s_ctrl_d, s_i2s_ctrl_q;
  logic s_i2s_ctrl_en;
  logic [`I2S_DIV_WIDTH-1:0] s_i2s_div_d, s_i2s_div_q;
  logic s_i2s_div_en;
  logic [`I2S_STAT_WIDTH-1:0] s_i2s_stat_d, s_i2s_stat_q;
  // bit
  logic s_bit_en, s_bit_txie, s_bit_rxie, s_bit_clr, s_bit_lsr;
  logic s_bit_pol, s_bit_lsb, s_bit_wm;
  logic [1:0] s_bit_fmt, s_bit_chl, s_bit_dtl, s_bit_chm;
  logic [4:0] s_bit_txth, s_bit_rxth;
  logic s_bit_txif, s_bit_rxif, s_busy, s_chd;
  // i2s
  logic s_i2s_mst_sck, s_i2s_mst_sck_trg;
  logic s_i2s_sck_trg, s_i2s_sck, s_i2s_slv_sck_trg;
  logic s_i2s_mst_ws, s_i2s_ws;
  // irq
  logic s_tx_irq_trg, s_rx_irq_trg;
  // fifo
  logic s_tx_push_valid, s_tx_push_ready, s_tx_empty, s_tx_full, s_tx_pop_valid, s_tx_pop_ready;
  logic s_rx_push_valid, s_rx_push_ready, s_rx_empty, s_rx_full, s_rx_pop_valid, s_rx_pop_ready;
  logic [31:0] s_tx_push_data, s_tx_pop_data, s_rx_push_data, s_rx_pop_data;
  logic [LOG_FIFO_DEPTH:0] s_tx_elem, s_rx_elem;
  logic s_core_en, s_apb_access, s_apb_bad_access;
  logic s_raw_write, s_raw_read;
  logic s_tx_underrun, s_rx_overrun;
  logic [3:0] s_error_q, s_errie_q, s_error_event, s_error_clear;

  initial begin
    if (FIFO_DEPTH < 2 || FIFO_DEPTH > 32768 ||
        (FIFO_DEPTH & (FIFO_DEPTH - 1)) != 0)
      $fatal(1, "I2S FIFO_DEPTH must be a power of two in [2, 32768]");
  end

  assign s_apb4_addr     = apb4.paddr[5:2];
  assign s_apb_access = apb4.psel && apb4.penable;
  assign s_raw_write = s_apb_access && apb4.pwrite;
  assign s_raw_read = s_apb_access && !apb4.pwrite;
  assign s_apb4_wr_hdshk = s_raw_write && !apb4.pslverr;
  assign s_apb4_rd_hdshk = s_raw_read && !apb4.pslverr;
  assign apb4.pready     = 1'b1;
  assign apb4.pslverr    = STRICT_APB && s_apb_access && s_apb_bad_access;

  // Never stall the CPU indefinitely on empty/full FIFO. Reject the access
  // with an APB error and leave FIFO/register contents unchanged instead.
  always_comb begin
    s_apb_bad_access = (|apb4.paddr[31:6]) || (|apb4.paddr[1:0]);
    if (apb4.pwrite && apb4.pstrb != 4'hf) s_apb_bad_access = 1;
    unique case (s_apb4_addr)
      `I2S_CTRL: begin
        // While enabled, software may stop, flush, mask interrupts or change
        // watermarks, but must stop before changing the serial format.
        if (apb4.pwrite && s_bit_en && apb4.pwdata[0] &&
            apb4.pwdata[15:4] != s_i2s_ctrl_q[15:4]) s_apb_bad_access = 1;
      end
      `I2S_DIV: if (apb4.pwrite && s_bit_en) s_apb_bad_access = 1;
      `I2S_TXR: if (!apb4.pwrite || (s_tx_full && !s_tx_pop_ready)) s_apb_bad_access = 1;
      `I2S_RXR: if (apb4.pwrite || s_rx_empty) s_apb_bad_access = 1;
      `I2S_STAT, `I2S_LEVEL, `I2S_IRQ: if (apb4.pwrite) s_apb_bad_access = 1;
      `I2S_ERROR, `I2S_ERRIE: begin end
      default: s_apb_bad_access = 1;
    endcase
  end

  assign s_bit_en        = s_i2s_ctrl_q[0];
  assign s_bit_txie      = s_i2s_ctrl_q[1];
  assign s_bit_rxie      = s_i2s_ctrl_q[2];
  assign s_bit_clr       = s_i2s_ctrl_q[3];
  assign s_bit_lsr       = s_i2s_ctrl_q[4];
  assign s_bit_pol       = s_i2s_ctrl_q[5];
  assign s_bit_lsb       = s_i2s_ctrl_q[6];
  assign s_bit_wm        = s_i2s_ctrl_q[7];
  assign s_bit_fmt       = s_i2s_ctrl_q[9:8];
  assign s_bit_chm       = s_i2s_ctrl_q[11:10];
  assign s_bit_chl       = s_i2s_ctrl_q[13:12];
  assign s_bit_dtl       = s_i2s_ctrl_q[15:14];
  assign s_bit_txth      = s_i2s_ctrl_q[20:16];
  assign s_bit_rxth      = s_i2s_ctrl_q[25:21];
  assign s_bit_txif      = s_i2s_stat_q[0];
  assign s_bit_rxif      = s_i2s_stat_q[1];
  assign s_core_en = s_bit_en && !(STRICT_APB && s_bit_clr);

  // i2s if
  assign i2s.mclk_o      = s_bit_lsr ? 1'b0 : apb4.pclk;
  assign i2s.sck_o       = s_bit_lsr ? 1'b0 : s_i2s_mst_sck;
  assign i2s.sck_en_o    = ~s_bit_lsr;
  assign i2s.ws_o        = s_bit_lsr ? 1'b0 : s_i2s_mst_ws;
  assign i2s.ws_en_o     = ~s_bit_lsr;
  // intern signals
  assign s_i2s_sck_trg   = s_bit_lsr ? s_i2s_slv_sck_trg : s_i2s_mst_sck_trg;
  assign s_i2s_sck       = s_bit_lsr ? i2s.sck_i : s_i2s_mst_sck;
  assign s_i2s_ws        = s_bit_lsr ? i2s.ws_i : s_i2s_mst_ws;
  // irq
  assign s_tx_irq_trg    = s_bit_txth > s_tx_elem;
  assign s_rx_irq_trg    = s_bit_rxth < s_rx_elem;
  assign tx_irq_o = LEVEL_IRQ ? (s_core_en && s_bit_txie && s_tx_irq_trg) : s_bit_txif;
  assign rx_irq_o = LEVEL_IRQ ? (s_core_en && s_bit_rxie && s_rx_irq_trg) : s_bit_rxif;
  assign error_irq_o = |(s_error_q & s_errie_q);
  assign i2s.irq_o = tx_irq_o | rx_irq_o | error_irq_o;

  // ERROR bits: 0=TX full write, 1=RX empty read, 2=RX lost word,
  // 3=TX empty channel. New errors dominate a simultaneous W1C write.
  assign s_error_event = {s_tx_underrun, s_rx_overrun,
      s_raw_read && s_apb4_addr == `I2S_RXR && s_rx_empty,
      s_raw_write && s_apb4_addr == `I2S_TXR && s_tx_full && !s_tx_pop_ready};
  assign s_error_clear = (s_apb4_wr_hdshk && s_apb4_addr == `I2S_ERROR) ?
      apb4.pwdata[3:0] : 4'b0;
  always_ff @(posedge apb4.pclk or negedge apb4.presetn) begin
    if (!apb4.presetn) begin
      s_error_q <= 0;
      s_errie_q <= 0;
    end else begin
      if (s_bit_clr) s_error_q <= 0;
      else s_error_q <= (s_error_q & ~s_error_clear) | s_error_event;
      if (s_apb4_wr_hdshk && s_apb4_addr == `I2S_ERRIE)
        s_errie_q <= apb4.pwdata[3:0];
    end
  end

  cdc_sync #(
      .STAGE     (2),
      .DATA_WIDTH(1)
  ) u_ext_trg_cdc_sync (
      apb4.pclk,
      apb4.presetn,
      i2s.sck_i,
      s_i2s_slv_sck_trg
  );

  assign s_i2s_ctrl_en = (s_apb4_wr_hdshk && s_apb4_addr == `I2S_CTRL &&
      (STRICT_APB || !s_busy)) || (STRICT_APB && s_bit_clr);
  always_comb begin
    s_i2s_ctrl_d = apb4.pwdata[`I2S_CTRL_WIDTH-1:0];
    if (STRICT_APB && !(s_apb4_wr_hdshk && s_apb4_addr == `I2S_CTRL)) begin
      s_i2s_ctrl_d = s_i2s_ctrl_q;
      s_i2s_ctrl_d[3] = 0; // CLR is a pulse in the M33 profile.
    end
  end
  dffer #(`I2S_CTRL_WIDTH) u_i2s_ctrl_dffer (
      apb4.pclk,
      apb4.presetn,
      s_i2s_ctrl_en,
      s_i2s_ctrl_d,
      s_i2s_ctrl_q
  );

  assign s_i2s_div_en = s_apb4_wr_hdshk && s_apb4_addr == `I2S_DIV && ~s_busy;
  assign s_i2s_div_d  = apb4.pwdata[`I2S_DIV_WIDTH-1:0];
  dffer #(`I2S_DIV_WIDTH) u_i2s_div_dffer (
      apb4.pclk,
      apb4.presetn,
      s_i2s_div_en,
      s_i2s_div_d,
      s_i2s_div_q
  );

  always_comb begin
    s_tx_push_valid = 1'b0;
    s_tx_push_data  = '0;
    if (s_apb4_wr_hdshk && s_apb4_addr == `I2S_TXR) begin
      s_tx_push_valid = 1'b1;
      unique case (s_bit_dtl)
        `I2S_DAT_8_BITS:  s_tx_push_data = {apb4.pwdata[7:0], 24'b0};
        `I2S_DAT_16_BITS: s_tx_push_data = {apb4.pwdata[15:0], 16'b0};
        `I2S_DAT_24_BITS: s_tx_push_data = {apb4.pwdata[23:0], 8'b0};
        `I2S_DAT_32_BITS: s_tx_push_data = apb4.pwdata[31:0];
      endcase
    end
  end

  always_comb begin
    s_i2s_stat_d[5] = s_chd;
    s_i2s_stat_d[4] = s_rx_empty;
    s_i2s_stat_d[3] = s_tx_full;
    s_i2s_stat_d[2] = s_busy;
    if (LEVEL_IRQ) begin
      s_i2s_stat_d[1:0] = {rx_irq_o, tx_irq_o};
    end else if ((s_bit_txif || s_bit_rxif) && s_apb4_rd_hdshk && s_apb4_addr == `I2S_STAT) begin
      s_i2s_stat_d[1:0] = 2'b0;
    end else if (~s_bit_txif && s_bit_en && s_bit_txie && s_tx_irq_trg) begin
      s_i2s_stat_d[1:0] = {s_bit_rxif, 1'b1};
    end else if (~s_bit_rxif && s_bit_en && s_bit_rxie && s_rx_irq_trg) begin
      s_i2s_stat_d[1:0] = {1'b1, s_bit_txif};
    end else begin
      s_i2s_stat_d[1:0] = {s_bit_rxif, s_bit_txif};
    end
  end
  dffr #(`I2S_STAT_WIDTH) u_i2s_stat_dffr (
      apb4.pclk,
      apb4.presetn,
      s_i2s_stat_d,
      s_i2s_stat_q
  );

  always_comb begin
    s_rx_pop_ready = 1'b0;
    apb4.prdata    = '0;
    if (s_apb4_rd_hdshk) begin
      unique case (s_apb4_addr)
        `I2S_CTRL: apb4.prdata[`I2S_CTRL_WIDTH-1:0] = s_i2s_ctrl_q;
        `I2S_DIV:  apb4.prdata[`I2S_DIV_WIDTH-1:0] = s_i2s_div_q;
        `I2S_RXR: begin
          s_rx_pop_ready                  = 1'b1;
          apb4.prdata[`I2S_RXR_WIDTH-1:0] = s_rx_pop_data;
        end
        `I2S_STAT: begin
          apb4.prdata[`I2S_STAT_WIDTH-1:0] = s_i2s_stat_q;
          if (LEVEL_IRQ)
            apb4.prdata[5:0] = {s_chd, s_rx_empty, s_tx_full, s_busy, rx_irq_o, tx_irq_o};
        end
        `I2S_LEVEL: apb4.prdata = {16'(s_rx_elem), 16'(s_tx_elem)};
        `I2S_ERROR: apb4.prdata[3:0] = s_error_q;
        `I2S_ERRIE: apb4.prdata[3:0] = s_errie_q;
        `I2S_IRQ: apb4.prdata[5:0] = {s_error_q & s_errie_q, rx_irq_o, tx_irq_o};
        default: begin
          s_rx_pop_ready = 1'b0;
          apb4.prdata    = '0;
        end
      endcase
    end
  end

  i2s_clkgen u_i2s_clkgen (
      .clk_i    (apb4.pclk),
      .rst_n_i  (apb4.presetn),
      .en_i     (s_core_en),
      .pol_i    (s_bit_pol),
      .chm_i    (s_bit_chm),
      .chl_i    (s_bit_chl),
      .div_i    (s_i2s_div_q),
      .sck_o    (s_i2s_mst_sck),
      .sck_trg_o(s_i2s_mst_sck_trg),
      .ws_o     (s_i2s_mst_ws)
  );

  assign s_tx_push_ready = ~s_tx_full;
  assign s_tx_pop_valid  = ~s_tx_empty;
  fifo #(
      .DATA_WIDTH  (32),
      .BUFFER_DEPTH(FIFO_DEPTH)
  ) u_tx_fifo (
      .clk_i  (apb4.pclk),
      .rst_n_i(apb4.presetn),
      .flush_i(s_bit_clr),
      .cnt_o  (s_tx_elem),
      .push_i (s_tx_push_valid),
      .full_o (s_tx_full),
      .dat_i  (s_tx_push_data),
      .pop_i  (s_tx_pop_ready),
      .empty_o(s_tx_empty),
      .dat_o  (s_tx_pop_data)
  );

  // A CPU pop frees a slot on this same edge. Match the FIFO's acceptance
  // rule so the core acknowledges a pending word exactly once, even full.
  assign s_rx_push_ready = ~s_rx_full || s_rx_pop_ready;
  assign s_rx_pop_valid  = ~s_rx_empty;
  fifo #(
      .DATA_WIDTH  (32),
      .BUFFER_DEPTH(FIFO_DEPTH)
  ) u_rx_fifo (
      .clk_i  (apb4.pclk),
      .rst_n_i(apb4.presetn),
      .flush_i(s_bit_clr),
      .cnt_o  (s_rx_elem),
      .push_i (s_rx_push_valid),
      .full_o (s_rx_full),
      .dat_i  (s_rx_push_data),
      .pop_i  (s_rx_pop_ready),
      .empty_o(s_rx_empty),
      .dat_o  (s_rx_pop_data)
  );

  i2s_core u_i2s_core (
      .clk_i        (apb4.pclk),
      .rst_n_i      (apb4.presetn),
      .en_i         (s_core_en),
      .lsb_i        (s_bit_lsb),
      .wm_i         (s_bit_wm),
      .fmt_i        (s_bit_fmt),
      .chm_i        (s_bit_chm),
      .chl_i        (s_bit_chl),
      .busy_o       (s_busy),
      .chd_o        (s_chd),
      .tx_valid_i   (s_tx_pop_valid),
      .tx_ready_o   (s_tx_pop_ready),
      .tx_data_i    (s_tx_pop_data),
      .rx_valid_o   (s_rx_push_valid),
      .rx_ready_i   (s_rx_push_ready),
      .rx_data_o    (s_rx_push_data),
      .tx_underrun_o (s_tx_underrun),
      .rx_overrun_o  (s_rx_overrun),
      .i2s_sck_i    (s_i2s_sck),
      .i2s_sck_trg_i(s_i2s_sck_trg),
      .i2s_ws_i     (s_i2s_ws),
      .i2s_sd_o     (i2s.sd_o),
      .i2s_sd_i     (i2s.sd_i)
  );
endmodule
