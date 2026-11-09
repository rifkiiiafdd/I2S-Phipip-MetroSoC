// Flat APB4 peripheral for a Cortex-M33 AHB-to-APB subsystem.
// PADDR is a BYTE offset, e.g. 12'h00c for RXR. Do not pass PADDR[11:2].
// PCLK must be free-running while I2S is enabled; no APB-idle clock gating.
`include "i2s_define.sv"

module cmsdk_apb_i2s #(
    parameter int ADDR_WIDTH = 12,
    parameter int FIFO_DEPTH = 64
) (
    input  wire PCLK,
    input  wire PRESETn,
    input  wire PSEL,
    input  wire [ADDR_WIDTH-1:0] PADDR,
    input  wire PENABLE,
    input  wire PWRITE,
    input  wire [31:0] PWDATA,
    input  wire [3:0] PSTRB,
    input  wire [2:0] PPROT,
    output wire [31:0] PRDATA,
    output wire PREADY,
    output wire PSLVERR,
    input  wire I2S_SCKI,
    input  wire I2S_WSI,
    input  wire I2S_SDI,
    output wire I2S_MCLK,
    output wire I2S_SCKO,
    output wire I2S_SCKEN,
    output wire I2S_WSO,
    output wire I2S_WSEN,
    output wire I2S_SDO,
    output wire TXINT,
    output wire RXINT,
    output wire ERRINT,
    output wire I2SINT
);
  apb4_if apb(PCLK, PRESETn);
  i2s_if audio();
  initial begin
    if (ADDR_WIDTH < 6 || ADDR_WIDTH > 32)
      $fatal(1, "cmsdk_apb_i2s ADDR_WIDTH must be in [6, 32]");
  end
  assign apb.paddr = 32'(PADDR);
  assign apb.psel = PSEL;
  assign apb.penable = PENABLE;
  assign apb.pwrite = PWRITE;
  assign apb.pwdata = PWDATA;
  assign apb.pstrb = PSTRB;
  assign apb.pprot = PPROT;
  assign PRDATA = apb.prdata;
  assign PREADY = apb.pready;
  assign PSLVERR = apb.pslverr;
  assign audio.sck_i = I2S_SCKI;
  assign audio.ws_i = I2S_WSI;
  assign audio.sd_i = I2S_SDI;
  assign I2S_MCLK = audio.mclk_o;
  assign I2S_SCKO = audio.sck_o;
  assign I2S_SCKEN = audio.sck_en_o;
  assign I2S_WSO = audio.ws_o;
  assign I2S_WSEN = audio.ws_en_o;
  assign I2S_SDO = audio.sd_o;
  assign I2SINT = audio.irq_o;

  apb4_i2s #(.FIFO_DEPTH(FIFO_DEPTH), .LEVEL_IRQ(1'b1), .STRICT_APB(1'b1)) u_i2s (
      .apb4(apb), .i2s(audio),
      .tx_irq_o(TXINT), .rx_irq_o(RXINT), .error_irq_o(ERRINT)
  );
endmodule
