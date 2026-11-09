# APB I2S Peripheral

SystemVerilog I2S transmit/receive peripheral for integration into an AHB/APB SoC. The peripheral is processor-independent: any bus master that can reach an APB slave through an AHB-to-APB bridge can access it. The bridge and interconnect are supplied by the integrating SoC.

## Integration

```text
Processor / bus master -> AHB interconnect -> AHB-to-APB bridge
                                               |
                                         cmsdk_apb_i2s -> Audio device
                                               |
                                         Interrupt controller
```

Use `rtl/cmsdk_apb_i2s.sv` as the integration top. Despite its name, it has no processor-specific interface. It wraps `apb4_i2s` with checked APB accesses and level-sensitive interrupts.

- Connect `PCLK`, active-low `PRESETn`, `PSEL`, `PENABLE`, `PWRITE`, `PADDR`, `PWDATA`, `PSTRB`, and `PPROT` to the bridge. Return `PRDATA`, `PREADY`, and `PSLVERR`; preserve error propagation back to AHB.
- Allocate an aligned peripheral window and pass the **local byte offset** on `PADDR` (`0x0C` for RXR). Do not pass a word index or the full system address. Registers occupy `0x00` through `0x20`; reserve at least 64 bytes. Other offsets within the window return an error.
- Use aligned 32-bit accesses. Writes require `PSTRB = 4'hF`. For an APB bridge without strobes, tie `PSTRB` high and restrict upstream writes to full words. `PPROT` is currently unused and may be tied to zero.
- `PREADY` is always high. Invalid accesses return `PSLVERR` without applying the requested operation. A TX write with no space available or an RX read with no data available also sets the corresponding ERROR flag; accesses do not wait for data or space.
- Keep `PCLK` running while audio is enabled, including when the APB bus is idle. The audio engine uses this clock. If AHB and APB use different clocks, provide a bridge that handles the clock-domain crossing.
- Route either combined `I2SINT` or separate `TXINT`, `RXINT`, and `ERRINT` to the interrupt controller, configured for active-high level interrupts.

Parameters: `ADDR_WIDTH` defaults to 12 (supported range 6–32); `FIFO_DEPTH` defaults to 64 words per direction and must be a power of two from 2 to 32768.

## Audio Connections and Clocking

Connect `I2S_SDO` to the audio device's data input and `I2S_SDI` to its data output. In master mode (`CTRL.LSR = 0`), drive the device from `I2S_SCKO` and `I2S_WSO`; `I2S_MCLK` equals `PCLK`. In slave mode (`LSR = 1`), supply external clocks on `I2S_SCKI` and `I2S_WSI`; MCLK is held low. `I2S_SCKEN` and `I2S_WSEN` are active-high output enables for clock pads or muxes.

In master stereo mode, for channel width `N`:

```text
BCLK = PCLK / (2 * (DIV + 1))
sample_rate = BCLK / (2 * N)
```

For example, `PCLK = 12.288 MHz`, `DIV = 3`, and `N = 32` produce 1.536 MHz BCLK and 24 kHz stereo frames. Check codec clock requirements and serial timing in the target implementation. Slave clocks are sampled in the `PCLK` domain; external timing and clock-domain constraints must be validated.

## Register Map

All offsets are relative to the assigned peripheral base address. Reserved bits should be written as zero.

| Offset | Register | Access | Description |
|---|---|---|---|
| `0x00` | CTRL | R/W | Enable, audio configuration, FIFO clear, and interrupt thresholds |
| `0x04` | DIV | R/W | BCLK divider, bits `[15:0]`; write only while disabled |
| `0x08` | TXR | W | Push one channel sample into the TX FIFO |
| `0x0C` | RXR | R | Pop one channel sample from the RX FIFO |
| `0x10` | STAT | R | `[5]` channel indication, `[4]` RX empty, `[3]` TX full, `[2]` busy, `[1]` RX IRQ, `[0]` TX IRQ |
| `0x14` | LEVEL | R | RX word count `[31:16]`, TX word count `[15:0]` |
| `0x18` | ERROR | R/W1C | Sticky errors: `[0]` TX full write, `[1]` RX empty read, `[2]` RX word lost, `[3]` TX underrun |
| `0x1C` | ERRIE | R/W | Error interrupt enables corresponding to ERROR bits `[3:0]` |
| `0x20` | IRQ | R | Live causes: `[0]` TX, `[1]` RX, `[5:2]` enabled error flags |

CTRL fields:

| Bits | Field | Meaning |
|---|---|---|
| `0` | EN | Enable audio engine |
| `1`, `2` | TXIE, RXIE | Enable TX/RX threshold interrupts |
| `3` | CLR | Self-clearing pulse; flush both FIFOs, clear errors, and reset serial state |
| `4` | LSR | `0`: master; `1`: slave |
| `5` | POL | Clock polarity control |
| `6` | LSB | `0`: MSB first; `1`: LSB first |
| `7` | WM | `0`: normal operation; `1`: serial input-to-output bypass |
| `[9:8]` | FMT | Use `00` for Philips I2S; alternate format encodings are not implemented |
| `[11:10]` | CHM | `00`: stereo; `01`: left only; `10`: right only |
| `[13:12]` | CHL | Channel width: `00/01/10/11` = 8/16/24/32 bits |
| `[15:14]` | DTL | TX data width, same encoding as CHL; use matching widths |
| `[20:16]` | TXTH | TX IRQ when TX word count is **less than** TXTH |
| `[25:21]` | RXTH | RX IRQ when RX word count is **greater than** RXTH |

TX samples are written right-aligned in TXR; RX samples are read right-aligned from RXR. Each FIFO entry holds one channel sample. Stereo order starts with left, then right. Thresholds are limited to 0–31 regardless of FIFO depth.

## Software Sequence

1. Clear EN, then configure CTRL and DIV. Change serial format fields `[15:4]` only while disabled.
2. Pulse CLR with the intended CTRL configuration, then preload TXR if transmitting. Check LEVEL before FIFO accesses.
3. Set error masks in ERRIE and threshold enables in CTRL as needed, then set EN.
4. Service TX/RX by filling or draining the FIFOs. Reading STAT or IRQ does not acknowledge level interrupts: service the threshold condition or mask its enable. Clear handled ERROR flags by writing ones to those bits.
5. Clear EN before reconfiguration or shutdown. Stopping resets serial state; use CLR when buffered samples must also be discarded.

TX and RX share EN. Receive-only operation can therefore raise TX underrun; enable only the error interrupts relevant to the application.

## Build Requirements

Compile `i2s_clkgen.sv`, `i2s_core.sv`, `apb4_i2s.sv`, and `cmsdk_apb_i2s.sv` with SystemVerilog interface support and `rtl/` on the include path. Compile the `apb4_if` interface definition before modules that use it.

The shared RTL dependencies are provided in `common/`; the required Vivado files are listed below. The testbench in `tb/` also requires pad models and test infrastructure. Validate bridge decoding, error propagation, FIFO handling, interrupts, and audio timing in the integrated SoC. Direct instantiation of `apb4_i2s` uses different defaults (`STRICT_APB = 0`, `LEVEL_IRQ = 0`); this README describes the wrapper configuration.

### Vivado: Required Files from `common/`

Paths below are relative to this project folder.

| File | Purpose | Vivado handling |
|---|---|---|
| `common/rtl/interface/apb4_if.sv` | APB4 interface definition | Add to **Design Sources** as SystemVerilog; compile before the I2S modules |
| `common/rtl/cdc/cdc_sync.sv` | External clock input synchronizer | Add to **Design Sources** as SystemVerilog |
| `common/rtl/utils/register.sv` | Register primitives (`dffr`, `dffer`) | Include-only; already included by `apb4_i2s.sv` |
| `common/rtl/utils/fifo.sv` | TX/RX FIFO implementation | Include-only; already included by `apb4_i2s.sv` |
| `common/rtl/utils/edge_det.sv` | Serial clock edge detector | Include-only; already included by `i2s_core.sv` |
| `common/rtl/config.svh` | Shared configuration header | Include-only; included by `register.sv` |
| `common/rtl/utils/xchecker.sv` | Assertions used by some register primitives | Add as a SystemVerilog source when those primitives are elaborated with assertions enabled; not required by the I2S `dffr`/`dffer` instances |

Add the four I2S module files listed above to **Design Sources** as SystemVerilog. Keep `rtl/i2s_define.sv` available as an include-only file. In **Settings > General > Verilog options > Include directories**, add:

```text
rtl/
common/rtl/
common/rtl/utils/
common/rtl/interface/
common/rtl/cdc/
```

Resolve these paths from the project source folder, or use absolute paths if the Vivado project is stored elsewhere. Apply the same include directories to the simulation fileset when running simulation.

For include-only files, make them available through the include directories and avoid compiling them again as independent design sources: `register.sv`, `fifo.sv`, and `edge_det.sv` contain module definitions without include guards. Do not mark them as global includes. Use `cmsdk_apb_i2s` as the top for standalone peripheral elaboration, or instantiate it under the SoC top and update the compile order. `common/rtl/tech/gpio_pad.sv` is used by the supplied pad-based testbench; it is not required for the flat wrapper's synthesis.

## License

The original RTL copyright notices specify Mulan PSL v2. Preserve the notices and follow the license terms when redistributing or modifying the source.
