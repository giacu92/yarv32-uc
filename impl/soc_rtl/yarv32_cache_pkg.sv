`resetall
`timescale 1ns / 1ps
`default_nettype none

// Width-parametrizable native-protocol struct pair. SystemVerilog packages
// cannot parameterize typedefs, so these macros build the pair per
// (ADDR_W, DATA_W). Field order and semantics are fixed; only the widths
// vary:
//   REQ : valid, we, addr (byte address, ADDR_W bits), wdata (DATA_W bits),
//         wstrb (DATA_W/8 bits), rready
//   RSP : wready, rvalid, rdata (DATA_W bits), bvalid
// Expand into named typedefs at package or module scope
// (`YARV_MEM_TYPES), or directly in a port list (see native_ram, which
// re-expands with its own parameters). All struct-to-struct port
// connections are packed-vector assignments, so two expansions only
// connect cleanly when their widths match — an intentional width
// mismatch shows up as a WIDTH warning, not a type error.
`ifndef YARV_MEM_TYPES_SV
`define YARV_MEM_TYPES_SV
`define YARV_MEM_REQ_T(ADDR_W, DATA_W) \
    struct packed { \
        logic valid; \
        logic we; \
        logic [ADDR_W-1:0] addr; \
        logic [DATA_W-1:0] wdata; \
        logic [DATA_W/8-1:0] wstrb; \
        logic rready; \
    }
`define YARV_MEM_RSP_T(DATA_W) \
    struct packed { \
        logic wready; \
        logic rvalid; \
        logic [DATA_W-1:0] rdata; \
        logic bvalid; \
    }
`define YARV_MEM_TYPES(REQ_T, RSP_T, ADDR_W, DATA_W) \
    typedef `YARV_MEM_REQ_T(ADDR_W, DATA_W) REQ_T; \
    typedef `YARV_MEM_RSP_T(DATA_W) RSP_T;
`endif

package yarv32_cache_pkg;

    // CPU-side native protocol: 64-bit data, 64-bit byte address.
    localparam int unsigned MEM_WIDTH = 64;
    localparam int unsigned STRB_WIDTH = MEM_WIDTH / 8;

    // Cache-line variant: one whole cache line per RAM word (2^5 = 32 B
    // at CL_SIZE=5). Must stay consistent with cache_cntrl's
    // DATA_WIDTH = 2**(CL_SIZE+3).
    localparam int unsigned CACHE_WIDTH = 256;
    localparam int unsigned CACHE_STRB_WIDTH = CACHE_WIDTH / 8;

    // Native protocol, CPU width: fetch/LSU side, bootrom.
    `YARV_MEM_TYPES(mem_req_t, mem_rsp_t, MEM_WIDTH, MEM_WIDTH)

    // Native protocol, cache-line width: cache data macros.
    `YARV_MEM_TYPES(cache_req_t, cache_rsp_t, MEM_WIDTH, CACHE_WIDTH)

    // ------------------------------------------------------------------
    // System address map (24 bit). The SDRAM needs 23 bits for its 8 MiB,
    // so bit 23 is free and separates memory from everything else:
    //
    //   0x00_0000 - 0x7F_FFFF  SDRAM, 8 MiB, cacheable (or bypassed)
    //   0x80_0000 - 0x80_07FF  bootrom, 2 KiB, read-only
    //   0x80_1000              control register, 8 bit, read/write
    //
    // Only bits [23] and [12] are decoded, so the bootrom aliases through
    // 0x80_0000-0x80_0FFF and the register through 0x80_1000-0x80_1FFF.
    // Address bits above 23 are ignored.
    // ------------------------------------------------------------------
    localparam int unsigned SYS_ADDR_W = 24;
    localparam int unsigned SYS_MEM_BIT = 23;  // 0 = SDRAM, 1 = peripherals
    localparam int unsigned SYS_CSR_BIT = 12;  // within peripherals: 0 = rom, 1 = csr

    localparam logic [SYS_ADDR_W-1:0] BOOTROM_BASE = 24'h80_0000;
    localparam logic [SYS_ADDR_W-1:0] CSR_BASE = 24'h80_1000;

    // Bootrom depth: 2 KiB (ADDR_W of its native_ram instance).
    localparam int unsigned BOOTROM_ADDR_W = 11;

    // Control register: 8 bits, byte 0 of the addressed doubleword.
    //   [0] CACHE_BYPASS — SDRAM loads and stores go straight to the
    //       device, leaving the cache arrays untouched. Set it while a
    //       loader writes a program into SDRAM, so the program is really
    //       in the device (and not sitting dirty in the D-cache) when the
    //       fetch side goes looking for it. NOT a coherence mechanism:
    //       lines cached before the bit was set stay cached and stale.
    //   [7:1] unused, readable/writable scratch.
    localparam int unsigned CSR_W = 8;
    localparam int unsigned CSR_BIT_BYPASS = 0;

    // Address region a request falls in.
    localparam logic [1:0] RGN_MEM = 2'd0;
    localparam logic [1:0] RGN_BOOT = 2'd1;
    localparam logic [1:0] RGN_CSR = 2'd2;

    function automatic logic [1:0] yarv_region(input logic [MEM_WIDTH-1:0] a);
        if (!a[SYS_MEM_BIT]) yarv_region = RGN_MEM;
        else if (!a[SYS_CSR_BIT]) yarv_region = RGN_BOOT;
        else yarv_region = RGN_CSR;
    endfunction

endpackage

`resetall
