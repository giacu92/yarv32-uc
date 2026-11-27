# Linux SoC simulation (M5): the same core and Linux-branch peripherals as
# sim/sim_top.sv, but with the cache subproject + SDRAM controller + SDRAM
# model in place of the two BSRAMs. Reuses sim_main.cpp (built with
# -DSOC_BUILD).
#
#   make            build obj_soc/Vsim_top
#   make regress    run the oracle set on the SoC (images merged by mkimage.py)
#
# The cache sources are taken from the commit the superproject PINS for the
# submodule (git archive), not from the submodule's working tree, so the
# build is reproducible whatever is checked out there.
REPO      := $(abspath ../..)
CACHE_DIR := $(REPO)/src/sub/yarv32-native-caches
CACHE_REV := $(shell git -C $(REPO) ls-files -s src/sub/yarv32-native-caches | awk '{print $$2}')
SDRC_REV  := $(shell git -C $(CACHE_DIR) ls-tree $(CACHE_REV) src/ips/sdram-controller | awk '{print $$3}')
SNAP      := snap/$(CACHE_REV)

CORE := $(REPO)/src/rtl
# Linux-branch peripherals (keep in step with ../Makefile's RTL list).
PERI_RTL := $(CORE)/utils/clint.sv $(CORE)/utils/axi4_lite_uart16550.sv \
	$(CORE)/utils/axi4_lite_spi.sv $(CORE)/utils/axi4_lite_gpio.sv \
	$(CORE)/utils/axi4_lite_plic_sifive.sv

RTL := \
	$(CORE)/pkg/rv32_pkg.sv \
	$(SNAP)/src/rtl/pkg/yarv32_cache_pkg.sv \
	$(CORE)/bus/axi4_lite_if.sv \
	$(CORE)/bus/axi4_lite_master_bridge.sv \
	$(CORE)/bus/axi4_lite_xbar.sv \
	$(CORE)/core/fetch_stage.sv \
	$(CORE)/core/reg_file.sv \
	$(CORE)/core/csr_regfile.sv \
	$(CORE)/core/decode_stage.sv \
	$(CORE)/core/trap_unit.sv \
	$(CORE)/core/alu.sv \
	$(CORE)/core/branch_predictor.sv \
	$(CORE)/core/execute_stage.sv \
	$(CORE)/core/rv32imac_zicsr_zifencei.sv \
	$(CORE)/utils/dport_32to64.sv \
	$(SNAP)/src/rtl/native_ram.sv \
	$(SNAP)/src/rtl/cache_cntrl.sv \
	$(SNAP)/src/ips/sdram-controller/rtl/sdram_controller.v \
	$(SNAP)/sim/sdram_model.sv \
	$(PERI_RTL) \
	$(abspath ..)/spi_flash_model.sv \
	$(abspath ..)/sd_spi_model.sv \
	sim_soc.sv
TOP := obj_soc/Vsim_top
VFLAGS := --cc --exe --build --top-module sim_top -Mdir obj_soc -CFLAGS -DSOC_BUILD \
	-Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT -Wno-LATCH -Wno-INITIALDLY -Wno-TIMESCALEMOD \
	-Wno-MULTIDRIVEN -Wno-PINMISSING --public-flat-rw --trace \
	-I$(CORE)/pkg -I$(SNAP)/src/rtl/pkg

$(SNAP)/.done:
	@test -n "$(CACHE_REV)" || { echo "no pinned submodule commit"; exit 1; }
	mkdir -p $(SNAP)/src/ips/sdram-controller
	git -C $(CACHE_DIR) archive $(CACHE_REV) | tar -x -C $(SNAP)
	git -C $(CACHE_DIR)/src/ips/sdram-controller archive $(SDRC_REV) | tar -x -C $(SNAP)/src/ips/sdram-controller
	python3 qualify_pkg.py $(SNAP)/src/rtl/pkg/yarv32_cache_pkg.sv \
	    $(SNAP)/src/rtl/native_ram.sv $(SNAP)/src/rtl/cache_cntrl.sv
	touch $@

$(TOP): $(SNAP)/.done ../sim_main.cpp sim_soc.sv ../spi_flash_model.sv ../sd_spi_model.sv $(filter $(CORE)/%,$(RTL))
	verilator $(VFLAGS) ../sim_main.cpp $(RTL)

build: $(TOP)
clean:
	rm -rf obj_soc snap img linux_run linux_flash_run linux_sd_run

.PHONY: build clean

regress: $(TOP)
	python3 regress.py
.PHONY: regress
compliance: $(TOP)
	python3 compliance.py
.PHONY: compliance

# Boot the nommu kernel to the BusyBox shell (M10). Pass = the shell echoes
# BOOT-OK. Reports cycles to `Run /bin/sh` and to the first prompt, and the
# number of misaligned-access traps the kernel took. ~5-10 min of wall time.
LINUX_IMAGE ?= $(HOME)/yarv-linux/soc/Image
linux: $(TOP)
	python3 linux.py $(LINUX_IMAGE) linux_run
# Same, booted the board's way: M9 loader ROM, Image as a yload frame at
# 2 MiB in the SPI flash model. Copying 1.8 MB over SPI adds a long prefix.
linux-flash: $(TOP)
	python3 linux.py $(LINUX_IMAGE) linux_flash_run --flash
# The board's way since M9b (D42): Image as a yload frame at sector 2048 of
# the SPI-mode SD card model (+SDCARD). SD_ARGS=+SDSC for a byte-addressed card.
linux-sd: $(TOP)
	python3 linux.py $(LINUX_IMAGE) linux_sd_run --sd
.PHONY: linux linux-flash linux-sd
