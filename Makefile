VERILOG = Ws2811Test.v
PCF = ws2811.pcf

prog : bin/toplevel.bin
	cat bin/toplevel.bin >/dev/ttyACM0

bin/toplevel.json : ${VERILOG}
	mkdir -p bin
	yosys -v3 -p "synth_ice40 -json bin/toplevel.json" ${VERILOG}

bin/toplevel.asc : ${PCF} bin/toplevel.json
	nextpnr-ice40 --freq 50 --hx8k --package tq144:4k --json bin/toplevel.json --pcf ${PCF} --asc bin/toplevel.asc --opt-timing --placer heap

bin/toplevel.bin : bin/toplevel.asc
	icepack bin/toplevel.asc bin/toplevel.bin

compile : bin/toplevel.bin

time: bin/toplevel.bin
	icetime -tmd hx8k bin/toplevel.asc

clean :
	rm -rf bin
