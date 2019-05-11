

VERILOG = ../../../MuraxArduino.v toplevel.v toplevel_pll.v

generate :
	(cd ../../..; sbt "runMain vexriscv.demo.MuraxBlackiceTest")

../../../MuraxArduino.v :
	(cd ../../..; sbt "runMain vexriscv.demo.MuraxBlackiceTest")

../../../MuraxArduino.v*.bin:

bin/toplevel.json : ${VERILOG} ../../../MuraxArduino.v*.bin
	mkdir -p bin
	rm -f Murax.v*.bin
	cp ../../../MuraxArduino.v*.bin . | true
	yosys -v3 -p "synth_ice40 -top toplevel -json bin/toplevel.json" ${VERILOG}

#bin/toplevel.asc : toplevel.pcf bin/toplevel.blif
#	arachne-pnr -p toplevel.pcf -d 8k --max-passes 600 -P tq144:4k bin/toplevel.blif -o bin/toplevel.asc

bin/toplevel.asc : toplevel.pcf bin/toplevel.json
	nextpnr-ice40 --freq 43 --hx8k --package tq144:4k --json bin/toplevel.json --pcf toplevel.pcf --asc bin/toplevel.asc --opt-timing --placer heap

bin/toplevel.bin : bin/toplevel.asc
	icepack bin/toplevel.asc bin/toplevel.bin

compile : bin/toplevel.bin

time: bin/toplevel.bin
	icetime -tmd hx8k bin/toplevel.asc

prog : bin/toplevel.bin
	cat bin/toplevel.bin >/dev/ttyACM0

clean :
	rm -rf bin
	rm -f Murax.v*.bin
