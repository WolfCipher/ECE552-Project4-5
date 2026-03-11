.ONESHELL:
PROGRAM ?= add0
FILES := $(filter %.asm,$(MAKECMDGOALS))
PROGRAMS := $(basename $(notdir $(FILES)))

.PHONY: assemble clean

# Treat extra '*.asm' command-line args as data, not real make targets.
%.asm:
	@:

assemble:
	set -e
	if [ -n "$(PROGRAMS)" ]; then
		PROGS="$(PROGRAMS)"
		MULTI=1
	else
		PROGS="$(PROGRAM)"
		MULTI=0
	fi
	for p in $$PROGS; do
		docker run --rm -v "${CURDIR}:/src" coderkalyan/ece552-tools:latest bash -c "\
			cd /src && \
			riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $$p.asm -o $$p.o && \
			riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $$p.o $$p.hex \
		"
		if [ $$MULTI -eq 1 ]; then
			OUT="../../tb/$$p.mem"
		else
			OUT="../../tb/program.mem"
		fi
		P="$$p" OUT="$$OUT" python -c "import os, pathlib; p=os.environ['P']; out=pathlib.Path(os.environ['OUT']); b=pathlib.Path(f'{p}.hex').read_bytes(); out.parent.mkdir(parents=True, exist_ok=True); out.write_text('\n'.join(' '.join(f'{x:02x}' for x in b[i:i+4]) for i in range(0, len(b), 4)) + '\n')"
		echo "Wrote $$OUT"
	done

clean:
	-del /q *.o *.hex 2>nul
	-del /q ..\\..\\tb\\*.mem ..\\..\\tb\\a.out 2>nul