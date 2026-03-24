.ONESHELL:
PROGRAM ?= add0
PROJECT_DIR := $(abspath ../..)
MOUNT_DIR := $(subst \\,/,$(PROJECT_DIR))

assemble:
	docker run --rm -v "$(MOUNT_DIR):/work" coderkalyan/ece552-tools:latest bash -c "\
		cd /work/tests/asm && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM).asm -o $(PROGRAM).o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM).o $(PROGRAM).hex \
	"
	python -c "from pathlib import Path; d=Path('$(PROGRAM).hex').read_bytes(); out='\n'.join(' '.join(f'{b:02x}' for b in d[i:i+4]) for i in range(0, len(d), 4)); Path('../../tb/program.mem').write_text(out + '\n')"

clean:
	rm -f *.o *.hex ../../tb/program.mem ../../tb/a.out