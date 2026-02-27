.ONESHELL:
PROGRAM ?= 01add
PROGRAM2 ?= 02addi
PROGRAM3 ?= 03and
PROGRAM4 ?= 04andi
PROGRAM5 ?= 05lui
assemble:
	docker run --rm -v $(PWD):/src coderkalyan/ece552-tools:latest bash -c "\
		cd /src && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM).asm -o $(PROGRAM).o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM).o $(PROGRAM).hex \
	"
	xxd -g 1 -c 4 -p $(PROGRAM).hex | sed 's/../& /g' > ../../tb/$(PROGRAM).mem

	docker run --rm -v $(PWD):/src coderkalyan/ece552-tools:latest bash -c "\
		cd /src && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM2).asm -o $(PROGRAM2).o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM2).o $(PROGRAM2).hex \
	"
	xxd -g 1 -c 4 -p $(PROGRAM2).hex | sed 's/../& /g' > ../../tb/$(PROGRAM2).mem

	docker run --rm -v $(PWD):/src coderkalyan/ece552-tools:latest bash -c "\
		cd /src && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM3).asm -o $(PROGRAM3).o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM3).o $(PROGRAM3).hex \
	"
	xxd -g 1 -c 4 -p $(PROGRAM3).hex | sed 's/../& /g' > ../../tb/$(PROGRAM3).mem

	docker run --rm -v $(PWD):/src coderkalyan/ece552-tools:latest bash -c "\
		cd /src && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM4).asm -o $(PROGRAM4).o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM4).o $(PROGRAM4).hex \
	"
	xxd -g 1 -c 4 -p $(PROGRAM4).hex | sed 's/../& /g' > ../../tb/$(PROGRAM4).mem

	docker run --rm -v $(PWD):/src coderkalyan/ece552-tools:latest bash -c "\
		cd /src && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM5).asm -o $(PROGRAM5).o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM5).o $(PROGRAM5).hex \
	"
	xxd -g 1 -c 4 -p $(PROGRAM5).hex | sed 's/../& /g' > ../../tb/$(PROGRAM5).mem

clean:
	rm -f *.o *.hex ../../tb/*.mem ../../tb/a.out