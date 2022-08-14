import array
import struct
import sys
import select
from getch import getch

# based on
# https://www.jmeiners.com/lc3-vm
# plus trap, getch and sign extend code from
# https://github.com/mhashim6/LC3-Virtual-Machine

# memory
# ------------------------------------------------------------------------------

PC_START = 0x3000	# start of program memory

MEM_SIZE = 1<<16
memory = array.array('H', [0 for i in range(MEM_SIZE)])

def mem_write(address, value):
	memory[address % MEM_SIZE] = value % MEM_SIZE

def mem_read(address):
	if address == MR_KBSR:
		if check_key():
			# TODO make these mem_write commands?
			memory[MR_KBSR] = (1 << 15)
			memory[MR_KBDR] = ord(getch())
		else:
			memory[MR_KBSR] = 0

	return memory[address % MEM_SIZE]

# registers
# ------------------------------------------------------------------------------
			# registers 0 to 7
R_PC  = 8 	# program counter
R_CND = 9	# condition flags

registers = [0, 0, 0, 0, 0, 0, 0, 0, PC_START, 0]

def reg_write(r, value):
	registers[r] = value % MEM_SIZE

def reg_read(r):
	return registers[r]

# memory mapped registers
# ------------------------------------------------------------------------------

MR_KBSR = 0xFE00	# keyboard status
MR_KBDR = 0xFE02	# keyboard data

# flags
# ------------------------------------------------------------------------------
FL_POS = 1 << 0
FL_ZRO = 1 << 1
FL_NEG = 1 << 2

# traps
# ------------------------------------------------------------------------------
TRAP_GETC  = 0x20	# get char from keyboard, not echoed
TRAP_OUT   = 0x21	# output character
TRAP_PUTS  = 0x22	# output word string
TRAP_IN    = 0x23	# get char from keyboard, echoed
TRAP_PUTSP = 0x24	# output byte string
TRAP_HALT  = 0x25	# halt program

# input buffering (mac/linux/unix specific)
# ------------------------------------------------------------------------------

def check_key():
    # select system call, unix only.
    _, w, _ = select.select([], [sys.stdin], [], 0)
    return len(w)

# utility functions
# ------------------------------------------------------------------------------

def set_cc(r):
	"""
	Sets condition codes based on value of register r.
	"""
	if reg_read(r) == 0:
		reg_write(R_CND, FL_ZRO)
	elif reg_read(r) >> 15:		# check sign bit
		reg_write(R_CND, FL_NEG)
	else:
		reg_write(R_CND, FL_POS)

def sign_extend(value, bits):
	# https://stackoverflow.com/questions/32030412/twos-complement-sign-extension-python
	# TODO precalculate masks
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)

def sign_extend2(x, bit_count):
    if (x >> (bit_count - 1)) & 1:
        x |= 0xFFFF << bit_count
    return x & 0xffff
	
def read_image_file(file):
	blocksize = 2
	
	with open(file, "rb") as f:
		buf = f.read(blocksize)
		mem_addr = struct.unpack('>H', buf)[0]
		while True:
			buf = f.read(blocksize)
			if not buf:
				break
			value = struct.unpack('>H', buf)	# < means little endian, H is unsigned short
			#print(value[0], hex(value[0]))
			memory[mem_addr] = value[0]
			mem_addr += 1

# 0000 branch
# ------------------------------------------------------------------------------

def op_br():
	"""
	BR
	The condition codes specified by the state of bits [11:9] are tested.
	If bit [11] is set, N is tested; if bit [11] is clear, N is not tested.
	If bit [10] is set, Z is tested, etc. If any of the condition codes
	tested is set, the program branches to the location specified by adding
	the sign-extended PCoffset9 field to the incremented PC.
	"""
	p = (instruction >> 9) & 0x1
	z = (instruction >> 10) & 0x1
	n = (instruction >> 11) & 0x1

	cnd = reg_read(R_CND)

	if (p and cnd & FL_POS) or (z and cnd & FL_ZRO) or (n and cnd & FL_NEG):
		pc_offset = sign_extend2(instruction & 0x1FF, 9)
		# pc_offset = sign_extend(instruction & 0x1FF, 16)
		reg_write(R_PC, reg_read(R_PC) + pc_offset)


# 0001 add
# ------------------------------------------------------------------------------

def op_add():
	"""
	ADD
	If bit [5] is 0, the second source operand is obtained from SR2.
	If bit [5] is 1, the second source operand is obtained by sign-extending
	the imm5 field to 16 bits. In both cases, the second source operand is
	added to the contents of SR1 and the result stored in DR. The condition
	codes are set, based on whether the result is negative, zero,
	or positive.
	"""
	r0  = (instruction >> 9) & 0x07		# destination register
	r1  = (instruction >> 6) & 0x07		# operand 1
	imm = (instruction >> 5) & 0x01		# immediate mode instruction?

	if imm:
		imm5 = sign_extend2(instruction & 0x1F, 5)
		# imm5 = sign_extend(instruction & 0x1F, 16)
		reg_write(r0, reg_read(r1) + imm5)
	else:
		r2 = instruction & 0x07			# operand 2
		reg_write(r0, reg_read(r1) + reg_read(r2))

	set_cc(r0)


# 0010 load
# ------------------------------------------------------------------------------

def op_ld():
	"""
	LD
	An address is computed by sign-extending bits [8:0] to 16 bits and adding
	this value to the incremented PC. The contents of memory at this address
	are loaded into DR. The condition codes are set, based on whether the
	value loaded is negative, zero, or positive.
	"""
	r0 = (instruction >> 9) & 0x7
	pc_offset = sign_extend2(instruction & 0x1FF, 9)
	# pc_offset = sign_extend(instruction & 0x1FF, 16)
	mem_addr = reg_read(R_PC) + pc_offset
	reg_write(r0, mem_read(mem_addr))
	set_cc(r0)


# 0011 store
# ------------------------------------------------------------------------------

def op_st():
	"""
	ST

	0011           | sr        | offset
	--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
	 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0

	The contents of the register specified by SR are stored in the memory
	location whose address is computed by sign-extending bits [8:0] to 16 bits
	and adding this value to the incremented PC.
	"""
	sr = (instruction >> 9) & 0x7
	offset = sign_extend2(instruction & 0x1FF, 9)
	# offset = sign_extend(instruction & 0x1FF, 16)
	mem_addr = reg_read(R_PC) + offset

	mem_write(mem_addr, reg_read(sr))


# 0100 jump register
# ------------------------------------------------------------------------------

def op_jsr():
	"""
	JSR, JSRR
	First, the incremented PC is saved in R7. This is the linkage back to the
	calling routine. Then the PC is loaded with the address of the first
	instruction of the subroutine, causing an unconditional jump to that
	address. The address of the subroutine is obtained from the base register
	(if bit [11] is 0), or the address is computed by sign-extending bits
	[10:0] and adding this value to the incremented PC (if bit [11] is 1).
	"""
	pc_addr = reg_read(R_PC)
	reg_write(7, pc_addr)
	if (instruction >> 11) & 0x1:
		pc_offset = sign_extend2(instruction & 0x7FF, 11)
		# pc_offset = sign_extend(instruction & 0x7FF, 16)
		reg_write(R_PC, pc_addr + pc_offset)
	else:
		base_r = (instruction >> 6) & 0x7
		reg_write(R_PC, reg_read(base_r))


# 0101 bitwise and
# ------------------------------------------------------------------------------

def op_and():
	"""
	AND
	If bit [5] is 0, the second source operand is obtained from SR2.
	If bit [5] is 1, the second source operand is obtained by sign-extending
	the imm5 field to 16 bits. In either case, the second source operand and
	the contents of SR1 are bit- wise ANDed, and the result stored in DR.
	The condition codes are set, based on whether the binary value produced,
	taken as a 2’s complement integer, is negative, zero, or positive.
	"""
	r0  = (instruction >> 9) & 0x07		# destination register
	r1  = (instruction >> 6) & 0x07		# operand 1
	imm = (instruction >> 5) & 0x01		# immediate mode instruction?

	if imm:
		imm5 = sign_extend2(instruction & 0x1F, 5)
		# imm5 = sign_extend(instruction & 0x1F, 16)
		reg_write(r0, reg_read(r1) & imm5)
	else:
		r2 = instruction & 0x07			# operand 2
		reg_write(r0, reg_read(r1) & reg_read(r2))

	set_cc(r0)


# 0110 load register
# ------------------------------------------------------------------------------

def op_ldr():
	"""
	LDR

	0110       | dr     | base_r | offset
	-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0 
	
	An address is computed by sign-extending bits [5:0] to 16 bits and adding
	this value to the contents of the register specified by bits [8:6].
	The contents of memory at this address are loaded into DR.
	The condition codes are set, based on whether the value loaded is
	negative, zero, or positive.
	"""
	dr 	   = (instruction >> 9) & 0x7
	base_r = (instruction >> 6) & 0x7
	offset = sign_extend2(instruction & 0x3F, 6)
	# offset = sign_extend(instruction & 0x3F, 16)
	mem_addr = reg_read(base_r) + offset
	# print(f'\t\tmem_addr: {mem_addr:04x}, cont: {mem_read(mem_addr):04x}')
	reg_write(dr, mem_read(mem_addr))

	set_cc(dr)


# 0111 store register
# ------------------------------------------------------------------------------

def op_str():
	"""
	STR

	0111           | sr        | base_r    | offset
	--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
	 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0

	The contents of the register specified by SR are stored in the memory
	location whose address is computed by sign-extending bits [5:0] to 16 bits
	and adding this value to the contents of the register specified by
	bits [8:6].
	"""
	sr = (instruction >> 9) & 0x7
	base_r = (instruction  >> 6) & 0x7
	offset = sign_extend2(instruction & 0x3F, 6)
	# offset = sign_extend(instruction & 0x3F, 16)

	mem_write(reg_read(base_r) + offset, reg_read(sr))


# 1000 return from interrupt
# ------------------------------------------------------------------------------

def op_rti():
	"""
	RTI

	1000           | 000000000000
	--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
	 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0 

	If the processor is running in Supervisor mode, the top two elements on the
	Supervisor Stack are popped and loaded into PC, PSR. If the processor is
	running in User mode, a privilege mode violation exception occurs.
	"""
	pass


# 1001 bitwise not
# ------------------------------------------------------------------------------

def op_not():
	"""
	NOT

	1001           | dr        | sr        | 1 | 11111
	--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
	 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0 

	The bit-wise complement of the contents of SR is stored in DR.
	The condition codes are set, based on whether the binary value produced,
	taken as a 2’s complement integer, is negative, zero, or positive.
	"""
	dr = (instruction >> 9) & 0x7
	sr = (instruction >> 6) & 0x7

	reg_write(dr, ~reg_read(sr))

	set_cc(dr)


# 1010 load indirect
# ------------------------------------------------------------------------------

def op_ldi():
	"""
	LDI
	An address is computed by sign-extending bits [8:0] to 16 bits and adding
	this value to the incremented PC. What is stored in memory at this
	address is the address of the data to be loaded into DR. The condition
	codes are set, based on whether the value loaded is negative, zero,
	or positive.
	"""
	dr  = (instruction >> 9) & 0x07		# destination register
	pc_offset = sign_extend2(instruction & 0x01FF, 9)
	# pc_offset = sign_extend(instruction & 0x01FF, 16)
	
	reg_write(dr, mem_read(mem_read(reg_read(R_PC) + pc_offset)))
	# print(f'dr: {dr}, pc: {reg_read(R_PC)}, offset: {pc_offset}, cont: ')

	set_cc(dr)


# 1011 store indirect
# ------------------------------------------------------------------------------

def op_sti():
	"""
	STI

	1011           | sr        | offset
	--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
	 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
	
	The contents of the register specified by SR are stored in the memory
	location whose address is obtained as follows: Bits [8:0] are sign-extended
	to 16 bits and added to the incremented PC. What is in memory at this
	address is the address of the location to which the data in SR is stored.
	"""
	sr = (instruction >> 9) & 0x7
	offset = sign_extend2(instruction & 0x1FF, 9)
	# offset = sign_extend(instruction & 0x1FF, 16)
	mem_addr = mem_read(reg_read(R_PC) + offset)

	mem_write(mem_addr, reg_read(sr))


# 1100 jump (ret)
# ------------------------------------------------------------------------------

def op_jmp():
	"""
	JMP, RET

	1100           | 000        | base_r   | 000000
	--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
	 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0

	The program unconditionally jumps to the location of the register
	specified by the contents of the base register.
	"""
	base_r = (instruction >> 6) & 0x7

	reg_write(R_PC, reg_read(base_r))


# 1110 load effective address
# ------------------------------------------------------------------------------

def op_lea():
	"""
	LEA

	0110       | dr     | offset
	-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0 

	An address is computed by sign-extending bits [8:0] to 16 bits and adding
	this value to the incremented PC. This _address_ is loaded into DR.
	The condition codes are set, based on whether the value loaded is negative,
	zero, or positive.
	"""
	dr = (instruction >> 9) & 0x7
	offset = sign_extend2(instruction & 0x1FF, 9)
	# offset = sign_extend(instruction & 0x1FF, 16)
	# print(f'\t\tdr: {dr}\tpc: {reg_read(R_PC):04x}\toffset: {offset}')
	# print(f'\t\tmem_addr: {reg_read(R_PC) + offset:04x}\t cont: {mem_read(reg_read(R_PC) + offset):04x}')
	reg_write(dr, reg_read(R_PC) + offset)
	
	set_cc(dr)


# 1101 ---
# ------------------------------------------------------------------------------

def op_res():
	pass

# 1111 execute trap
# ------------------------------------------------------------------------------

def op_trap():
	"""
	TRAP

	1111           | 0000          | trapvect
	--- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
	 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
	
	First R7 is loaded with the incremented PC. (This enables a return to the
	instruction physically following the TRAP instruction in the original
	program after the service routine has completed execution.)
	Then the PC is loaded with the starting address of the system call specified
	by trapvector8. The starting address is contained in the memory location
	whose address is obtained by zero-extending trapvector8 to 16 bits.
	"""
	trapvect = instruction & 0x00FF

	reg_write(7, reg_read(R_PC))
	
	if trapvect == TRAP_GETC:		# get char from keyboard, not echoed
		ch = getch()
		reg_write(0, ord(getch()))
		set_cc(0)
	
	elif trapvect == TRAP_OUT:		# output character
		sys.stdout.write(chr(reg_read(0)))
		sys.stdout.flush()

	elif trapvect == TRAP_PUTS:		# output word string
		mem_addr = reg_read(0)
		ch = mem_read(mem_addr)
		
		while ch != 0x0000:
			sys.stdout.write(chr(ch))
			mem_addr += 1
			ch = mem_read(mem_addr)
		
		sys.stdout.flush()

	elif trapvect == TRAP_IN:		# get char from keyboard, echoed
		sys.stdout.write('Enter a character: ')
		sys.stdout.fliuh()
		reg_write(0, ord(sys.stdin.read(1)))
		set_cc(0)

	elif trapvect == TRAP_PUTSP:	# output byte string
		mem_addr = reg_read(0)
		ch = mem_read(mem_addr)

		while ch != 0x0000:
			sys.stdout.write(chr(ch & 0xFF))
			char = ch >> 8
			if char:
				sys.stdout.write(chr(char))
			mem_addr += 1
			ch = mem_read(mem_addr)
		sys.stdout.flush()

	elif trapvect == TRAP_HALT:		# halt program
		print('HALT')
		running = False

# cpu
# ------------------------------------------------------------------------------

running = True

memory[0x3004] = 1

instruction = 0x0000

read_image_file('2048.obj')

while running:

	# get instruction

	instruction = memory[reg_read(R_PC)]
	instruction_names = ['BR', 'ADD', 'LD', 'ST', 'JSR', 'AND', 'LDR', 'STR', 'RTI', 'NOT', 'LDI', 'STI', 'JMP', 'RES', 'LEA', 'TRAP']
	# if instruction > 0:
		# print(f'pc: {reg_read(R_PC):04x}\tin: {instruction:016b}\top: {instruction_names[instruction >> 12]}')
	reg_write(R_PC, reg_read(R_PC) + 1)
	
	opcode = instruction >> 12
	
	# print(f'pc: {registers[8]}\top: {opcode}')

	# execute instruction

	if   opcode == 0:  op_br() 			# 0000 branch
	elif opcode == 1:  op_add() 		# 0001 add
	elif opcode == 2:  op_ld() 			# 0010 load
	elif opcode == 3:  op_st() 			# 0011 store
	elif opcode == 4:  op_jsr()			# 0100 jump register
	elif opcode == 5:  op_and()			# 0101 bitwise and
	elif opcode == 6:  op_ldr()			# 0110 load register
	elif opcode == 7:  op_str()			# 0111 store register
	elif opcode == 8:  op_rti()			# 1000 return from interrupt
	elif opcode == 9:  op_not()			# 1001 bitwise not
	elif opcode == 10: op_ldi()			# 1010 load indirect
	elif opcode == 11: op_sti()			# 1011 store indirect
	elif opcode == 12: op_jmp()			# 1100 jump (ret)
	elif opcode == 13: op_res()			# 1101 ---
	elif opcode == 14: op_lea()			# 1110 load effective address
	elif opcode == 15: op_trap()		# 1111 execute trap
	else:
		raise Exception(f'BAD OP {opcode:4b} at {registers[8]:16b}')

# end
# ------------------------------------------------------------------------------
print(f'exited: {instruction}')