#python3
import re
import shutil

files_to_patch = ["WildhornAV/kernel/stm32mp157d-wildhornav-mx.dts", "WildhornAV/tf-a/stm32mp157d-wildhornav-mx.dts", "WildhornAV/u-boot/stm32mp157d-wildhornav-mx.dts"]


for file in files_to_patch:
	block_count = 0
	uart8_block = 0
	new_file = file+".new"
	old_file = file+".old"
	f = open(file, "r")
	new_f = open(file+".new", "w")
	for line in f.readlines():
		block = re.findall(r"{", line)
		eblock = re.findall(r"}", line)
		uart8_name = re.findall(r"uart8_mx-0 {", line)
		if(uart8_block and block_count == uart8_block):
			uart8_block = 0;
		if(uart8_name):
			uart8_block = block_count;
		block_count += len(block) - len(eblock) 
		line = re.sub(r"bias-disable;", "bias-pullup;", line)
		if(uart8_block):
			print(block_count, uart8_block, line, end="")
		new_f.write(line)
	f.close()
	new_f.close()
	shutil.move(file, old_file)
	shutil.move(new_file, file)
