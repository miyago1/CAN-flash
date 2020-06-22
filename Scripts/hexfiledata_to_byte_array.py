from intelhex import IntelHex

input_string = input("Enter filename of intel hex file:")


lh = IntelHex()
lh.loadfile(input_string, format='hex')


f = open("h.txt", 'w')

f.write("uint8_t array[")
f.write(str(len(lh)))
f.write("] = { ")


pydict = lh.todict()

for line in pydict.values():
	f.write(str(hex(line)))
	f.write(", ")

f.seek(f.tell() - 2, 0)
f.write(" };")

f.close()
