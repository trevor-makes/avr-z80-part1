import argparse

parser = argparse.ArgumentParser()
parser.add_argument('infile')
parser.add_argument('outfile')
parser.add_argument('-r', '--record_size', type=int, default=32, help='bytes of binary data per line of text')
parser.add_argument('-b', '--base_address', type=int, default=0, help='where the binary data will be loaded into memory')
args = parser.parse_args()

# Read infile as bytes object
with open(args.infile, mode="rb") as f:
  data = f.read()

with open(args.outfile, mode="w") as f:
  index = 0
  while index < len(data):
    # Chunk data into record size
    rec = data[index:index+args.record_size]
    size = len(rec)

    # Offset by base address
    address = index + args.base_address
    index += size

    # Start new data record (type 00)
    # TODO add support for 24/32 bit addresses using 02/04 record types
    f.write(f':{size:02X}{address:04X}00')
    checksum = size + (address >> 8) + (address & 0xFF)

    # Write record data
    for byte in rec:
      f.write(f'{byte:02X}')
      checksum += byte

    # Finish record with negated checksum
    checksum = -checksum & 0xFF
    f.write(f'{checksum:02X}\n')

  # Write end of file record (type 01)
  f.write(':00000001FF\n')
