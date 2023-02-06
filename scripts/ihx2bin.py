import argparse
import re

parser = argparse.ArgumentParser()
parser.add_argument('infile')
parser.add_argument('outfile')
args = parser.parse_args()

sparse = {}

with open(args.infile, mode="r") as f:
  for line in f:
    # Skip characters up to start code (:)
    try:
      line = line.split(':', 1)[1]
    except:
      continue

    # Read start of record
    size = int(line[0:2], base=16)
    address = int(line[2:6], base=16)
    type = int(line[6:8], base=16)
    checksum = size + (address >> 8) + (address & 0xFF) + type

    # Stop when end of file record is found
    if type == 1:
      break

    # Read data into sparse dictionary
    end = 8 + 2 * size
    for i, v in enumerate(re.findall('..', line[8:end])):
      data = int(v, base=16)
      sparse[address + i] = data
      checksum += data

    # Validate end of record checksum
    neg_checksum = int(line[end:end+2], base=16)
    assert((checksum + neg_checksum) & 0xFF == 0)

# Find min and max address
base = min(sparse.keys())
size = 1 + max(sparse.keys()) - base

# Copy data from dictionary into array
dense = bytearray(size)
for i, v in sparse.items():
  dense[i - base] = v

print('Base address:', base)
with open(args.outfile, mode='wb') as f:
  f.write(dense)
