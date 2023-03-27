# inputfile = './TaiChi.motion'
# outputfile = './TaiChi_mod.motion' 
# output_scale = 0.8
output_scale = 0.2 # stable, 0.1 not, 10s
# output_scale = 0.185

inputfile = './TurnLeft20.motion'
outputfile = './TurnLeft20_mod.motion'
output_scale = 0.5

inputfile = './TurnRight20.motion'
outputfile = './TurnRight20_mod.motion'
output_scale = 0.5

# Same for TunLeft40 and TurnRight40
inputfile = './TurnLeft40.motion'
outputfile = './TurnLeft40_mod.motion'
output_scale = 0.7

inputfile = './TurnRight40.motion'
outputfile = './TurnRight40_mod.motion'
output_scale = 0.7

import datetime
# Time delta in seconds
delta = 0.5
point = '00:00:000'
point = datetime.datetime.strptime(point, '%M:%S:%f')

# Read input file
times_in = [point]
times_out = [point]
with open(inputfile, 'r') as f:
  next(f)
  for line in f:
    point = line[:9]
    # Read time in format mm:ss:mss
    point = datetime.datetime.strptime(point, '%M:%S:%f')
    # Create time from min, sec, ms info
    times_in.append(point)

duration_in = times_in[-1].second
# print(duration_in); exit()
duration_out = duration_in * output_scale

# print(times_in)
# Find cumulative time differences
# diffs
for i in range(1, len(times_in)):
  # Find time difference in seconds
  diff = times_in[i] - times_in[i-1]
  scaled = diff * output_scale
  # print(diff*output_scale)
  new_time = times_out[i-1] + scaled
  times_out.append(new_time)

# Date time object to hh:mm:ss:mss
time = times_out[0].strftime('%M:%S:%f')
for idx, time in enumerate(times_out):
  times_out[idx] = time.strftime('%M:%S:%f')[:-3]

print(times_out)


with open(inputfile, 'r') as f:
  with open(outputfile, 'w') as out:
    header = next(f)
    out.write(header)
    idx = 0
    for line in f:
      motion = line[9:]
      idx += 1
      full = times_out[idx] + motion
      out.write(full)
    


