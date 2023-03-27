# inputfile = './ForwardLoop.motion'
# outputfile = './ForwardLoop_fast.motion'

# Create time increment by 2 seconds in mm:ss:ms format using library function
# https://stackoverflow.com/questions/538666/format-timedelta-to-string
import datetime
def increment_time(time, increment):
  time = datetime.datetime.strptime(time, '%M:%S:%f')
  time += datetime.timedelta(seconds=increment)
  return time.strftime('%M:%S:%f')

with open(inputfile, 'r') as f:
  t = '00:01:720'
  for line in f:
    init = line[:len(t)]
    
    # Time in format mm:ss:ms
    # Divide the time by 2 in duration
    
