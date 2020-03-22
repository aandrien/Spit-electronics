import numpy

clock_speed = 16000000
prescaler = 256 # Can be [1,8,64,256,1024]

desired_interrrupt_freq = 100 #[Hz]

compare_match_reg = clock_speed/(prescaler*desired_interrrupt_freq)-1

print(compare_match_reg)