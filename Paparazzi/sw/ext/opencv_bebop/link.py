#! /usr/bin/python



text_file = open("install/lib/pkgconfig/opencv.pc", "r")
lines = text_file.readlines()

for l in lines:
	if 'Libs' in l:
		flags = (l.strip().replace("${exec_prefix}","$(PAPARAZZI_HOME)/sw/ext/opencv_bebop/install").split(" -"))
		iterflags = iter(flags)
		next(iterflags)
		for f in iterflags:
			print("\t<flag name=\"LDFLAGS\" value=\"" + f + "\" />" )



text_file.close()
