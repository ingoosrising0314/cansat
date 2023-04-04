import serial

ser=serial.Serial('/dev/ttyACM0',38400) #reset serial port

while True:
	data=ser.readline().decode('utf-8').rstrip().split(',')
	if len(data)!=10:
		continue
	ax,ay,az,gx,gy,gz,mx,my,mz,heading=[float(i) for i in data]
	print('Axyz: ',ax,', ',ay,', ',az,'\n')
	print('Gxyz: ',gx,', ',gy,', ',gz,'\n')
	print('Mxyz: ',mx,', ',my,', ',mz,'\n')
	print('Heading: ',heading,'\n')
	print('------------------------')