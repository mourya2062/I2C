# I2C
I2C ip core with Avalon Interface 

Have a look at the document before viewing the code 

This I2C works at 100 KHz 

This module expects 50MHz clock (reason is I used DE1-SoC).So, I Hard coded to generate a 200 KHz reference enable 

why 200 KHz ref clock is used ?
Beacause it is very convinient to code the logic (FSM) with double the required frequency 

200 KHz ref clock or enable ?
It is 200 KHz enable because my FSM runs with the 50 MHz clock and I am using 200 KHz enable to change the FSM states .At every enable I am negating the scl so I am acheiving 100 KHz SCL 

If you dont want to use the avalon interface ,remove the avalon interface in the I2C Master file and add any medium or interface to write the fifo and read from the fifo and to configure registers 

FIFO has 1 clock cycle latency for read so keep that constraint in mind, when FIFO is being used 

https://www.ti.com/lit/an/slva704/slva704.pdf?ts=1670610078103&ref_url=https%253A%252F%252Fwww.google.com%252F

I refered the above document during my FSM development as well as code development 

I tried my level best to write the comments in the code 

If you have any doubts regarding that you can reach me at mouryabc@outlook.com

I thank my professor Jason Losh and my TA Azmi for clarifying my doubts during the development time 
