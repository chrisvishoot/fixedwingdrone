Instructions for working with the Xbee and the Raspberry Pi:


1. Connect one of the Xbees to the laptop.  Next, run the solution
   included in  OneDrive. 
2. Now we look into connecting the Xbee to the raspberry Pi. True
   there are various rhings involved - address configurations
   and what not to make two Xbees to communicate. But the good news is that you don't have 
   to do all of that. The Xbees are already configured to do that for you. They will communicate
   with each other back and forth with the addresses that are  assigned to them already. So
   you do not have to  make any changes to addresses or settings using the software that comes with it/

3. Now, note that the Xbee and the raspberry Pi communicate with each other using serial communication.
   This means that the Rx pin on the PI goes to the TX pin on the Xbee, and the TX on the PI to the RX
    of the Xbee. I referred to the followinbg article to connect the Raspberry PI to the Xbee:
   http://sonyarouje.com/2014/12/20/connecting-xbee-to-raspberry-pi/


4. On the laptop - Please run the  Visual Studio solution included in OneDrive in the folder titled: 'VisualStudioSolution'
   I wanted to use the XCTU software to communicate wbetween the two Xbees, but that for some reasons  = some
    bug I believe did not work. So I used the solution I have included in the One Drive project. I download the 
  solution from the link: http://www.prodigyproductionsllc.com/articles/programming/add-xbee-to-raspberry-pi-with-python/
   and it worked beautifully. Please run the solution to start the program, and make sure that the COM port that communicates 
   with the Xbee is not being used by another program - - otherwise the solution will NOT be able to use that PORT to 
  communicate with the Xbee. 
  

4. Back to the PI:  please refer to the code: python_read.py This file contains the code necessary to communicate with 
   the other Xbee. So all you have to do is run this program on the  raspberry PI by doing:
   python python_read.py

5. Now the program will be suspended in the readline() method, waiting for content to arrive.

6. Type something into the Visual Studio solution's text bbox. You will see that the message  has been received by the PI
   and the PI has responded as well.


   