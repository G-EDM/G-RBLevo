# G-RBL - A three axis motion control software for electrical discharge machining

    "Doing Wire EDM in your shop and on your tabletop,
    never waste time with two eyes on your clock, waiting for a collab with da doc or a file drop 
    tick tock,.,. get the holes popped-and,.,.,.
    pull the string from the top through the stock with a nonstop flush-in-the-cut-of-your-block"


# Gerber files, schematics and more can be found here: 

https://github.com/G-EDM/GEDM-EVO2-CNC


</br>
</br>
</br>

<img src="https://raw.githubusercontent.com/G-EDM/GEDM-EVO2-CNC/main/media/images/111.jpg">
<img src="https://raw.githubusercontent.com/G-EDM/GEDM-EVO2-CNC/main/media/images/3.jpg">
<img src="https://raw.githubusercontent.com/G-EDM/GEDM-EVO2-CNC/main/media/images/444.jpg">

</br>
</br>
</br>


# G-RBL EVOII Firmware ⚡⚡⚡

```diff
  ██████        ███████ ██████  ███    ███  
 ██             ██      ██   ██ ████  ████  
 ██   ███ █████ █████   ██   ██ ██ ████ ██ 
 ██    ██       ██      ██   ██ ██  ██  ██ 
  ██████        ███████ ██████  ██      ██ 
```

Firmware for the G-EDM EVOII router. XYZ axis + spindle stepper to pull the wire.

</br>
</br>
</br>


# Notes

    There are different DPM8605 versions available. The cheap one is used that has the simple protocol activated. Not the RS version.
    Also it may be a good choice for the future to look at the DPH8909. 
    It seems like that this device has the same display and the same protocol but allows a much higher voltage up to 96v and 10A current. 
    Well.. Maybe it is not that easy to build. I was not able to find any 100v power supplies at an acceptable price tag. 
    Basically the only one I found would cost multiple hundreds. Also the max voltage on the pulseboard should not exceed 90v. 
    The board has 100v Zener diodes to protect the INA inputs that have a max common mode voltage of 110v.
    The DPM/DPH series are just buck converters that need an external source of power.

    Needs an ILI touch display to work too. See the G-EDM EVOII router repository for details.
    Even if the name suggest it, it does not work the way Grbl does
    Read the manual.

# Install

Download the repo into a folder.


# Install vStudio with the Platform.io extension 


In the left panel in vStudio click the extension icon. Use the search field to search for PlatformIO and install it.
Restart vStudio and then open the folder via the file menu ->open_folder. After the folder is loaded platform.io will setup some things. Once it is finished with the first setup vCode should be restarted. 
Sometimes it will fail after the initial loading if not restarted.

# Flash the firmware to the ESP32

Remove the ESP32 from the motion board!
Never connect the USB to the ESP while it is plugged into the PCB. I have never tested it and it may break something.
If the PCB is powered AND USB is connected too it will burn the ESP for sure. But maybe it could burn the buck converter stage
even if the board is not powered. As said. Maybe.

Remove the ESP from the PCB and connect it via USB to the PC. 
I am working with Linux and it is possible that the user needs privilegs to access the USB device.
In Linux this can be done via a small command in the terminal as root:

ttyUSB0 may be different on other machines. Figure it out via google and chatgpt. 

    sudo chown -R YOURUSERNAME /dev/ttyUSB0


In vStudio at the top right is a small check or arrow icon with a dropdown menu. Open the small menu and select upload. Then click the upload icon.

Sometimes there is some stuff going that seems to block flashing. If the flashing failed just hit the upload button again.
If the second attempt does not work there is a different problem.

On windows it may be required to install a driver for the CP2102 first:
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers


Read the DPM manual and set the Baudrate to the highest possible if communication is wanted: Baud 115200
If this is not done communication will fail.



# First use

After first boot it will start the display calibration. If for some reason a mistake happens the UI provides an option to restart the calibration.
To get there just click the small panel on top of the integrated scope and navigate to the UI settings page.


Next adjust the busvoltage feedback.
This is a little tricky but needs to be learned.

    1. Adjust the pulseboard bus voltage feedback to ensure a fast digital low once the DPM voltage drops
    
        Warning: Never allow more then 4.8v from the bus feedback to the sensorcircuit. Before connecting the sensorboard to the pulseboard make sure the feedback stays below 5v. Turning the Poti clockwise will reduce the voltage.
        Unconnect the feedback from the sensorboard, turn on the DPM, set it to the max voltage and measure the voltage on the JST output. Set it to something below 5v. 4v is ok too. Once the max feedback is adjusted turn the power off and connect it to the sensorboard.
    
        Turn the power on again.
        Set the DPM to 56v
        Set the on/off switch to OFF. This way you can enable PWM without starting the process and have a working scope running.
        Enable PWM via the big touch button with the Start/Stop switch turned to OFF.
        Use the big poti on the pulseboard (the one below the sensorboard) and turn it clockwise to reduce the feedback voltage until the right dot on the scope starts to jump around between HIGH/LOW. Green dot = Logic high/Red = logic low. It is an analog signal that is used as digital one. 
        Done. The state should now react very fast to a voltage drop but not be overreactive on short tiny 1-2v drops. 
        Maybe this can be improved by allowing the voltage to drop some more. Needs testing.
        
        
    2. Prepare the SD card
    
        The firmware will create a directory tree on the sd card. All gcode files need the file extension .gcode and need to be within the gcode folder. The sd card needs to be Fat32 formatted and some sd card may not work. If there are problems try a different card.
        
        Put an SD card into the display (Micro_SD + Adapter) and turn the power on. if the SD menu on the bottom left side remains red it has troubles detecting the card.
        
        If it changes color to fit the rest of the menu it was able to read the card and should by now have created the directory tree.
    
    3. First run
    
        Place the example.gcode file into the gcode folder on the SD card.
        Insert the card back into the display and wait until it is detected.
        Click the SD menu entry and then load the example.gcode file
        
        Set the mode to 2D wire.
        Set the DPM to 60v 0.5A (if the DPM is connected to the RX/TX channels the firmware will enable the DPM on its own, but this options needs to be enabled in the menu first)
        If DPM communication is not enabled turn the DPM on manually.
        
        Turn the Start/Stop switch to On. 
        All motion is blocked if the switch is OFF, it will stop all motion if switched to off while motion is happening and it will only start the process if the switch is in the ON state.
        
        If "enable PWM" is pressed now it will start the process.
        
        The default config is very sensitive and it will stop basically after it makes the first touch. This is the point to figure out the settings. Good luck.
        
        Tips: Start to increase the short circuit duration by 100us steps to not go into a pause so fast.
        Increasing the DPM current will also prevent voltage drops on tiny loads. 
        Look at the menu to get some infos about what the parameters do.
        It is not documented yet but most menu items have a tiny desciption.
        
        If the machine position color in the process turns blue it does a soft correction retraction, if it turns red it does a hard retraction due to a short.
        
        Goal should be to get rid of almost all red retractions and reduce the blue oney to a minimal while also having a constant burn.
        But also archive good precision. Line ends can be a problem. There is a setting for line to line confirmations. Within an arc motion that contains a large number of tiny lines at easy angles it will use half the confirmations of what is set there. 
        
        The higher the value the higher the delay will be at the line end.
        
        Forward motion is only allowed if the "full range AVG" (the small line on the right) is below setpoint min, the default average is below setpoint min and some other stuff.
        
        A soft retraction is triggered if the fAVG is above the fAVG treshhold.
        
        Retraction confirmation will help to not ovverreact on shorts and will reduce the hard retraction to a short retraction for given number of confrimations etc.
        
        
        

        
</br>
</br>
</br>




# G-RBL

The gedm folder is copyright protected and not allowed to be used in any commercial ways and redistribution without permission is prohibited.


The /lib/Grbl_Esp32 folder is a highly modified version of grbl_esp32 and complies with the licensing they set.

Don't expect the code to work like Grbl. It is heavily modified.

The GEDM code is not a commercial product and is not sold, nor is it a ready to use product or compiled binary. It is not shipped with another product in any way. The code is not used in any GEDM products that may become available. It is just a beautiful poem on github that everyone can read and have fun with:

https://github.com/bdring/Grbl_Esp32/issues/1528#issuecomment-1749408457

All those points make it possible that the license of grbl doesn't affect the GEDM folder. 

The code is for private use and research only and not meant to be compiled. If it compiles at all.





# Stay informed:

[>>> Follow the project on Youtube <<<](https://www.youtube.com/@G-EDM/videos)

[>>> Follow the project on Hackaday <<<](https://hackaday.io/project/190371-g-edm)

[>>> Join the discord <<<](https://discord.gg/9cTsyDkEbe)





# Legal notes

    The author of this project is in no way responsible for whatever people do with it.

    No warranty. 
    
    
    
    
    
# License

    All files provided are for personal and private use if not declared otherwise and any form of commercial use or redistribution of the protected files is prohibited. 
    
    
    
    
    
# Donations

    * Every coffee counts. You want to donate something? 
    * Paypal: paypal.me/gedmdev
    * Bitcoin: bc1q9akp00a5hceh9n3jc9wfttxuwuk9c7da0sqkr8

<img src="https://raw.githubusercontent.com/G-EDM/G-EDM/main/images/artwork/donations/donate.png">
