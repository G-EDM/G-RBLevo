# Documentation

There is no real documentation yet.


.gcode files should only contain the raw position commands in Grbl format. No start and no end commands. Metric system only. 
If there is gcode to change to imperial the code will overwrite it and switch back to metric.
I don't have time to change this at the moment.

The code is not comparable to Grbl. Don't expect anything to work like it does with Grbl.


There are a lot of parameters to adjust the process. Some are available on the front page via the left menu buttons but the most are within the menu that can be opened from the scope.

Touch the top bar of the scope to enter the menu and use the navigation arrows to change pages.




#### Menu from the scope - Hit the scope top bar to access it ####

# Page 1:

    # Frequency: 
        The PWM frequency for the spark in Hz. The code tries to detect rising edges but doesn't do this for every sample within the I2S batch. This would be too slow.
        The rising edge is based on the average of the I2S batch. If the frequency is too high it may not work well. I never tested anything above 20khz.
        I may be able to add some rising edge detection within the I2S batch later. 

    # Duty
        Duty cycle in precent.

    # Max speed:
        The maximum feedrate allowed in the process. This is not very accurate. There is no complex logic to calculate the true speeds. It just looks for the slowest axis and calculates the step delay for this axis to not pulse faster then this axis can go. 

    # Setpoint min
        Allow forward motion only if the slow avergae (fAVG/full range AVG), the fast/default average and the recent reading are below this setpoint value.
        This value is in percent from the max ADC resolution. 4096 = 100%.

    # Setpoint max
        Part of the retraction logic. 

    # Spindle RPM
        Speed of the spindle stepper that pulls the wire. 
        If your setup is different the RPM may not be what the display shows. 
        It is based on Full steps per rotation * Microsteps for a axis and uses MCPWM at a calculated frequency to pulse the TMC driver.

    
# Page 1:


    # Draw linear
        Draw the used readings of the ADC as vertical lines on the scopes. Can help debug. 
        The values shown are not the true readings but the calculated ones based on the averages set for rising and falling.

    # Short duration in us
        Total time in microseconds that a short circuit is allowed to take before a pause is enforced.
        200-500us based on the settings. Make sure the DPM settings allows a voltage drop big enough at given PWM settings to trigger a logic low.
        Without logic low on shorts this timer will not work.

    # Brokenwire(mm)
        The maximum distance the machine is allowed to move without load before it enters a pause. 
        Normally this count starts after the initial contact but sometimes the initial contact may trigger to early.
        Need to fix it. No big issue.
        This distance is also used as max retraction distance.

# Page 3

    # eTreshhold
        Edge treshhold in ADC resolution 0-4096. This is to reduce false edge detection. A reading below this value will not be counted as a rising edge.
        There is some more going on with this parameter but that is not important to know.

    # Zoom
       Zoom used on the scope. The wave displayed will get bigger if needed. Never really used it.

    # I2S samples
        Number of samples used from the full I2S batch. Better use the full batch here.

    # I2S rate kSps
        Speed of the I2S reading. Changing I2S on runtime can sometimes brick I2S. Don't change this within the process if the process is important.
        
    # I2S Buffer Length
        dma_buf_len. I2S buffer size. The number of readings per I2S batch.

    # I2S Buffer Count
        dma_buf_count. Basically the number of parallel batches. A higher value will generate outdated batches. 

    # Averaging Rise
       If there is a rising edge detected the sample is pushed nth times to the sample buffer to speed up the rising of the average.

    # Averaging Main
       Fast average/Default average. Higher value will slow down the falling edge etc.


# Page 4

    # Early retract exit
        Allow early exit of retractions if the code thinks a short is canceled. 

    # Retractconfirm
        Confirm a retraction x number of times and reduce hard retraction to soft ones if counter is below this

    # Max reverse depth
        Maximum number of lines allowed to move back in history on retractions

    # Retract Hard
       Hard retraction distance and speed

    # Retract Soft
       Soft retraction distance and speed


# Page 5

    # HW Timer ADC
        If enabled there will run a HW timer at 20khz for collecting the I2S samples. If disabled the sampling is synchronized with the pulse.
        Default: Off

    # SamplesBestOf
        Only used for possible forward motion. if a forward motion is indicated it will collect this number of samples more to confirm it before each forward step.

    # FullRangeAVG size
        Slow AVG/Full range AVG/ fAVG size. Number of samples use to calculate the slow average. Shwon as the small line left in the scope. Turns red if above the treshhold.

    # fAVG treshhold
        If the slow average moves above this treshhold it will trigger a retraction. 

    # Pulse count jitter
        deprecated and not used anymore. Need to remove this.


# Page 6

    # Pulse off duration
        PWM turned off for given pulses on high loads/shorts. This value is in pulses and not time.
        At 20khz a pulse is a few us. It will convert the pulsenumber into time.

    # Early exit on
       If early exit on retractions is enabled it will exit retraction once the motion plan is at this point. 

    # Line end confirms
       On a line end it will confirm the forwar motion x times before it moves to the next line. For arcs half of that value is used.

    # ResetSenseQueue
       If enabled it will reset the sense queue before each step to wait for the next fresh most realtime reading.


# Page 7

    # Hold/Go Signals
        Enable/Disable the use of those signals
    
    # Zeros jitter
        Number of low readings in a row needed to reset the high readings counter
      
    # Readings High
        Number of high readings in a row that will trigger some protection logic

    # Readings Low
        Number of low readings in a row before a forward motion is allowed

    # Zero treshhold
        ADC value treshhold. Count low readings only if the ADC value is above this. Helps to ignore noise etc.

    # High count at
        Defines waht motion plan is interpreted as a high plan

# Page 8

    # DPM settings if RX/TX is used

    Turn it on/off and change voltage current via UI.

# Page 9

    # Calibrate touch
        Only hit if you want to do a recalibration of the touch display.


# Page 10

    Set the steps per mm for XYZ. Need a restart to make the changes work.
    Also this is stored on SD card. If you SD card is available this will not work. The SD card with the settings need to be availabel on boot too.
    Will change this later to store this in the ESP storage.

    
    
