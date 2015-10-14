# SUPPORTED STATES OF TASK EXECUTIVE
# Please note, 0 is coded as NO_STATE to avoid confusions.
uint8 NO_STATE=0
uint8 INITIALIZED=1
uint8 STOPPED=2
uint8 RUNNING=3

# Actual state to communicate using the above definitions
uint8 state
