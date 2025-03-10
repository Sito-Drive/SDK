# Control Mode
ASYNC_MODE                          =   0x03
OPEN_LOOP_MODE                      =   0x04
CURRENT_MODE                        =   0x05
PROFILE_VELOCITY_MODE               =   0x06
PROFILE_POSITION_MODE               =   0x07
POSITION_MODE                       =   0x08
MIT_MODE                            =   0x09
BRAKE_CONTROL                       =   0x20

# RAM Modify Address
OPENLOOP_ACC_DEC_SET                =   0x30
VELOCITY_ACC_DEC_SET                =   0x31
CURRENT_PID_SET                     =   0x42
VELOCITY_PID_SET                    =   0x43
POSITION_PID_SET                    =   0x44
MIT_PID_SET                         =   0x45

# Tips
pattern_phrases = { 0x03:'ASYNC Mode',
                    0x04:'Openloop Mode',
                    0x05:'Current Mode',
                    0x06:'Profile Velocity Mode',
                    0x07:'Profile Position Mode',
                    0x08:'Position Mode',
                    0x09:'MIT Mode',
                    0x0E:'EEPROM MODE'}
