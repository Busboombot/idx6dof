int8 ABSOLUTE=1
int8 RELATIVE=2 
int8 JOG=3

Header header
int8 movetype
float32 t           # Common time for either X_COMMAND or V_COMMAND
float32[6] x   # Joint values, either distance or velocity