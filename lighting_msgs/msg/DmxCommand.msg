int8 DISPLAY=0     # Immediatly display frames, ignores name
int8 PLAY=1        # Start playback of named command
int8 STOP=2        # Stop playback of named command if supplied, stops all running commands if name is empty
int8 REMOVE=3      # Remove stored frame with supplied name. Empty name removes all stored frames
int8 STORE=4       # Store the supplied frames, non-empty name is required
int8 LOAD=5        # Load stored command from disk

int8 action
string name
DmxFrame[] layers
bool loop
string next
