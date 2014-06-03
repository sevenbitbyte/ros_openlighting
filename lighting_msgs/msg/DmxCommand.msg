DmxFrame[] frames
int8 action
string name

int8 DISPLAY=0     # Immediatly display frames, ignores name
int8 PLAY=1        # Start playback of named command
int8 STOP=2        # Stop playback of named command if supplied, stops all running commands if name is empty
int8 REMOVE=3      # Remove stored from with supplied name. Empty name removes all stored frames
int8 STORE=4       # Store the supplied frames, non-empty name is required
int8 WRITE=5       # Write stored command to disk. If name is empty all stored frames will be written to disk. If name is supplied and frames is empty the named frame will be written to disk. If both name and frames are supplied this command will be sent directly to disk
int8 LOAD=6        # Load stored command from disk
