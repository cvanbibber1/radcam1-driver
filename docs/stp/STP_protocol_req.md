STP (space test program) is a space flight which this board is intended to fly in.
This will be in low earth orbit, and the only way we can communicate is via RS422. Everything we can possibly want to do must be done via RS422 and this protocol.

This board has an ADM2582EBRWZ RS422 transciever, allowing UART_TX and UART_RX (with DE to enable drivers when you want to speak, make sure this does not interfere with other RS422 slaves or the master, only enable when required.)
We do not know how many chained RS422 transcievers are present, this may be 1 or up to 5, please add a unique device ID for each that allows for specific targetted commands from the master (flight computer).

The STP Command Packet (from flight computer to us, the experiment) consists of:
A 4 byte sync bytes (0x1ACF FC1D)
Then Coarse Time (timestamp with 1 second resolution as 4 bytes)
Then Fine time (2 bytes, resolution of 15.3 micro seconds) # How to reconstruct?
Then packet type (1 byte, packet of 0x10 is command,) # List of other packet types?


Optional but may be helpful: Consider radiation and bit flips and possibility of triple modular redundancy in several areas (full triple modular redundancy or only for critical / vulnerable parts) otherwise implement radiation protections when feasible.


As final checks, you should do a reliability check - confirm that the entire codebase works properly and is consistent, bug-free, and reliable, testing any edge cases or other scenarios.
Then do a safety check - ensure as many safety features as viable are present to protect from radiation events, or any unforseen or error states etc.
Finally do a autonomous check - ensure the entire system can function semi-autonomously, handling any errors present and only needing input from this RS422 protocol for all functions.
