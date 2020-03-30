## Multiplayer Demo

### Steps to run:

#### Run the server

    $ python3 examples/high-level/multiplayer/server.py

#### Run client 3D (as much instances as you can handle)

    $ python3 examples/high-level/multiplayer/client3D.py


### Notes:

* Port is hardcoded as `6000` so you might want to change that.
* Host information for client is hardcoded as `localhost`. You can change it to any other host information.

### Network Data Structure:

Each package is `63` bytes long. It includes uuid4-x-y-z coordinates. Coordinates are 8 chars long and formatted as: "%08.2f". 
So it is quite straight-forward. There still might be some lag and it is related to GIL. I didn't pay much attention to performance
for the client side networking.