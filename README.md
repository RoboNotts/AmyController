# Happy Mapping Interface

A very basic interface for creating a map and named locations for Happy. Runs
in Docker at http://[ip]:8000. The Docker host must be on the Happy Wi-Fi network.

Steps to create a new map:

 1. Position Happy in the start position.
 2. Click 'Start Mapping' and wait for success response.
 3. Explore the area using joystick or keyboard teleop interfaces.
 4. Return Happy to the start position.
 5. Click 'Save Map' and wait for success response.
 6. Click 'Stop Mapping' and wait for success response.
 7. Click 'Start Navigating' and wait for success response.
 8. Use joystick or keyboard teleop to drive Happy to different locations.
 9. At each location, click 'Add Location' to create a location by name.
 10. Once you have collected all the locations you need, click 'Save Locations' to get a JSON file of all locations.

The locations file can be used to look up position and orientation data to send
to the move base action by name.
