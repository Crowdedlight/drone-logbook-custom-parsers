# Custom Parsers for Open DroneLogBook

This is custom parser for ULOG files from PX4 flight-controller that is made to extend the files https://github.com/arpanghosh8453/open-dronelog can parse. 

The parser will convert from ulog format into the CSV format Open-DroneLog expects. 
As I have no way to extract drone-name or similar metadata alone from the ulog, the only meta-data that is extracted and
passed to dronelogbook is the following fields. (as long as they are set and saved into the ulog file) 
```
{
    "drone_serial": "000400000000467889313234572630240032", 
    "drone_fc_hw": "PX4_FMU_V6X", 
    "start_time": "2026-03-12T10:02:13Z", 
    "battery_serial": "0"
}
```

# Usage
You can manually mount the parser files and make sure the open-dronelog docker image has the python requirements installed.   
You can see in the Dockerfile that the parser and parser config should be mounted at:

```
# Copy the ulogParser folder into /app/plugins/
COPY ulogParser /app/plugins/ulogParser
# copy the parsers.json config 
COPY parsers.json /app/plugins/parsers.json
``` 

However for ease of use, this repo is set up to track new releases from open-dronelog and 
automatically trigger docker image rebuilds. So you can also just use this image in your docker-compose and 
automatically have the parser included. 

```
image: ghcr.io/arpanghosh8453/open-dronelog:latest 
```



**python requirements:**
- scipy
- pyulog

**Following parsers work:**
- ulog (PX4)

**Potential todo:**
- Betaflight - blackbox