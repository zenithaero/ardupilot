#! /bin/sh 

cd /Applications/FlightGear.app/Contents/Resources 
./../MacOS/fgfs --fg-root=./  --aircraft=dhc2F --native-fdm=socket,in,10,,5503,udp --fdm=external --fog-fastest --disable-clouds --start-date-lat=2004:06:01:10:00:00 --disable-sound --in-air --enable-freeze --airport=KSFO --runway=10L --altitude=2 --heading=113 --offset-distance=4.72 --offset-azimuth=0
