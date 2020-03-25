set AUTOTESTDIR="%~dp0\aircraft"

FOR /F "delims=" %%D in ('dir /b "\Program Files"\FlightGear*') DO set FGDIR=%%D
echo "Using FlightGear %FGDIR%"
cd "\Program Files\%FGDIR%\bin"
fgfs ^
    --native-fdm=socket,in,10,,5503,udp ^
    --fdm=external ^

    --aircraft=dhc2F ^
    --airport=KSFO ^
    --geometry=650x550 ^
    --bpp=32 ^
    --disable-clouds ^
    --disable-anti-alias-hud ^
    --disable-hud-3d ^
    --disable-horizon-effect ^
    --timeofday=noon ^
    --disable-sound ^
    --disable-fullscreen ^
    --disable-random-objects ^
    --disable-ai-models ^
    --fog-disable ^
    --disable-specular-highlight ^
    --disable-anti-alias-hud ^
    --aircraft=dhc2F ^
    --fdm=network,localhost,5501,5502,5503 ^
    --fog-fastest ^
    --disable-clouds ^
    --start-date-lat=2004:06:01:09:00:00 ^
    --disable-sound ^
    --in-air ^
    --enable-freeze ^
    --airport=KSFO ^
    --runway=10L ^
    --altitude=100 ^
    --heading=113 ^
    --offset-distance=4.72 ^
    --offset-azimuth=0^
    --wind=0@0
pause
