activitygen -n .\Map_new.net.xml -s .\Traffic_Gen_2.xml -o .\Traffic_Trip.xml
duarouter -n .\Map_new.net.xml -t .\Traffic_Trip.xml -o .\routes_2.rou.xml
pause()