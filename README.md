# Kalman-Filter

Implementation of a Kalman Filter and example based on:
http://greg.czerniak.info/guides/kalman1/

Run the code and send the output to out.dat:
kalman_filter_cannonball.exe > out.dat

In gnuplot, the output can be plotted with:
plot 'out.dat' using 1:2 w lines, 'out.dat' using 3:4 w lines, 'out.dat' using 5:6 w lines

Other resources used:
http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
http://robotsforroboticists.com/kalman-filtering/
https://en.wikipedia.org/wiki/Kalman_filter
https://courses.cs.washington.edu/courses/cse466/11au/calendar/14-StateEstimation-posted.pdf
