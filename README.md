
# SpatialRoboticCoordination_casadi
Work at AASS, MRO lab 
=======

for results with 1 moving obstacle and 1 static obstacle: Run Oru_main.m, it calls ReferenceTraj2_with_static.m and then oru_sptl_OC_m2_3obs_nv.m. We take just 3 points out of N prediction horizon in consideration. Also, we update the reference traj with solution from mpc.<br>
[![3 Robots Coordination](http://img.youtube.com/vi/q4uXA4o79dI/0.jpg)](https://www.youtube.com/watch?v=q4uXA4o79dI "3 Robot Coordination")<br>
A smarter solution is obtained when bounds on velocity are less-constrained (loosened).
[![3 Robots Coordination_smarter](http://img.youtube.com/vi/A04rr_k4d_o/0.jpg)](https://www.youtube.com/watch?v=A04rr_k4d_o "3 Robot Coordination_smarter")<br>
Robot coordination in Antipodal configuration.<br>
[![Antipodal Config](http://img.youtube.com/vi/cO1aWaIwPyU/0.jpg)](https://www.youtube.com/watch?v=cO1aWaIwPyU "Antipodal Config")<br>
[![Sense Radius in Antipodal config](http://img.youtube.com/vi/gYxwLL98XjE/0.jpg)](https://www.youtube.com/watch?v=gYxwLL98XjE "Sense Radius in Antipodal config")<br>
