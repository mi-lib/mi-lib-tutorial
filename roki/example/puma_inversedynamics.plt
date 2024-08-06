FILE='puma_inversedynamics.dat'

set terminal png

set output "puma_inversedynamics_q1.png"
plot FILE u 1:2 w l lt 1 t 'q1 [rad]'
set output "puma_inversedynamics_q2.png"
plot FILE u 1:3 w l lt 2 t 'q2 [rad]'
set output "puma_inversedynamics_q3.png"
plot FILE u 1:4 w l lt 4 t 'q3 [rad]'
set output "puma_inversedynamics_tau1.png"
plot FILE u 1:8 w l lt 1 t 'tau1 [N.m]'
set output "puma_inversedynamics_tau2.png"
plot FILE u 1:9 w l lt 2 t 'tau2 [N.m]'
set output "puma_inversedynamics_tau3.png"
plot FILE u 1:10 w l lt 4 t 'tau3 [N.m]'
set output "puma_inversedynamics_dq-tau.png"
plot FILE u (abs($2)):(abs($8)) w l lt 1 t 'dq1/dt-tau1', FILE u (abs($3)):(abs($9)) w l lt 2 t 'dq2/dt-tau2', FILE u (abs($4)):(abs($10)) w l lt 4 t 'dq3/dt-tau3'

! convert +append puma_inversedynamics_q1.png puma_inversedynamics_q2.png puma_inversedynamics_q3.png puma_inversedynamics_q.png
! convert -resize 1024x puma_inversedynamics_q.png puma_inversedynamics_q.png
! convert +append puma_inversedynamics_tau1.png puma_inversedynamics_tau2.png puma_inversedynamics_tau3.png puma_inversedynamics_tau.png
! convert -resize 1024x puma_inversedynamics_tau.png puma_inversedynamics_tau.png
! rm puma_inversedynamics_q1.png puma_inversedynamics_q2.png puma_inversedynamics_q3.png
! rm puma_inversedynamics_tau1.png puma_inversedynamics_tau2.png puma_inversedynamics_tau3.png
