FILE='puma_fkcnt.dat'

set terminal png

set output "puma_fkcnt_tau1.png"
plot [][-5:5] FILE u 1:5 w l lt 1 t 'tau1 [N.m]'
set output "puma_fkcnt_tau2.png"
plot [][-35:-5] FILE u 1:6 w l lt 2 t 'tau2 [N.m]'
set output "puma_fkcnt_tau3.png"
plot [][-5:5] FILE u 1:7 w l lt 4 t 'tau3 [N.m]'

! convert +append puma_fkcnt_tau1.png puma_fkcnt_tau2.png puma_fkcnt_tau3.png puma_fkcnt_tau.png
! convert -resize 1024x puma_fkcnt_tau.png puma_fkcnt_tau.png
! rm puma_fkcnt_tau1.png puma_fkcnt_tau2.png puma_fkcnt_tau3.png
