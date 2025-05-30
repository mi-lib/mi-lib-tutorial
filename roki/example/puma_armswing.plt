FILE='puma_armswing.dat'

set terminal png

set output 'puma_armswing_x_pos_vel.png'
plot 0 lt 8 t '', FILE u 1 w l lt 1 t 'x-position', FILE u 4 w l lt 2 t 'x-velocity'
set output 'puma_armswing_x_vel_acc.png'
plot 0 lt 8 t '', FILE u 4 w l lt 2 t 'x-velocity', FILE u 10 w l lt 4 t 'x-acceleration'
set output 'puma_armswing_x_vel.png'
plot [][-1:1.2] 0 lt 8 t '', FILE u 4 w l lt 2 t 'x-velocity', FILE u 7 w l lt 3 t 'approx. x-velocity'
set output 'puma_armswing_x_acc.png'
plot 0 lt 8 t '', FILE u 10 w l lt 4 t 'x-acceleration', FILE u 13 w l lt 6 t 'approx. x-acceleration'

set output 'puma_armswing_y_pos_vel.png'
plot 0 lt 8 t '', FILE u 2 w l lt 1 t 'y-position', FILE u 5 w l lt 2 t 'y-velocity'
set output 'puma_armswing_y_vel_acc.png'
plot 0 lt 8 t '', FILE u 5 w l lt 2 t 'y-velocity', FILE u 11 w l lt 4 t 'y-acceleration'
set output 'puma_armswing_y_vel.png'
plot 0 lt 8 t '', FILE u 5 w l lt 2 t 'y-velocity', FILE u 8 w l lt 3 t 'approx. y-velocity'
set output 'puma_armswing_y_acc.png'
plot [][-8:9] 0 lt 8 t '', FILE u 11 w l lt 4 t 'y-acceleration', FILE u 14 w l lt 6 t 'approx. y-acceleration'

set output 'puma_armswing_z_pos_vel.png'
plot 0 lt 8 t '', FILE u 3 w l lt 1 t 'z-position', FILE u 6 w l lt 2 t 'z-velocity'
set output 'puma_armswing_z_vel_acc.png'
plot 0 lt 8 t '', FILE u 6 w l lt 2 t 'z-velocity', FILE u 12 w l lt 4 t 'z-acceleration'
set output 'puma_armswing_z_vel.png'
plot [][-2:2] 0 lt 8 t '', FILE u 6 w l lt 2 t 'z-velocity', FILE u 9 w l lt 3 t 'approx. z-velocity'
set output 'puma_armswing_z_acc.png'
plot [][-10:10] 0 lt 8 t '', FILE u 12 w l lt 4 t 'z-acceleration', FILE u 15 w l lt 6 t 'approx. z-acceleration'

!convert +append puma_armswing_x_pos_vel.png puma_armswing_y_pos_vel.png puma_armswing_z_pos_vel.png puma_armswing_pos_vel.png
!convert -resize 1024x puma_armswing_pos_vel.png puma_armswing_pos_vel.png
!convert +append puma_armswing_x_vel_acc.png puma_armswing_y_vel_acc.png puma_armswing_z_vel_acc.png puma_armswing_vel_acc.png
!convert -resize 1024x puma_armswing_vel_acc.png puma_armswing_vel_acc.png

!convert +append puma_armswing_x_vel.png puma_armswing_y_vel.png puma_armswing_z_vel.png puma_armswing_vel.png
!convert -resize 1024x puma_armswing_vel.png puma_armswing_vel.png
!convert +append puma_armswing_x_acc.png puma_armswing_y_acc.png puma_armswing_z_acc.png puma_armswing_acc.png
!convert -resize 1024x puma_armswing_acc.png puma_armswing_acc.png

!rm puma_armswing_x_pos_vel.png puma_armswing_y_pos_vel.png puma_armswing_z_pos_vel.png
!rm puma_armswing_x_vel_acc.png puma_armswing_y_vel_acc.png puma_armswing_z_vel_acc.png
!rm puma_armswing_x_vel.png puma_armswing_y_vel.png puma_armswing_z_vel.png
!rm puma_armswing_x_acc.png puma_armswing_y_acc.png puma_armswing_z_acc.png
