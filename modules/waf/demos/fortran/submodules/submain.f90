program submain
  use parent

  implicit none

  type(parent_type) :: a,b
  real :: dist, weight

  call init(a, 1.0, 2.0)
  call init(b, 10.0, 12.0)

  call harmonize(a)
  weight = parent_weight(b)
  write(*,*) weight
  dist = parent_distance(a, b)
  write(*,*) dist

end program submain
