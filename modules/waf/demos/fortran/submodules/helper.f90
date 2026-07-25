submodule (parent:container) helper
  implicit none

contains

  module function parent_distance(pa, pb) result(dist)
    type(parent_type), intent(in) :: pa, pb
    real :: dist

    dist = sqrt(parent_weight(pa) + parent_weight(pb))
  end function parent_distance

end submodule helper
