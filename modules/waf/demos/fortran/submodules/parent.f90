module parent
  implicit none

  type parent_type
    real :: mother
    real :: father
  end type parent_type

  interface
    module subroutine init(p, mother, father)
      type(parent_type), intent(out) :: p
      real, intent(in) :: mother, father
    end subroutine init

    module subroutine harmonize(p)
      type(parent_type), intent(inout) :: p
    end subroutine harmonize

    module function parent_weight(p) result(w)
      type(parent_type), intent(in) :: p
      real :: w
    end function parent_weight

    module function parent_distance(pa, pb) result(dist)
      type(parent_type), intent(in) :: pa, pb
      real :: dist
    end function parent_distance
  end interface

end module parent
