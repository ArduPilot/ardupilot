submodule (parent) container
  implicit none

contains

    module procedure init
      p%mother = mother
      p%father = father
    end procedure init

    module subroutine harmonize(p)
      type(parent_type), intent(inout) :: p
      real :: avg

      avg = 0.5 * (p%father + p%mother)
      p%father = avg
      p%mother = avg
    end subroutine harmonize

    module function parent_weight(p) result(w)
      type(parent_type), intent(in) :: p
      real :: w

      w = p%mother**2 + p%father**2
    end function parent_weight

end submodule container
