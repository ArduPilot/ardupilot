        module calculator
          implicit none
        contains
          subroutine add (a, b, output)
            integer, intent (in) :: a, b
            integer, intent (out) :: output
            output = a + b
          end subroutine add
        end module calculator
