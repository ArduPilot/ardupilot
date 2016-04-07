set print pretty on

print "GPIOA registers:"
p/x *GPIOA->regs
print "GPIOB registers:"
p/x *GPIOB->regs
print "GPIOC registers:"
p/x *GPIOC->regs
print "GPIOD registers:"
p/x *GPIOD->regs
print "AFIO registers:"
p/x *(struct afio_reg_map*)0x40010000
