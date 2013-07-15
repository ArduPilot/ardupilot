import arducopter

def unit_test(mavproxy, mav):
    '''A scripted flight plan'''
    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=30, takeoff_throttle=1510)  and
        arducopter.hover(mavproxy,mav, hover_throttle=1300) and
        arducopter.loiter(mavproxy, mav, holdtime=45, maxaltchange=20)): # 45 second loiter
        return True
    return False

