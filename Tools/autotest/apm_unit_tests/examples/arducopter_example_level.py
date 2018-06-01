import arducopter

def unit_test(mavproxy, mav):
    '''A scripted flight plan'''
    if  (arducopter.calibrate_level(mavproxy, mav)):
        return True
    return False

