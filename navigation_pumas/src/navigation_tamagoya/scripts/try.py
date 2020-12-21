from navigation_tamagoya.nav_tool_lib import nav_module

import hsrb_interface

robot = hsrb_interface.Robot()
omni_base=nav_module("pumas")
#omni_base=robot.get("omni_base")


#print "---------------------abs"
#omni_base.go_abs(1.47, -2.53, -1.0, 1)
#omni_base.go_abs(0.73,1.49,0.88, "hsr")
#omni_base.go_abs(1.59,-4,0)
#print "-----------------------rel"

#omni_base.go_abs(0.73,1.49,0.88)
omni_base.go_rel(.5,0,0, .3)
#omni_base.set_navigation_type("hsr")

#omni_base.go_abs(1.37, -2.53, -1.0, "pumas")
#omni_base.go_abs(0.2 ,0.5 ,0.88, "pumas")
#omni_base.go_abs(0.63,1.39,0.88, "hsr")
#print "first instruction"
#omni_base.go_rel(.5 ,0 ,1.57)

#omni_base.go_rel(0 ,0.5 ,-1.57, "hsr")
