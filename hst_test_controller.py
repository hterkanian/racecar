#!/uar/bin/python

# Author Harry Terkanian
# July 31, 2017

# test controller

import math

err = [ 0.75, 0.5, 0.25, 0, -0.25, -0.5, -0.75, -1, -1.5 ]

def controller_out( err_in ):
    adjusted_error = ( (2 ** err_in) / 6 )
    U_t = math.copysign( adjusted_error, err_in )
    print( "Error: %.2f, u(t): %.2f" % ( err_in, U_t ) )

for err_val in err:
    controller_out( err_val )
