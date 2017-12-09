def bikeFE(x, y, psi, v, a, d_f, a0, Ff, theta, ts):

	# compute next state
	lr			= 1.738
	lf			= 1.738
	beta		= atan(lr*tan(d_f)/(lf+lr))
	x_next		= x + ts*v*cos(psi+beta)
	y_next		= y + ts*v*sin(psi+beta)
	psi_next	= psi + ts*(v/lr)*sin(beta)
	v_next		= v + ts*a

	return array([x_next, y_next, psi_next, v_next])