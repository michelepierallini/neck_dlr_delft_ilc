import numpy as np

def inertiaMatrix(q, L, I_xx, I_yy, I_zz, m_0):
    dL = q[2]
    phi = q[0]
    theta = q[1]
    
    t2 = np.cos(phi)
    t3 = np.cos(theta)
    t4 = np.sin(phi)
    t5 = np.sin(theta)
    t6 = L+dL
    t7 = theta*2.0
    t14 = 1.0/(theta**2)
    t15 = 1.0/(theta**3)
    t19 = theta/2.0
    t8 = t2**2
    t9 = t3**2
    t10 = t4**2
    t11 = np.sin(t7)
    t12 = t5**2
    t13 = t5*theta
    t16 = t14**2
    t17 = t3-1.0
    t18 = t6**2
    t20 = np.cos(t19)
    t21 = np.sin(t19)
    t22 = -t13
    t23 = t3*t8
    t24 = -t8
    t25 = t17**2
    t26 = t10-1.0
    t27 = t12-1.0
    t28 = t20**2
    t30 = t21**2
    t32 = t8*t9
    t29 = t28**2
    t31 = t30**2
    t33 = t30*4.0
    t36 = t30-1.0
    t37 = t8*t28*4.0
    t41 = t8+t23-1.0
    t45 = t24+t32+1.0
    t34 = t31*4.0
    t35 = -t33
    t38 = t8*t29*4.0
    t39 = t10*t33
    t42 = -t37
    t43 = t10*t31*-4.0
    t44 = t22+t33
    t46 = 1.0/(t45**2)
    t40 = t10*t34
    t47 = t38+t42+1.0
    t49 = m_0*t6*t15*(t13+t35)*(-9.99e+2/1.0e+3)
    t50 = m_0*t6*t15*(t13+t35)*(9.99e+2/1.0e+3)
    t51 = t34+t35+t39+t43+1.0
    t53 = I_xx*t2*t4*t5*t17*t41*t46*9.98001e-1
    t48 = 1.0/t47
    t52 = 1.0/t51
    t54 = -t53
    t55 = I_yy*t2*t3*t4*t20*t21*t48*1.996002
    t57 = I_zz*t2*t4*t11*t48*t52*4.990005e-1
    t56 = -t55
    t58 = t54+t56+t57
    return np.reshape(np.array([m_0*t14*t18*t25*(9.99e+2/1.0e+3)+I_xx*t25*t41**2*t46*9.98001e-1-I_zz*t8*t48**2*((np.cos(t7)**2)-1.0)*2.4950025e-1+I_yy*t28*t48*(t8-1.0)*(t28-1.0)*3.992004,
           t58,
           0.0,
           t58, 
           I_zz*t10*(t52**2)*9.98001e-1+m_0*t16*t18*((t5-t3*theta)**2)*(9.99e+2/1.0e+3)-(I_yy*t26*t27*9.98001e-1)/(t26*t36*4.0+t26*(t36**2)*4.0-1.0)+m_0*t16*t18*((t13+t17)**2)*(9.99e+2/1.0e+3)-I_xx*t10*t12*t26*1.0/(t10+t26*t27)**2*9.98001e-1,
           t50,
           0.0,
           t50,
           m_0*t14*t17*(-9.99e+2/5.0e+2)]),((3,3)))

 
def coriolisMatrix(q, q_dot, L, I_xx, I_yy, I_zz, m_0):
    
    dL = q[2]
    dL_dot = q_dot[2]
    phi = q[0]
    phi_dot = q_dot[0]
    theta = q[1]
    theta_dot = q_dot[1]
    
    t2 = np.cos(phi)
    t3 = np.cos(theta)
    t4 = np.sin(phi)
    t5 = np.sin(theta)
    t6 = L+dL
    t7 = L*2.0
    t8 = dL*2.0
    t9 = theta*2.0
    t20 = 1.0/(theta**2)
    t21 = 1.0/(theta**3)
    t23 = 1.0/(theta**5)
    t26 = theta/2.0
    t10 = t2**2
    t11 = t2**3
    t12 = np.cos(t9)
    t13 = t3**2
    t14 = t4**2
    t15 = t4**3
    t16 = np.sin(t9)
    t17 = t5**2
    t18 = t3*theta
    t19 = t5*theta
    t22 = t20**2
    t24 = t3-1.0
    t25 = t6**2
    t27 = t2*t4*2.0
    t28 = np.cos(t26)
    t29 = np.sin(t26)
    t39 = t6*2.0
    t82 = m_0*t5*t20*(9.99e+2/1.0e+3)
    t30 = t12**2
    t31 = -t18
    t32 = -t19
    t33 = t3*t10
    t34 = -t10
    t35 = t10-1.0
    t36 = t24**2
    t37 = t14-1.0
    t38 = t17-1.0
    t40 = t28**2
    t41 = t28**3
    t43 = t29**2
    t44 = t29**3
    t46 = t3*t27
    t47 = t10*t13
    t49 = t19+t24
    t53 = t13*t27
    t56 = t2*t4*t13*-2.0
    t66 = t28*t29*4.0
    t88 = m_0*t21*t24*(9.99e+2/5.0e+2)
    t102 = t24*t25*t82
    t42 = t40**2
    t45 = t43**2
    t48 = t5+t31
    t50 = t30-1.0
    t51 = t43*4.0
    t55 = t43-1.0
    t58 = t49**2
    t59 = t40-1.0
    t61 = t27*t38
    t62 = t10*t40*4.0
    t67 = t33+t35
    t70 = t27+t46
    t71 = t37*t38
    t72 = t2*t4*t40*8.0
    t74 = t2*t4*t43*8.0
    t76 = -t66
    t81 = t28*t44*8.0
    t83 = t10*t66
    t85 = t14*t66
    t86 = t34+t47+1.0
    t87 = t27+t56
    t95 = t10*t29*t41*8.0
    t99 = t14*t28*t44*-8.0
    t103 = m_0*t21*t25*t36*(9.99e+2/1.0e+3)
    t104 = m_0*phi_dot*t6*t20*t36*(9.99e+2/1.0e+3)
    t113 = t82+t88
    t52 = t45*4.0
    t54 = -t51
    t57 = t48**2
    t60 = t55**2
    t63 = t10*t42*4.0
    t64 = t14*t51
    t68 = -t62
    t69 = t14*t45*-4.0
    t73 = t2*t4*t42*8.0
    t75 = t2*t4*t45*8.0
    t77 = t67**2
    t80 = t32+t51
    t84 = t2*t4*t55*8.0
    t89 = t37*t55*4.0
    t90 = t27+t61
    t91 = t14+t71
    t93 = 1.0/(t86**2)
    t94 = 1.0/(t86**3)
    t96 = t14*t81
    t98 = -t95
    t109 = m_0*t6*t22*t58*(9.99e+2/1.0e+3)
    t133 = t76+t81+t85+t99
    t65 = t14*t52
    t78 = -t73
    t79 = -t75
    t92 = t2*t4*t60*8.0
    t97 = t37*t60*4.0
    t100 = 1.0/(t91**2)
    t101 = 1.0/(t91**3)
    t105 = t63+t68+1.0
    t108 = m_0*t6*t22*t57*(9.99e+2/1.0e+3)
    t115 = t83+t98
    t120 = t52+t54+t64+t69+1.0
    t125 = I_xx*t5*t24*t77*t93*9.98001e-1
    t128 = I_xx*t5*t10*t36*t67*t93*9.98001e-1
    t130 = I_xx*t5*t33*t36*t77*t94*1.996002
    t106 = t72+t78
    t107 = t74+t79
    t110 = 1.0/t105
    t114 = t84+t92
    t116 = t89+t97-1.0
    t119 = I_xx*t2*t15*t17*t100*9.98001e-1
    t121 = 1.0/t120
    t124 = I_xx*t2*t4*t17*t37*t100*9.98001e-1
    t129 = t108+t109
    t131 = -t130
    t134 = I_xx*t14*t17*t37*t90*t101*9.98001e-1
    t111 = t110**2
    t112 = t110**3
    t117 = 1.0/t116
    t122 = t121**2
    t123 = t121**3
    t132 = I_yy*t29*t35*t41*t110*1.996002
    t136 = -t134
    t139 = I_yy*t28*t29*t35*t59*t110*1.996002
    t118 = t117**2
    t126 = I_zz*t10*t12*t16*t111*4.990005e-1
    t135 = I_yy*t2*t4*t38*t117*9.98001e-1
    t137 = I_zz*t2*t4*t122*9.98001e-1
    t140 = I_zz*t10*t50*t112*t115*2.4950025e-1
    t142 = I_zz*t14*t107*t123*9.98001e-1
    t143 = I_yy*t35*t40*t59*t111*t115*1.996002
    t127 = -t126
    t138 = -t137
    t141 = -t140
    t144 = I_yy*t71*t114*t118*4.990005e-1
    t145 = -t144
    t147 = t102+t103+t125+t127+t128+t131+t132+t139+t141+t143
    t146 = t119+t124+t135+t136+t138+t142+t145
    et1 = t146+I_xx*t4*t11*t17*t24*t93*9.98001e-1+I_xx*t2*t4*t17*t67*t93*9.98001e-1-I_yy*t2*t3*t4*t40*t110*9.98001e-1+I_yy*t2*t3*t4*t43*t110*9.98001e-1+I_zz*t2*t4*t12*t110*t121*9.98001e-1-I_xx*t2*t3*t4*t24*t67*t93*9.98001e-1+I_yy*t2*t4*t5*t28*t29*t110*1.996002-I_zz*t2*t4*t16*t111*t115*t121*4.990005e-1
    et2 = I_zz*t2*t4*t16*t110*t122*(t66-t81+t96-t14*t28*t29*4.0)*4.990005e-1-I_xx*t3*t4*t11*t17*t24*t67*t94*3.992004+I_yy*t2*t3*t4*t28*t29*t111*t115*1.996002
    et3 = t147-I_yy*t28*t29*t33*t110*1.996002+I_zz*t10*t16*t110*t121*4.990005e-1-I_zz*t14*t16*t110*t121*4.990005e-1-I_xx*t5*t10*t24*t67*t93*9.98001e-1+I_xx*t5*t14*t24*t67*t93*9.98001e-1+I_yy*t3*t14*t28*t29*t110*1.996002+I_xx*t2*t4*t5*t24*t70*t93*9.98001e-1-I_zz*t2*t4*t16*t106*t111*t121*4.990005e-1
    et4 = I_zz*t2*t4*t16*t107*t110*t122*(-4.990005e-1)+I_xx*t2*t4*t5*t24*t67*t87*t94*1.996002+I_yy*t2*t3*t4*t28*t29*t106*t111*1.996002
    et5 = dL_dot*t129-phi_dot*t146
    et6 = theta_dot*(m_0*t23*t25*t57*(-9.99e+2/5.0e+2)-m_0*t23*t25*t58*(9.99e+2/5.0e+2)+I_yy*t71*t118*(t37*t66+t28*t29*t37*t55*8.0)*4.990005e-1+I_zz*t14*t123*(t66-t81+t96-t14*t28*t29*4.0)*9.98001e-1-I_yy*t3*t5*t37*t117*9.98001e-1+m_0*t3*t21*t25*t49*(9.99e+2/1.0e+3)+m_0*t5*t21*t25*t48*(9.99e+2/1.0e+3)+I_xx*t3*(t5**3)*t14*(t37**2)*t101*1.996002-I_xx*t3*t5*t14*t37*t100*9.98001e-1)
    return np.reshape(np.array([-t147*theta_dot-phi_dot*(I_xx*t36*t67*t70*t93*9.98001e-1+I_xx*t36*t77*t87*t94*9.98001e-1-I_zz*t2*t4*t50*t111*2.4950025e-1-I_zz*t10*t50*t106*t112*2.4950025e-1+I_yy*t2*t4*t40*t59*t110*3.992004+I_yy*t35*t40*t59*t106*t111*1.996002)+dL_dot*m_0*t6*t20*t36*(9.99e+2/1.0e+3),
                                -t146*theta_dot+phi_dot*(et3+et4),
                                -t104,
                                -phi_dot*t147+theta_dot*(et1+et2),et5+et6,
                                -theta_dot*(t129-m_0*t6*t21*(t5+t18+t76)*(9.99e+2/1.0e+3)+m_0*t6*t22*(t19+t54)*2.997)+dL_dot*t113,t104,
                                -dL_dot*(t113-m_0*t21*(t19+t54)*(9.99e+2/1.0e+3))+t129*theta_dot,
                                t113*theta_dot]),(3,3))

def tendonMatrix(in1,L,r_head_1,theta_head_1):

    dL = in1[2]
    phi = in1[0]
    theta = in1[1]
    
    t2 = np.cos(phi)
    t3 = np.cos(theta)
    t4 = np.cos(theta_head_1)
    t5 = np.sin(phi)
    t6 = np.sin(theta)
    t7 = np.sin(theta_head_1)
    t8 = L+dL
    t12 = 1.0/theta
    t15 = theta/2.0
    t9 = t2**2
    t10 = t3**2
    t11 = t3*theta
    t13 = t12**2
    t14 = t3-1.0
    t16 = np.cos(t15)
    t17 = np.sin(t15)
    t19 = t6*t12
    t18 = -t11
    t20 = t3*t9
    t21 = -t9
    t22 = t16**2
    t24 = -t19
    t25 = t9*t10
    t23 = t22**2
    t26 = t6+t18
    t27 = t9*t22*4.0
    t29 = t9+t20-1.0
    t32 = t21+t25+1.0
    t28 = t9*t23*4.0
    t30 = -t27
    t31 = t8*t13*t26
    t33 = 1.0/t32
    t34 = r_head_1*t2*t4*t5*t6*t33
    t35 = r_head_1*t2*t5*t6*t7*t33
    t36 = t28+t30+1.0
    t38 = r_head_1*t7*t14*t29*t33
    t41 = r_head_1*t4*t14*t29*t33
    t37 = 1.0/np.sqrt(t36)
    t39 = r_head_1*t2*t3*t4*t37
    t40 = r_head_1*t2*t3*t7*t37
    t42 = r_head_1*t4*t5*t16*t17*t37*2.0
    t43 = r_head_1*t5*t7*t16*t17*t37*2.0
    
    return np.reshape(np.array([-t38-t42, 
                                t31+t35+t39,
                                t24,
                                -t41+t43,
                                t31+t34-t40,
                                t24,
                                t38+t42,
                                t31-t35-t39,t24,
                                t41-t43,
                                t31-t34+t40,t24]),(3,4))

def tendonMatrixChatGPT(in1,L,r_head_1,theta_head_1):

    dL = in1[2]
    phi = in1[0]
    theta = in1[1]
    
    t2 = np.cos(phi)
    t3 = np.cos(theta)
    t4 = np.cos(theta_head_1)
    t5 = np.sin(phi)
    t6 = np.sin(theta)
    t7 = np.sin(theta_head_1)
    t8 = L+dL
    t12 = 1.0/theta
    t15 = theta/2.0
    t9 = t2**2
    t10 = t3**2
    t11 = t3*theta
    t13 = t12**2
    t14 = t3-1.0
    t16 = np.cos(t15)
    t17 = np.sin(t15)
    t19 = t6*t12
    t18 = -t11
    t20 = t3*t9
    t21 = -t9
    t22 = t16**2
    t24 = -t19
    t25 = t9*t10
    t23 = t22**2
    t26 = t6+t18
    t27 = t9*t22*4.0
    t29 = t9+t20-1.0
    t32 = t21+t25+1.0
    t28 = t9*t23*4.0
    t30 = -t27
    t31 = t8*t13*t26
    t33 = 1.0/t32
    t34 = r_head_1*t2*t4*t5*t6*t33
    t35 = r_head_1*t2*t5*t6*t7*t33
    t36 = t28+t30+1.0
    t38 = r_head_1*t7*t14*t29*t33
    t41 = r_head_1*t4*t14*t29*t33
    t37 = 1.0/np.sqrt(t36)
    t39 = r_head_1*t2*t3*t4*t37
    t40 = r_head_1*t2*t3*t7*t37
    t42 = r_head_1*t4*t5*t16*t17*t37*2.0
    t43 = r_head_1*t5*t7*t16*t17*t37*2.0
    
    return np.reshape(np.array([-t38-t42, 
                                t31+t35+t39,
                                t24,
                                -t41+t43,
                                t31+t34-t40,
                                t24,
                                t38+t42,
                                t31-t35-t39,t24,
                                t41-t43,
                                t31-t34+t40,t24]),(3,4))

def tendonMatrixLinear(in1,L,r_head_1,theta_head_1):

    dL = in1[2]
    phi = in1[0]
    theta = in1[1]

    t2 = np.cos(phi)
    t3 = np.cos(theta)
    t4 = np.cos(theta_head_1)
    t5 = np.sin(phi)
    t6 = np.sin(theta)
    t7 = np.sin(theta_head_1)
    t8 = theta**2
    t11 = -theta_head_1
    t12 = 1.0*theta
    t9 = L*t6
    t10 = dL*t6
    t13 = 1.0*t8
    t14 = dL*t3*theta
    t15 = L*t3*theta
    t16 = phi+t11
    t20 = t6*t12
    t23 = r_head_1*t2*t3*t4*t8
    t24 = r_head_1*t2*t3*t7*t8
    t25 = r_head_1*t3*t4*t5*t8
    t26 = r_head_1*t3*t5*t7*t8
    t17 = np.cos(t16)
    t18 = -t14
    t19 = np.sin(t16)
    t21 = -t15
    t22 = -t20
    t27 = r_head_1*t6*t17
    t28 = r_head_1*t6*t19
    return np.reshape(np.array([-t28,t13*(t9+t10+t18+t21+t23+t26),t22,
                                t27,t13*(t9+t10+t18+t21-t24+t25),
                                t22,t28,-t13*(-t9-t10+t14+t15+t23+t26),t22,
                                -t27,t13*(t9+t10+t18+t21+t24-t25),t22]),(3,4))


def directKine(in1,L):
    dL = in1[2]
    phi = in1[0]
    theta = in1[1]
    t2 = np.cos(phi)
    t3 = np.cos(theta)
    t4 = np.sin(phi)
    t5 = L*2.0
    t6 = dL*2.0
    t7 = phi*2.0
    t10 = 1.0/theta
    t11 = theta/2.0
    t8 = t2**2
    t9 = np.sin(t7)
    t12 = np.cos(t11)
    t13 = t3/2.0
    t14 = np.sin(t11)
    t15 = t5+t6
    t16 = t13-1.0/2.0
    t18 = t2*t12*t14*2.0
    t19 = t4*t12*t14*2.0
    t17 = t9*t16
    return np.reshape([-t8+t3*t8+1.0,
                    t17,
                    -t18,
                    0.0,
                    t17, 
                    (t4**2)*(t3-1.0)+1.0,
                    -t19,
                    0.0,
                    t18,
                    t19,
                    t3,
                    0.0,
                    -t2*t10*t15*t16,
                    -t4*t10*t15*t16,
                    t10*np.sin(theta)*(L+dL),
                    1.0],[4,4])

def tendonMatrixLinearChatGPT(in1, L, r_head_1, theta_head_1):
    dL, phi, theta = in1

    c_phi = np.cos(phi)
    c_theta = np.cos(theta)
    c_theta_head = np.cos(theta_head_1)
    s_phi = np.sin(phi)
    s_theta = np.sin(theta)
    s_theta_head = np.sin(theta_head_1)

    t1 = L * s_theta + dL * s_theta * c_theta
    t2 = L * c_theta + dL * s_theta ** 2
    t3 = c_phi * s_theta_head
    t4 = s_phi * s_theta_head
    t5 = c_theta * c_theta_head
    t6 = c_phi * c_theta_head
    t7 = s_phi * c_theta_head

    t8 = theta ** 2
    t9 = c_theta * t8
    t10 = s_theta * theta_head_1
    t11 = np.cos(phi + t10)
    t12 = np.sin(phi + t10)

    t13 = r_head_1 * t9
    t14 = r_head_1 * t8 * s_theta * s_theta_head
    t15 = r_head_1 * t5 * s_phi
    t16 = r_head_1 * t5 * c_phi
    t17 = r_head_1 * t3 * t11
    t18 = r_head_1 * t4 * t12

    return np.array([[-t18, t2 * t5 + t3 * t1 + t13 - t16, -t7],
                     [t17, t2 * t6 + t4 * t1 + t14 + t15, -t6],
                     [t18, -t3 * t1 + t2 * t5 + t13 + t16, -t7],
                     [-t17, t2 * t4 - t6 * t1 - t14 + t15, -t6]])



def forwardKine(in1,L):
    dL = in1[2]
    phi = in1[0]
    theta = in1[1]
    t2 = np.cos(theta)
    t3 = L*2.0
    t4 = dL*2.0
    t5 = 1.0/theta
    t6 = t2/2.0
    t7 = t3+t4
    t8 = t6-1.0/2.0
    return np.asarray([-t5*t7*t8*np.cos(phi),
            -t5*t7*t8*np.sin(phi),
            t5*np.sin(theta)*(L+dL)])

def stiffnessTorque(in1,kappa_theta,kappa_phi,kappa_dL):
    dL = in1[2]
    phi = in1[0]
    theta = in1[1]
    return np.asanyarray([kappa_phi*phi, kappa_theta*theta, dL*kappa_dL])
 
def gravityVector(in1,L,m_0,g):
    dL = in1[2]
    theta = in1[1]
    
    t2 = np.sin(theta)
    t3 = L+dL
    t4 = 1.0/theta
    return np.asanyarray([0.0, -g*m_0*t2*t3*(t4**2)+g*m_0*t3*t4*np.cos(theta), g*m_0*t2*t4]).reshape((-1,))

def dampingTorque(in1,d_ii):
    dL_dot = in1[2]
    phi_dot = in1[0]
    theta_dot = in1[1]
    return np.asanyarray([d_ii*phi_dot, d_ii*theta_dot, dL_dot*d_ii])

# if __name__ == '__main__':
#     q = np.array([0.022, 0.085, 0]) # this should be the inital position from my test
#     # q = np.array([1.5, 0.4, -0.035]) # this should be the final position from my test
#     q_dot = np.zeros(3)
#     d_ii = 0.1
#     kappa_theta = 1
#     kappa_phi = 1
#     kappa_dL = 1
#     g = 0 # 9.81
#     L = 0.3 # 1.0
#     m_0 = 1.0
#     I_xx = 1/12 * m_0 * L**2
#     I_yy = 1/12 * m_0 * L**2
#     I_zz = 1/50 * m_0 * L**2
#     r_head_1 = 0.05
#     theta_head_1 = np.pi/4
    
#     G = gravityVector(q,L,m_0,g)
#     K = stiffnessTorque(q,kappa_theta,kappa_phi,kappa_dL)
#     D = dampingTorque(q_dot, d_ii)
#     M = inertiaMatrix(q, L, I_xx, I_yy, I_zz, m_0)
#     C = coriolisMatrix(q, q_dot, L, I_xx, I_yy, I_zz, m_0)
#     Alin = tendonMatrixLinear(q,L,r_head_1,theta_head_1)
#     A = tendonMatrix(q,L,r_head_1,theta_head_1)
#     pos = forwardKine(q, L)
#     print('=======================================================================')
#     print('Tendon Matrix Lin:\n',Alin, np.shape(Alin))
#     print('Tendon Matrix:\n', A, np.shape(A))
#     print('Inertial:\n', M, np.shape(M))
#     print('Coriolis:\n', C, np.shape(C))
#     print('Damping:\n', D, np.shape(D))
#     print('Stifnness:\n', K, np.shape(K))
#     print('Gravity:\n', G, np.shape(G))
#     print('=======================================================================')
#     print('=======================================================================')
#     print('Forward Kine:\n', pos, np.shape(pos))


