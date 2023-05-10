import numpy as np
from numba import jit 

def jacobianDiffKine(q,L,r):

    Dx = q[0]
    Dy = q[1]
    dL = q[2]

    t2 = L+dL
    t3 = Dx**2
    t4 = Dy**2
    t5 = L*2.0
    t6 = dL*2.0
    t7 = r**2
    t8 = 1.0/r
    t9 = t3+t4
    t10 = t2*2.0
    t11 = 1.0/t9
    t12 = np.sqrt(t9)
    t13 = 1.0/t12
    t14 = t3*t11
    t16 = t8*t12
    t15 = -t14
    t17 = np.cos(t16)
    t18 = np.sin(t16)
    t19 = t15+1.0
    t20 = t17-1.0
    t21 = t17/2.0
    t24 = t16*t18
    t22 = np.sqrt(t19)
    t23 = t21-1.0/2.0
    t25 = t20+t24
    return np.array([r*t10*t13*t22*t23,
        Dx*r*t2*t11*t23*-2.0,
        0.0,
        1.0,
        0.0,
        0.0,
        Dx*t2*t7*t13**3*t25,
        t2*t7*t11*t22*t25,
        -t2*t7*t11*(t18-t16*t17),
        0.0,
        1.0,
        0.0,
        -Dx*r*t11*t20,
        -r*t13*t20*t22,
        r*t13*t18,
        0.0,
        0.0,
        0.0]).reshape((6,3))

# @jit(nopython=True)
def inertiaMatrix(q,L,r,I_xx,I_yy,I_zz,m_0):
    Dx = q[0]
    Dy = q[1]
    dL = q[2]

    t2 = Dy
    t3 = L+dL
    t4 = Dx**2
    t5 = Dy**2
    t8 = r*2.0
    t9 = r**2
    t10 = -r
    t12 = 1.0/r
    t11 = -t8
    t13 = t3**2
    t14 = t4+t5
    t16 = 1.0/t14
    t19 = np.sqrt(t14)
    t17 = t16**2
    t18 = t16**3
    t20 = t12*t19
    t21 = np.cos(t20)
    t22 = np.sin(t20)
    t24 = t20/2.0
    t23 = t22**2
    t25 = r*t21
    t26 = r*t22
    t27 = t21**2
    t28 = t8*t21
    t29 = np.cos(t24)
    t30 = np.sin(t24)
    t31 = t10*t22
    t32 = t4*t21
    t33 = t5*t21
    t34 = t21/2.0
    t45 = t19*t21
    t46 = t19*t22
    t35 = t30**4
    t36 = t29**2
    t37 = t9*t23
    t38 = t27/4.0
    t40 = t5+t32
    t41 = t4+t33
    t42 = t34-1.0/2.0
    t48 = t8*t46
    t49 = t19*t26*-2.0
    t52 = t31+t45
    t53 = (t26-t45)**2
    t54 = t10+t25+t46
    t55 = t11+t28+t46
    t39 = t36*2.0
    t44 = t38-1.0/4.0
    t56 = t14+t37+t49
    t57 = Dx*I_xx*t2*t17*t40*t42*2.0
    t58 = Dx*I_yy*t2*t17*t41*t42*2.0
    t59 = Dx*m_0*r*t3*t17*t55
    t60 = Dy*m_0*r*t3*t17*t55
    t43 = t39-1.0
    t50 = Dx*I_zz*t2*t16*t44*4.0
    t61 = Dx*Dy*m_0*t13*t18*t56
    t47 = t43**2
    t51 = -t50
    t62 = t51+t57+t58+t61
    return np.array([m_0*(t3*t4*t17*t54-Dy*r*t2*t3*t17*t42*2.0)**2+m_0*(Dx*t2*t3*t17*t54+Dx*Dy*t3*t8*t17*t42)**2+I_xx*t17*(t5+t4*t43)**2-I_zz*t4*t16*(t47/4.0-1.0/4.0)*4.0+I_yy*t4*t5*t17*t35*4.0+m_0*t4*t13*t18*t53,
                    t62,
                    t59,
                    t62,
                    m_0*(Dy*t2*t3*t17*t54-t3*t4*t8*t17*t42)**2+m_0*(Dx*Dy*t3*t17*t54+Dx*t2*t3*t8*t17*t42)**2+I_yy*t17*(t4+t5*t43)**2-(I_zz*t5*t16*(t47*2.0-2.0))/2.0+I_xx*t4*t5*t17*t35*4.0+m_0*t5*t13*t18*t53,
                    t60,
                    t59,
                    t60,
                    m_0*t9*t16*(t21-1.0)*-2.0]).reshape((3,3))


# @jit(nopython=True)
def coriolisMatrix(q,q_dot,L,r,I_xx,I_yy,I_zz,m_0):

    Dx = q[0]
    Dx_dot = q_dot[0]
    Dy = q[1]
    Dy_dot = q_dot[1]
    dL = q[2]
    dL_dot = q_dot[2]

    t2 = Dy
    t3 = Dy
    t4 = L+dL
    t5 = Dx*2.0
    t6 = Dy*2.0
    t7 = Dx**2
    t8 = Dx**3
    t9 = Dy**2
    t10 = Dy**3
    t13 = r*2.0
    t14 = r**2
    t15 = -r
    t17 = 1.0/r
    t16 = -t13
    t18 = t4**2
    t19 = t7+t9
    t21 = 1.0/t19
    t25 = np.sqrt(t19)
    t22 = t21**2
    t23 = t21**3
    t26 = 1.0/t25
    t29 = t17*t25
    t24 = t22**2
    t27 = t26**3
    t28 = t26**5
    t30 = np.cos(t29)
    t31 = np.sin(t29)
    t33 = t29/2.0
    t32 = t31**2
    t34 = r*t30
    t35 = r*t31
    t36 = t30**2
    t37 = t5*t30
    t38 = t6*t30
    t39 = t13*t30
    t40 = np.cos(t33)
    t41 = np.sin(t33)
    t42 = t30-1.0
    t43 = t15*t31
    t44 = t7*t30
    t45 = t9*t30
    t46 = t30/2.0
    t50 = Dx*t17*t30
    t51 = Dy*t17*t30
    t60 = t25*t30
    t61 = t25*t31
    t63 = Dx*t26*t31
    t64 = Dy*t26*t31
    t97 = t4*t8*t28*t31
    t98 = Dx*Dy*t2*t4*t28*t31
    t99 = Dx*t4*t9*t28*t31
    t100 = Dy*t4*t7*t28*t31
    t101 = t2*t4*t7*t28*t31
    t102 = t2*t4*t9*t28*t31
    t47 = t41**3
    t48 = t41**4
    t49 = t40**2
    t52 = t14*t32
    t53 = t36/4.0
    t55 = t9+t44
    t56 = t7+t45
    t57 = t46-1.0/2.0
    t69 = t25*t35*-2.0
    t70 = Dx*m_0*t27*t35
    t71 = Dy*m_0*t27*t35
    t72 = -t63
    t73 = -t64
    t80 = m_0*t5*t14*t22*t42
    t81 = m_0*t6*t14*t22*t42
    t83 = Dy*t2*t4*t22*t50
    t86 = Dy*t4*t17*t22*t44
    t103 = -t99
    t104 = -t101
    t105 = -t102
    t131 = (t35-t60)**2
    t133 = t15+t34+t61
    t134 = t16+t39+t61
    t141 = Dx*t9*t17*t26*t40*t41*-2.0
    t142 = Dy*t7*t17*t26*t40*t41*-2.0
    t184 = Dx*m_0*t9*t17*t18*t23*t31*(t35-t60)
    t185 = Dy*m_0*t7*t17*t18*t23*t31*(t35-t60)
    t54 = t49*2.0
    t59 = t53-1.0/4.0
    t84 = Dx*I_xx*t9*t22*t48*4.0
    t85 = Dy*I_yy*t7*t22*t48*4.0
    t87 = Dy*r*t5*t22*t57
    t88 = -t86
    t89 = I_xx*t8*t9*t23*t48*8.0
    t90 = I_yy*t7*t10*t23*t48*8.0
    t91 = r*t2*t5*t22*t57
    t92 = r*t2*t6*t22*t57
    t93 = Dx*r*t5*t22*t57
    t96 = Dy*r*t2*t22*t57*-2.0
    t106 = r*t4*t5*t22*t57
    t107 = Dx*r*t4*t22*t57*4.0
    t109 = t2*t4*t13*t22*t57
    t114 = r*t3*t4*t6*t22*t57
    t115 = r*t4*t8*t23*t57*8.0
    t116 = Dx*Dy*r*t2*t4*t23*t57*8.0
    t117 = r*t4*t7*t22*t57*-2.0
    t118 = Dx*r*t4*t9*t23*t57*8.0
    t119 = Dy*r*t4*t7*t23*t57*8.0
    t123 = r*t2*t4*t7*t23*t57*8.0
    t124 = r*t2*t4*t9*t23*t57*8.0
    t136 = t50+t72
    t137 = t51+t73
    t138 = t19+t52+t69
    t143 = Dx*Dy*t22*t133
    t144 = t5+t141
    t145 = t6+t142
    t146 = I_xx*t8*t9*t17*t28*t40*t47*4.0
    t147 = I_yy*t7*t10*t17*t28*t40*t47*4.0
    t148 = t7*t22*t133
    t149 = Dx*t2*t22*t133
    t150 = Dy*t2*t22*t133
    t151 = Dy*t4*t22*t133
    t153 = t2*t4*t22*t133
    t158 = Dx*t3*t4*t22*t133
    t160 = t70+t80
    t161 = t71+t81
    t162 = Dy*t4*t7*t23*t133*4.0
    t163 = Dx*Dy*t2*t4*t23*t133*4.0
    t166 = Dx*Dy*m_0*r*t4*t23*t134*4.0
    t168 = Dx*m_0*t9*t18*t24*t131*3.0
    t169 = Dy*m_0*t7*t18*t24*t131*3.0
    t176 = m_0*t4*t7*t23*t131
    t177 = m_0*t4*t9*t23*t131
    t58 = t54-1.0
    t94 = -t89
    t95 = -t90
    t108 = t4*t87
    t110 = -t107
    t112 = t4*t91
    t113 = t4*t92
    t120 = t4*t96
    t126 = -t118
    t129 = -t123
    t130 = -t124
    t152 = t4*t143
    t154 = -t150
    t156 = t4*t149
    t157 = t4*t150
    t159 = t4*t148
    t164 = -t162
    t165 = -t163
    t167 = -t166
    t170 = -t168
    t171 = -t169
    t172 = (Dy*m_0*r*t4*t22*t136)/2.0
    t173 = (Dx*m_0*r*t4*t22*t137)/2.0
    t178 = Dx*Dy*m_0*t4*t23*t138
    t186 = t91+t143
    t187 = t87+t149
    t188 = t96+t148
    t62 = t58**2
    t67 = t7*t58
    t68 = t9*t58
    t174 = -t172
    t175 = -t173
    t179 = -t178
    t181 = Dx*Dy*I_zz*t5*t17*t27*t40*t41*t58
    t189 = t93+t154
    t192 = t108+t156
    t193 = t112+t152
    t194 = t120+t159
    t200 = m_0*(t150-r*t7*t22*t57*2.0)*(t157-t4*t7*t13*t22*t57)
    t203 = t83+t97+t110+t115+t165
    t204 = t88+t105+t109+t114+t130+t162
    t206 = t83+t103+t106+t126+t158+t165
    t207 = t86+t104+t109+t129+t151+t164
    t65 = t62*2.0
    t74 = t62/4.0
    t76 = t7+t68
    t77 = t9+t67
    t180 = I_zz*t5*t17*t27*t40*t41*t68
    t196 = m_0*t186*t193
    t197 = m_0*t187*t192
    t198 = m_0*t188*t194
    t201 = t173+t174+t178
    t202 = t172+t175+t178
    t205 = t167+t172+t173+t179
    t208 = m_0*t203*(t157-t4*t7*t13*t22*t57)
    t209 = -m_0*t194*(t86+t102-t109-t114+t124+t164)
    t210 = m_0*t194*(t86+t102-t109-t114+t124+t164)
    t211 = m_0*t192*t206
    t212 = m_0*t193*t207
    t75 = t65-2.0
    t78 = t76**2
    t79 = t77**2
    t82 = t74-1.0/4.0
    t190 = I_yy*t22*t76*t144
    t191 = I_xx*t22*t77*t145
    t213 = t176+t197+t198
    t214 = t177+t196+t200
    t121 = I_yy*t5*t23*t78
    t122 = I_xx*t6*t23*t79
    t127 = Dx*I_yy*t23*t78*-2.0
    t128 = Dy*I_xx*t23*t79*-2.0
    t132 = (Dx*I_zz*t9*t22*t75)/2.0
    t135 = Dy*I_zz*t7*t22*t82*4.0
    t215 = t84+t94+t127+t132+t146+t170+t180+t184+t190+t208+t212
    t216 = t85+t95+t128+t135+t147+t171+t181+t185+t191+t210+t211
    return np.array([
        Dy_dot*t216+dL_dot*t213+Dx_dot*(-m_0*t192*(t100+t119-t153-r*t4*t6*t22*t57-t2*t4*t17*t22*t44+t2*t4*t7*t23*t133*4.0)+m_0*t194*(t98+t116+t4*t5*t22*t133-t4*t8*t23*t133*4.0+t4*t8*t17*t22*t30)+I_zz*t8*t22*t82*4.0-Dx*I_xx*t23*t79*2.0-Dx*I_zz*t21*t82*4.0+I_xx*t22*t77*(t5*t58-t8*t17*t26*t40*t41*2.0)+Dx*I_yy*t9*t22*t48*4.0+Dx*m_0*t18*t23*t131-I_yy*t8*t9*t23*t48*8.0-m_0*t8*t18*t24*t131*3.0+I_yy*t8*t9*t17*t28*t40*t47*4.0+I_zz*t5*t17*t27*t40*t41*t67+m_0*t8*t17*t18*t23*t31*(t35-t60)),
        -Dx_dot*(t85+t95-t122+t135+t147+t171+t185+t191+t210+t211+I_zz*t2*t21*t59*4.0-Dy*m_0*t18*t23*t138-I_xx*t2*t22*t55*t57*2.0-I_yy*t2*t22*t56*t57*2.0-I_zz*t2*t7*t22*t59*8.0-I_yy*t2*t5*t22*t57*(t5+t9*t17*t72)-I_xx*t2*t5*t22*t57*(t37-t8*t17*t26*t31)+Dy*m_0*t7*t18*t24*t138*6.0-Dx*Dy*m_0*t18*t23*(t5-Dx*t30*2.0-Dx*t26*t35*2.0+t5*t26*t31*t34)+I_xx*t2*t7*t23*t55*t57*8.0+I_yy*t2*t7*t23*t56*t57*8.0-Dx*I_zz*t2*t17*t27*t31*t37+Dy*I_zz*t17*t27*t40*t41*t67*2.0+I_xx*t2*t7*t17*t28*t31*t55+I_yy*t2*t7*t17*t28*t31*t56)+Dy_dot*t215+dL_dot*t202,
        -Dx_dot*(t213+m_0*t4*t15*t22*t134+m_0*r*t4*t7*t23*t134*4.0-Dx*m_0*r*t4*t22*t136)+dL_dot*t160-Dy_dot*(t166+t174+t175+t178),
        Dx_dot*t216+Dy_dot*(-t215-Dx*I_zz*t3*t21*t59*4.0+Dx*m_0*t18*t23*t138+Dx*I_yy*t2*t22*t57*(t38-t10*t17*t26*t31)*2.0+Dx*Dy*I_zz*t2*t22*t59*8.0+Dx*I_xx*t3*t22*t55*t57*2.0+Dx*I_yy*t3*t22*t56*t57*2.0+Dy*I_zz*t2*t27*t31*t50*2.0+Dx*I_xx*t2*t22*t57*(t6+t7*t17*t73)*2.0-Dx*m_0*t9*t18*t24*t138*6.0+Dx*Dy*m_0*t18*t23*(t6-Dy*t30*2.0-Dy*t26*t35*2.0+t6*t26*t31*t34)-Dx*Dy*I_xx*t2*t23*t55*t57*8.0-Dx*Dy*I_yy*t2*t23*t56*t57*8.0-Dx*Dy*I_xx*t2*t17*t28*t31*t55-Dx*Dy*I_yy*t2*t17*t28*t31*t56)+dL_dot*t201,
        Dx_dot*t215+Dy_dot*(m_0*(t157-t4*t7*t13*t22*t57)*(t100+t119+t153+t3*t151+t2*t4*t17*t22*t45-t2*t4*t9*t23*t133*4.0)-m_0*t193*(t98+t116-t3*t106-Dx*t4*t22*t133-Dx*t4*t17*t22*t45+Dx*t4*t9*t23*t133*4.0)+(I_zz*t10*t22*t75)/2.0-Dy*I_yy*t23*t78*2.0-(Dy*I_zz*t21*t75)/2.0+I_yy*t22*t76*(t6*t58-t10*t17*t26*t40*t41*2.0)+Dy*I_xx*t7*t22*t48*4.0+Dy*m_0*t18*t23*t131-I_xx*t7*t10*t23*t48*8.0-m_0*t10*t18*t24*t131*3.0+I_xx*t7*t10*t17*t28*t40*t47*4.0+I_zz*t6*t17*t27*t40*t41*t68+m_0*t10*t17*t18*t23*t31*(t35-t60))+dL_dot*t214,
        -Dy_dot*(t214+m_0*t4*t15*t22*t134+m_0*r*t4*t9*t23*t134*4.0-Dy*m_0*r*t4*t22*t137)+dL_dot*t161-Dx_dot*(t166+t174+t175+t178),-dL_dot*(t160+Dx*m_0*t15*t22*t134)+Dx_dot*t213+Dy_dot*t201,-dL_dot*(t161+Dy*m_0*t15*t22*t134)+Dx_dot*t202+Dy_dot*t214,Dx_dot*t160+Dy_dot*t161]).reshape((3,3))

# # @jit(nopython=True)
def tendonMatrix(in1,r,L,theta_base,theta_head,r_base,r_head):

    '''
    This is done like DLR-decoupling matrix 
    '''
    Dx = in1[0]
    Dy = in1[1]
    dL = in1[2]

    t2 = np.abs(Dy)
    t3 = np.cos(theta_head)
    t4 = np.cos(theta_base)
    t5 = np.sin(theta_head)
    t6 = np.sin(theta_base)

    t7 = L+dL
    t8 = Dx**2
    t9 = Dy**2
    t12 = Dx*L*r
    t13 = Dx*dL*r
    t17 = -L
    t19 = 1.0/r
    t14 = Dx*t3
    t15 = Dx*t5
    t16 = r_base*t4
    t18 = r_base*t6
    t20 = L*r*t2
    t21 = dL*r*t2
    t22 = t2*t3
    t23 = t2*t5
    t24 = t5*t12
    t25 = -t12
    t26 = t5*t13
    t27 = -t13
    t30 = r_head*t3*t8
    t32 = r_head*t3*t9
    t34 = r_head*t5*t8
    t36 = r_head*t5*t9
    t40 = t8+t9
    t42 = r*t2*t17
    t43 = t7*2.0
    t28 = -t16
    t29 = t8*t16
    t31 = t9*t16
    t33 = t8*t18
    t35 = t9*t18
    t37 = r_head*t2*t14
    t38 = t3*t20
    t39 = r_head*t2*t15
    t41 = t3*t21
    t44 = -t21
    t45 = -t22
    t46 = -t23
    t47 = -t24
    t48 = -t26
    t51 = -t30
    t52 = -t32
    t53 = -t34
    t54 = -t36
    t55 = t2*t14*t16
    t56 = t2*t15*t18
    t58 = r*t17*t22
    t61 = 1.0/t40
    t63 = t14+t23
    t64 = np.sqrt(t40)
    t49 = t5*t29
    t50 = t3*t35
    t57 = -t37
    t59 = -t39
    t60 = -t41
    t62 = t61**2
    t66 = t2*t14*t28
    t67 = 1.0/t64
    t68 = t14+t46
    t69 = t15+t45
    t70 = L*t64
    t71 = t8*t61
    t74 = t17*t64
    t76 = t19*t64
    t65 = -t50
    t72 = Dx*t67
    t75 = -t71
    t77 = t71-1.0
    t79 = np.cos(t76)
    t80 = np.sin(t76)
    t83 = t76/2.0
    t73 = np.cos(t72)
    t81 = t75+1.0
    t84 = np.cos(t83)
    t85 = np.sin(t83)
    t86 = L*r*t80
    t87 = dL*r*t80
    t88 = t79-1.0
    t89 = t13*t79
    t90 = t8*t79
    t91 = t9*t79
    t92 = t79/2.0
    t93 = t12*t79
    t94 = t20*t79
    t95 = t21*t79
    t96 = r*t17*t80
    t99 = t27*t79
    t100 = t24*t79
    t101 = t25*t79
    t102 = t26*t79
    t103 = t37*t79
    t104 = t38*t79
    t105 = t39*t79
    t106 = t41*t79
    t107 = t42*t79
    t108 = t44*t79
    t109 = t30*t79
    t110 = t32*t79
    t111 = t34*t79
    t112 = t36*t79
    t115 = t55*t79
    t116 = t56*t79
    t117 = t57*t79
    t118 = t59*t79
    t120 = t3*t33*t79
    t121 = t5*t31*t79
    t122 = t51*t79
    t123 = t52*t79
    t124 = t53*t79
    t125 = t54*t79
    t128 = t71*t79
    t129 = r*t7*t67*t80
    t78 = t73*2.0
    t97 = np.sqrt(t81)
    t98 = -t87
    t113 = t9+t90
    t114 = t8+t91
    t119 = t92-1.0/2.0
    t126 = -t116
    t127 = -t120
    t130 = -t129
    t131 = t77*t88
    t145 = r_head*t14*t84*t85*2.0
    t146 = r_head*t15*t84*t85*2.0
    t149 = r_head*t22*t84*t85*2.0
    t150 = r_head*t23*t84*t85*2.0
    t156 = t81+t128
    t163 = r*t43*t67*t84*t85
    t166 = r*t7*t67*t84*t85*-2.0
    t167 = r_head*t15*t67*t84*t85*-2.0
    t170 = r_head*t23*t67*t84*t85*-2.0
    t178 = t12+t13+t29+t31+t39+t52+t99+t101+t118+t122
    t179 = t25+t27+t29+t31+t39+t52+t89+t93+t118+t122
    t180 = t25+t27+t29+t31+t52+t59+t89+t93+t105+t122
    t181 = t12+t13+t33+t35+t54+t57+t99+t101+t103+t124
    t183 = (t12+t13+t32+t59+t99+t101+t105+t109+t8*t28+t9*t28)**2
    t184 = (t12+t13+t32+t39+t99+t101+t109+t118+t8*t28+t9*t28)**2
    t186 = t29+t31+t42+t44+t51+t59+t94+t95+t105+t123
    t187 = t20+t21+t33+t35+t37+t53+t107+t108+t117+t125
    t188 = t20+t21+t33+t35+t53+t57+t103+t107+t108+t125
    t189 = t33+t35+t37+t42+t44+t53+t94+t95+t117+t125
    t192 = (t20+t21-t33+t34-t35+t57+t103+t107+t108+t112)**2
    t82 = np.sin(t78)
    t132 = r_head*t3*t61*t113
    t133 = r_head*t3*t61*t114
    t134 = r_head*t5*t61*t113
    t135 = r_head*t5*t61*t114
    t136 = t131-1.0
    t139 = t39*t61*t119*2.0
    t140 = t37*t61*t119*2.0
    t147 = Dx*r*t43*t61*t119
    t148 = -t145
    t151 = r*t2*t43*t61*t119
    t152 = Dx*r*t7*t61*t119*-2.0
    t153 = -t149
    t159 = r_head*t3*t156
    t160 = r_head*t5*t156
    t164 = t67*t145
    t165 = t67*t146
    t168 = t67*t149
    t169 = t67*t150
    t171 = r_head*t3*t84*t85*t97*2.0
    t172 = r_head*t5*t84*t85*t97*2.0
    t174 = r*t43*t67*t97*t119
    t175 = r*t7*t67*t97*t119*-2.0
    t182 = t178**2
    t185 = t181**2
    t190 = t187**2
    t191 = t188**2
    t193 = t186**2
    t194 = t74+t86+t87+t145+t150
    t198 = t62*t183
    t199 = t62*t184
    t201 = t70+t96+t98+t145+t150
    t209 = t62*t192
    t137 = -t132
    t138 = -t135
    t141 = -t139
    t142 = r_head*t3*t136
    t143 = r_head*t5*t136
    t144 = -t140
    t154 = r_head*t3*t82*t119
    t155 = r_head*t5*t82*t119
    t161 = -t159
    t162 = -t160
    t173 = -t172
    t176 = t132+t139
    t177 = t135+t140
    t195 = t74+t86+t87+t148+t150
    t196 = t74+t86+t87+t146+t153
    t197 = t62*t182
    t200 = t62*t185
    t202 = t194**2
    t205 = t201**2
    t206 = t62*t193
    t207 = t62*t190
    t208 = t62*t191
    t223 = L+t130+t164+t172
    t226 = L+t130+t167+t171
    t229 = t17+t129+t164+t172
    t250 = L+t164+t166+t169
    t251 = L+t164+t166+t170
    t252 = L+t166+t167+t168
    t253 = t17+t163+t164+t169
    t157 = -t154
    t158 = -t155
    t203 = t195**2
    t204 = t196**2
    t210 = t16+t147+t155+t161
    t211 = t28+t147+t155+t159
    t214 = t18+t152+t154+t162
    t222 = t61*t202
    t227 = L+t130+t164+t173
    t228 = t61*t205
    t230 = np.abs(t223)
    t231 = np.abs(t229)
    t232 = np.abs(t226)
    t238 = t16+t142+t155+t174
    t240 = t18+t143+t154+t175
    t212 = np.abs(t210)
    t213 = np.abs(t211)
    t215 = t16+t147+t158+t161
    t217 = np.abs(t214)
    t224 = t61*t203
    t225 = t61*t204
    t233 = np.abs(t227)
    t234 = t230**2
    t235 = t232**2
    t237 = t231**2
    t239 = np.abs(t238)
    t241 = t18+t143+t157+t174
    t242 = np.abs(t240)
    t245 = t18+t143+t157+t175
    t254 = t197+t207+t222
    t257 = t198+t209+t228
    t216 = np.abs(t215)
    t218 = t212**2
    t219 = t213**2
    t221 = t217**2
    t236 = t233**2
    t243 = np.abs(t241)
    t244 = t239**2
    t246 = np.abs(t245)
    t247 = t242**2
    t255 = t200+t206+t225
    t256 = t199+t208+t224
    t258 = 1.0/np.sqrt(t254)
    t261 = 1.0/np.sqrt(t257)
    t220 = t216**2
    t248 = t243**2
    t249 = t246**2
    t259 = 1.0/np.sqrt(t255)
    t260 = 1.0/np.sqrt(t256)
    t262 = t221+t235+t244
    t263 = t218+t236+t247
    t264 = t220+t234+t248
    t265 = t219+t237+t249
    t266 = 1.0/np.sqrt(t262)
    t267 = 1.0/np.sqrt(t263)
    t268 = 1.0/np.sqrt(t264)
    t269 = 1.0/np.sqrt(t265)
    return np.array([t215*t268,t241*t268,t223*t268,t177*t250*t261+r_head*t63*t67*t80*t261*(t18+t138+t144+t151),-t176*t250*t261-r_head*t63*t67*t80*t261*(t16+t137+t141+t147),r_head*t61*t261*(t24+t26+t50+t55-t56+t58+t60+t104+t106+t116+t120+t47*t79+t48*t79+t66*t79+t5*t8*t28+t5*t28*t91),-t214*t266,t238*t266,t226*t266,t252*t259*(t133+t141)-r_head*t67*t69*t80*t259*(t16-t133+t139+t151),t252*t259*(t134+t144)-r_head*t67*t69*t80*t259*(t18-t134+t140+t152),r_head*t61*t259*(t56+t66+t115+t126+t3*t12+t3*t13+t5*t20+t5*t21+t3*t33+t3*t99+t3*t101+t5*t108+t50*t79+t5*t9*t28+t5*t28*t90+r*t17*t23*t79),t211*t269,-t245*t269,-t229*t269,
        t177*t253*t258-r_head*t63*t67*t80*t258*(-t18+t151+t177),-t176*t253*t258+r_head*t63*t67*t80*t258*(t28+t147+t176),-r_head*t61*t258*(t24+t26+t49+t56+t58+t60+t65+t66+t104+t106+t115+t121+t126+t127+t47*t79+t48*t79),t210*t267,-t240*t267,t227*t267,-t251*t260*(t135+t144)-r_head*t67*t68*t80*t260*(t18+t138+t140-r*t2*t7*t61*t119*2.0),-t251*t260*(t132+t141)-r_head*t67*t68*t80*t260*(t16+t137+t139+t147),-r_head*t61*t260*(t24+t26+t38+t41+t50+t56+t66+t115+t120+t126+t47*t79+t48*t79+t58*t79+t60*t79+t5*t8*t28+t5*t28*t91)]).reshape((6,4))


# @jit(nopython=True)
def directKine(in1,L,r):
    Dx = in1[0]
    Dy = in1[1]
    dL = in1[2]

    t2 = Dx**2
    t3 = Dy**2
    t4 = L*2.0
    t5 = dL*2.0
    t6 = 1.0/r
    t7 = t2+t3
    t8 = t4+t5
    t9 = 1.0/t7
    t10 = np.sqrt(t7)
    t11 = 1.0/t10
    t12 = t2*t9
    t16 = t6*t10
    t13 = Dx*t11
    t15 = -t12
    t18 = np.cos(t16)
    t21 = t16/2.0
    t14 = np.cos(t13)
    t19 = t15+1.0
    t22 = np.cos(t21)
    t23 = np.sin(t21)
    t24 = t18/2.0
    t17 = t14*2.0
    t25 = np.sqrt(t19)
    t26 = t24-1.0/2.0
    t28 = t13*t22*t23*2.0
    t20 = np.sin(t17)
    t29 = t22*t23*t25*2.0
    t27 = t20*t26
    return np.array([t19+t12*t18,
        t27,
        -t28,
        0.0,
        t27,
        -(t12-1.0)*(t18-1.0)+1.0,
        -t29,
        0.0,
        t28,
        t29,
        t18,
        0.0,
        -Dx*r*t8*t9*t26,
        -r*t8*t11*t25*t26,r*t11*np.sin(t16)*(L+dL),1.0]).reshape((4,4))

   
# @jit(nopython=True)
def forwardKine(in1,L,r):
    Dx = in1[0]
    Dy = in1[1]
    dL = in1[2]
    t2 = Dx**2
    t3 = Dy**2
    t4 = L*2.0
    t5 = dL*2.0
    t6 = 1.0/r
    t7 = t2+t3
    t8 = t4+t5
    t9 = 1.0/t7
    t10 = np.sqrt(t7)
    t11 = 1.0/t10
    t12 = t6*t10
    t13 = np.cos(t12)
    t14 = t13/2.0
    t15 = t14-1.0/2.0
    return np.array([-Dx*r*t8*t9*t15,
        -r*t8*t11*t15*np.sqrt(-t2*t9+1.0),
        r*t11*np.sin(t12)*(L+dL)]).reshape((-1,))

# @jit(nopython=True)
def stiffnessTorque(in1,kappa_theta,kappa_phi,kappa_dL):
    dL = in1[2]
    phi = in1[0]
    theta = in1[1]
    return np.array([kappa_phi*phi, kappa_theta*theta, dL*kappa_dL])
 
# @jit(nopython=True)
def gravityVector(q,L,r,g,m_0):
    dL = q[2]
    Dx = q[0]
    Dy = q[1]

    t2 = L+dL
    t3 = Dx**2
    t4 = Dy**2
    t5 = 1.0/r
    t6 = t3+t4
    t7 = 1.0/t6
    t8 = np.sqrt(t6)
    t9 = 1.0/t8**3
    t10 = t5*t8
    t11 = np.cos(t10)
    t12 = np.sin(t10)
    return np.array([Dx*g*m_0*t2*t7*t11-Dx*g*m_0*r*t2*t9*t12,
            Dy*g*m_0*t2*t7*t11-Dy*g*m_0*r*t2*t9*t12,
            (g*m_0*r*t12)/t8]).reshape((-1,))

# @jit(nopython=True)
def dampingTorque(in1,d_ii):
    dL_dot = in1[2]
    phi_dot = in1[0]
    theta_dot = in1[1]
    return np.array([d_ii*phi_dot, d_ii*theta_dot, dL_dot*d_ii])

# if __name__ == '__main__':
#     q = np.array([0.022, 0.085, 0]) # this should be the inital position from my test
#     # q = np.array([1.5, 0.4, -0.035]) # this should be the final position from my test
#     q_dot = np.zeros(3)
#     d_ii = 0.1
#     kappa_theta = 1
#     kappa_phi = 1
#     kappa_dL = 1
#     r = 1
#     g = 0 # 9.81
#     L = 0.3 # 1.0
#     m_0 = 1.0
#     I_xx = 1/12 * m_0 * L**2
#     I_yy = 1/12 * m_0 * L**2
#     I_zz = 1/50 * m_0 * L**2
#     r_head_1 = 0.05
#     theta_head_1 = np.pi/4
    
#     G = gravityVector(q, L, r, m_0, g)
#     K = stiffnessTorque(q,kappa_theta,kappa_phi,kappa_dL)
#     D = dampingTorque(q_dot, d_ii)
#     M = inertiaMatrix(q, L, r, I_xx, I_yy, I_zz, m_0)
#     C = coriolisMatrix(q, q_dot, r, L, I_xx, I_yy, I_zz, m_0)
#     # Alin = tendonMatrixLinear(q,L,r_head_1,theta_head_1)
#     A = tendonMatrix(q, r, L, theta_head_1, theta_head_1, r_head_1, r_head_1)
#     pos = forwardKine(q, L, r)
#     J = jacobianDiffKine(q,L,r)
#     print('=======================================================================')
#     # print('Tendon Matrix Lin:\n',Alin, np.shape(Alin))
#     print('Tendon Matrix:\n', A, np.shape(A))
#     print('Inertial:\n', M, np.shape(M))
#     print('Coriolis:\n', C, np.shape(C))
#     print('Damping:\n', D, np.shape(D))
#     print('Stifnness:\n', K, np.shape(K))
#     print('Gravity:\n', G, np.shape(G))
#     print('Jacobian Pos:\n', J, np.shape(J))
#     print('=======================================================================')
#     print('=======================================================================')
#     print('Forward Kine:\n', pos, np.shape(pos))


