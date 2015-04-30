%%%% Xsf = position of the stance foot. 
%%%% Xnsf = position of the non-stance foot (what will be used for
%%%% stepping. 
%%%% Xf = final position of the foot  (viewed as a variable)
%%%% Vref = reference final velocity
syms PcmpRef Pcmpx Pcmpu real
syms Pzu Pzx Ppx Ppu Pvx Pvu u x0 real
syms LdotX LdotU real
syms Xsf Xf XfRef Pref P real
%%%% Reference velocity
syms a b c d e real;
%%% Dynamics of the system z = zmp dynamics, P = center of mass dynamics
P = Ppx*x0 + Ppu*u;
V = Pvx*x0 + Pvu*u;
Ldot = LdotX*x0 + LdotU*u;
CMP = Pcmpx*x0 + Pcmpu*u;

J1 = 0.5*a*transpose(u)*u;
J2 = 0.5*b*transpose(CMP-PcmpRef)*(CMP-PcmpRef);
J3 = 0.5*c*transpose(Ldot)*Ldot;
J = J1+J2+J3;

z = u;

dJ1 = jacobian(J1,z);
dJ2 = jacobian(J2,z);
dJ3 = jacobian(J3,z);

H1 = jacobian(dJ1,z);
H2 = jacobian(dJ2,z);
H3 = jacobian(dJ3,z);

H = H1 + H2 + H3

jacobian(J,z);

f = subs(jacobian(J,z),z,0*z)
