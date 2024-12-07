% initial code
L2 = 186;
L3 = 2.9 * L2;
O4B = 1.2 * L2;
O4C = 3.5 * L2;
d = 3 * L2;
Omega2 = 6;
dt = 0.0001;
t = dt;
Theta2 = (pi/2) + Omega2 * t;

syms Theta4 Theta3;
PersTheta3 = L3 * cos(Theta3) == d + O4B * cos (asin((L2*sin(Theta2) + L3*sin(Theta3))/O4B)) - L2*cos(Theta2);
persTheta4 = O4B*sin(Theta4) == L2 * sin(Theta2) + L3*sin(acos((d + O4B*cos(Theta4) - L2*cos(Theta2))/L3));
Initial_Coordinate_L2 = [0; 0];

R1 = Initial_Coordinate_L2 + [L2 * cos(Theta2);
                              L2 * sin(Theta2)];

R3 = R1 + [L3 * cos(solve(PersTheta3, Theta3));
           L3 * sin(solve(PersTheta3, Theta3))];

R4 = [d; 0] + [O4B*cos(solve(persTheta4, Theta4));
               O4B*sin(solve(persTheta4, Theta4))];



link O4C.
