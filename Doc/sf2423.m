R=5.3 %datasheet
L=12.5*10^-3; %datasheet
Iph = 1; %datasheet
Vbus = 22; %I choose that so that Vbatt always > Vbus
ttick = 250*10^-9; %definition on AN4144

%Finding Ke

N=3; %number of period
Ttot = 59.615 * 10^-3;
T=Ttot/N;
f=1/T
Vpeak = 3.2083
Ke = Vpeak/f

%Calculating Shield Parameters

Kval = (R*Iph) % (/vbus)*2^8 taken into account in the lib
intersectSpeed = ((4*R)/(2*pi*L))
startingSlope = ((Ke/4)/Vbus)
finalSlope = (((2*pi*L*Iph+Ke)/4)/Vbus)

