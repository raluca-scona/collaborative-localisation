import matplotlib.pyplot as plt
import math;

pfdata = open('pf.txt', 'r');
usbldata = open('usbl.txt', 'r');
odometrydata = open('odometry.txt', 'r');
truthdata = open('truth.txt', 'r');
pfx = [];
pfy = [];

usblx = [];
usbly = [];

odomx = [];
odomy = [];

truthx = [];
truthy = [];

pf = pfdata.readlines();
usbl = usbldata.readlines();
odometry = odometrydata.readlines();
truth = truthdata.readlines();


pfdata.close();
usbldata.close();
odometrydata.close();
truthdata.close();

for i in range(len(pf)):
    pfnos = pf[i].rsplit();
    pfx.append(float(pfnos[0]))
    pfy.append(float(pfnos[1]))
    
    usblnos = usbl[i].rsplit();
    usblx.append(float(usblnos[0]))
    usbly.append(float(usblnos[1]))
    
    truthnos = truth[i].rsplit();
    truthx.append(float(truthnos[0]));
    truthy.append(float(truthnos[1]));
    
    odometrynos = odometry[i].rsplit();
    odomx.append(float(odometrynos[0]));
    odomy.append(float(odometrynos[1]));

error = 0;
for i in range(len(pf)):
    error = error + (pfx[i] - truthx[i])**2 + (pfy[i] - truthy[i])**2;

error = math.sqrt(error);
print "error ", error    

    
tpl, = plt.plot(truthx, truthy, 'b')
usblpl, = plt.plot(usblx, usbly, 'r')
pfpl, = plt.plot(pfx, pfy, 'g')
odompl, = plt.plot(odomx, odomy, 'y')


plt.legend([tpl, usblpl, pfpl, odompl], ['truth', 'usbl', 'pf', 'dr'])

def fun(p):
    return 0.7 * p / (0.4*p + 0.3)



