s = tf('s');
K = 0.05529;
sigma = 14.224;
load ("testedXVals.mat")
load ("testedYVals.mat")
transFunc = K*(sigma/(sigma + s)); 
step(transFunc)
hold on
plot(testedXVals,testedYVals)



