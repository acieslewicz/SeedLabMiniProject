
%%The purpose of this code is to recreate a transfer function that
%represents the step response of our motor. K and Sigma values are
%calculated from Serial output displaynih radians and elapsed time. The
%actual output values from the Serial output for time and radians/sec are
%contained within testedXVals and testedYVals respectively.
s = tf('s');
K = 0.05529;
sigma = 14.224;
load ("testedXVals.mat")
load ("testedYVals.mat")
transFunc = K*(sigma/(sigma + s)); 
step(transFunc)
hold on
plot(testedXVals,testedYVals)



