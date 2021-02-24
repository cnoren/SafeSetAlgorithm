function [valInRange] = computeValInRange(LB,UB, seed, half)
%computeValInRange: This function computes a value in the range listed by
%lower bound and upper bound:
%INPUTS: 
%    LB: This is the lower bound that you want
%    UB: This is the upper bound that you want on the value
%    seed: this is the seed in the random number generator
seed;
if half
    valInRange = (LB + UB)/2;
else
    valInRange = (UB-LB).*rand(1,1) + LB;
end % This just checks to see if we want the middle of the interval
end

