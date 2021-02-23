function [valInRange] = computeValInRange(LB,UB, seed)
%computeValInRange: This function computes a value in the range listed by
%lower bound and upper bound:
%INPUTS: 
%    LB: This is the lower bound that you want
%    UB: This is the upper bound that you want on the value
%    seed: this is the seed in the random number generator
seed;
valInRange = (UB-LB).*rand(1,1) + LB;
end

