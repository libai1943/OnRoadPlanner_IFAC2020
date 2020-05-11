function [lb, rb] = ProvideRoadBound(s)
lb = -2 - cos(0.2 * s + 2.8166138) * 0.2;
rb = 5 + cos(0.78 * s - 0.8166138) * 0.15;
end