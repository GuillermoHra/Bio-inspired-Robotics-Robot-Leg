function [maxj] =get_max_jerk_rmhb(sol,p)

maxj=max(abs(diff(diff(diff(sol.y(1,:))))));
end