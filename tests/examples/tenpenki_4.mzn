% RUNS ON mzn20_fd
% RUNS ON mzn-fzn_fd
% RUNS ON mzn20_fd_linear
% RUNS ON mzn20_mip
include "tenpenki.mzn.model";

nrows = 3;
ncols = 3;

constraint row_constraint(1, [3]);
constraint row_constraint(2, [2]);
constraint row_constraint(3, [1]);

constraint col_constraint(1, [3]);
constraint col_constraint(2, [2]);
constraint col_constraint(3, [1]);

% Solution:
% 
%   # # #
%   # # .
%   # . .

