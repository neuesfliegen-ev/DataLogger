function [b, C] = A01_ellipsoidFit(X, Y, Z)
% ellipsoidFit  Fit an ellipsoid to 3D points and return
%   b = 3×1 hard-iron offset
%   C = 3×3 soft-iron correction matrix
%
% Model: (m - b)' * inv(C) * (m - b) = 1

  % build design matrix for quadratic form:
  D = [ X.^2   , Y.^2   , Z.^2   , ...
        2*X.*Y , 2*X.*Z , 2*Y.*Z , ...
        2*X    , 2*Y    , 2*Z    ];

  % right–hand side = ones
  rhs = ones(size(X));

  % solve D * v = rhs in least squares sense
  v = D \ rhs;

  % unpack parameters
  A = v(1); B = v(2); Cc = v(3);
  Dxy = v(4); Exz = v(5); Fyz = v(6);
  G = v(7); H = v(8); I = v(9);

  % assemble the 4×4 algebraic form
  M = [ A,    Dxy , Exz , G ;
        Dxy,  B   , Fyz , H ;
        Exz,  Fyz , Cc  , I ;
        G,    H   , I   ,-1 ];

  % find the center b by solving grad = 0:
  center = -M(1:3,1:3) \ [G; H; I];

  % translate to center the ellipsoid
  T = eye(4);
  T(4,1:3) = center';
  R = T * M * T';

  % extract the 3×3 shape matrix and normalize so the bottom-right = –1
  R3 = R(1:3,1:3) / (-R(4,4));

  % the correction matrix is the inverse square‐root of R3
  [U,S] = svd(R3);
  C = U * sqrt(S) * U';  

  b = center;
end
