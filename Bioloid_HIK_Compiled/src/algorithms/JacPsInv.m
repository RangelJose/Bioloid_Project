function [Jtd,Jt] =JacPsInv(J)

[m,n] = size(J);
if m<n
    A = J*J.';
    [n,~]=size(A);
    A = A+(eye(n)*0.0001);
    A = A\eye(n);
    Jtd = (J.')*A;
elseif n<m
    A = J.'*J;
    [n,~]=size(A);
    A = A+(eye(n)*0.1);
    A = A\eye(n);
    Jtd = A*(J.');
end

Jt = pinv(J);

end
