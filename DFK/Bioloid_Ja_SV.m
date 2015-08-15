function  out = Bioloid_Ja_SV (J_g, q)
%% Analytical Jacobian matrix
% Inputs:
%    Jg_RH = Geometric Jacobian matrix, where
%            Jg = [J_g(1),  J_g(7),  J_g(13), J_g(19), J_g(25), J_g(31), J_g(37), J_g(43), J_g(49), J_g(55), J_g(61), J_g(67), J_g(73),  J_g(79), J_g(85), J_g(91),  J_g(97), J_g(103), J_g(109),  J_g(115), J_g(121), J_g(127), J_g(133), J_g(139); 
%                  J_g(2),  J_g(8),  J_g(14), J_g(20), J_g(26), J_g(32), J_g(38), J_g(44), J_g(50), J_g(56), J_g(62), J_g(68), J_g(74),  J_g(80), J_g(86), J_g(92),  J_g(98), J_g(104), J_g(110),  J_g(116), J_g(122), J_g(128), J_g(134), J_g(140);
%                  J_g(3),  J_g(9),  J_g(15), J_g(21), J_g(27), J_g(33), J_g(39), J_g(45), J_g(51), J_g(57), J_g(63), J_g(69), J_g(75),  J_g(81), J_g(87), J_g(93),  J_g(99), J_g(105), J_g(111),  J_g(117), J_g(123), J_g(129), J_g(135), J_g(141);
%                  J_g(4),  J_g(10), J_g(16), J_g(22), J_g(28), J_g(34), J_g(40), J_g(46), J_g(52), J_g(58), J_g(64), J_g(70), J_g(76),  J_g(82), J_g(88), J_g(94), J_g(100), J_g(106), J_g(112),  J_g(118), J_g(124), J_g(130), J_g(136), J_g(142);
%                  J_g(5),  J_g(11), J_g(17), J_g(23), J_g(29), J_g(35), J_g(41), J_g(47), J_g(53), J_g(59), J_g(65), J_g(71), J_g(77),  J_g(83), J_g(89), J_g(95), J_g(101), J_g(107), J_g(113),  J_g(119), J_g(125), J_g(131), J_g(137), J_g(143);
%                  J_g(6),  J_g(12), J_g(18), J_g(24), J_g(30), J_g(36), J_g(42), J_g(48), J_g(54), J_g(60), J_g(66), J_g(72), J_g(78),  J_g(84), J_g(90), J_g(96), J_g(102), J_g(108), J_g(114),  J_g(120), J_g(126), J_g(132), J_g(138), J_g(144)];
%       EE_Pose = [J_g(145)
%                  J_g(146)
%                  J_g(147)
%                  J_g(148)
%                  J_g(149)
%                  J_g(150)]
%
%    q = Robot configuration, i.e. each element of q is a joint position
% Outputs:
%    Ja = Analaytical Jacobian matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
REuler1=RotX(J_g(148));
REuler2=RotY(J_g(149));
REuler3=RotZ(J_g(150));
REuler1=REuler1(1:3,1:3);
REuler2=REuler2(1:3,1:3);
REuler3=REuler3(1:3,1:3);
Eu1=[1;0;0];
Eu2=[0;1;0];
Eu3=[0;0;1];

B=[REuler1*Eu1 REuler1*REuler2*Eu2 REuler1*REuler2*REuler3*Eu3];

Jv=[J_g(1),  J_g(7),  J_g(13), J_g(19), J_g(25), J_g(31), J_g(37), J_g(43), J_g(49), J_g(55), J_g(61), J_g(67), J_g(73),  J_g(79), J_g(85), J_g(91),  J_g(97), J_g(103), J_g(109),  J_g(115), J_g(121), J_g(127), J_g(133), J_g(139); 
    J_g(2),  J_g(8),  J_g(14), J_g(20), J_g(26), J_g(32), J_g(38), J_g(44), J_g(50), J_g(56), J_g(62), J_g(68), J_g(74),  J_g(80), J_g(86), J_g(92),  J_g(98), J_g(104), J_g(110),  J_g(116), J_g(122), J_g(128), J_g(134), J_g(140);
    J_g(3),  J_g(9),  J_g(15), J_g(21), J_g(27), J_g(33), J_g(39), J_g(45), J_g(51), J_g(57), J_g(63), J_g(69), J_g(75),  J_g(81), J_g(87), J_g(93),  J_g(99), J_g(105), J_g(111),  J_g(117), J_g(123), J_g(129), J_g(135), J_g(141)];
Jw=[J_g(4),  J_g(10), J_g(16), J_g(22), J_g(28), J_g(34), J_g(40), J_g(46), J_g(52), J_g(58), J_g(64), J_g(70), J_g(76),  J_g(82), J_g(88), J_g(94), J_g(100), J_g(106), J_g(112),  J_g(118), J_g(124), J_g(130), J_g(136), J_g(142);
    J_g(5),  J_g(11), J_g(17), J_g(23), J_g(29), J_g(35), J_g(41), J_g(47), J_g(53), J_g(59), J_g(65), J_g(71), J_g(77),  J_g(83), J_g(89), J_g(95), J_g(101), J_g(107), J_g(113),  J_g(119), J_g(125), J_g(131), J_g(137), J_g(143);
    J_g(6),  J_g(12), J_g(18), J_g(24), J_g(30), J_g(36), J_g(42), J_g(48), J_g(54), J_g(60), J_g(66), J_g(72), J_g(78),  J_g(84), J_g(90), J_g(96), J_g(102), J_g(108), J_g(114),  J_g(120), J_g(126), J_g(132), J_g(138), J_g(144)];

damping=eye(3)*0.00001;
B=B+damping;
Jwa=B\Jw;
Ja=[Jv;Jwa];
%Ja=zeros(6,24);
out = [Ja(:,1); Ja(:,2); Ja(:,3); Ja(:,4); Ja(:,5); Ja(:,6);Ja(:,7); Ja(:,8); Ja(:,9); Ja(:,10); Ja(:,11); Ja(:,12);Ja(:,13); Ja(:,14); Ja(:,15); Ja(:,16); Ja(:,17); Ja(:,18);Ja(:,19); Ja(:,20); Ja(:,21); Ja(:,22); Ja(:,23); Ja(:,24)];