function [ q_inv ] = quatinv( q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
q_inv = quatconj(q)/norm(q)^2;

end

