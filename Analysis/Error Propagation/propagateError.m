function [ outputError ] = propagateError( Type, firstComponent, secondComponent, firstComponentErrors, secondComponentErrors)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

if strcmp(Type,'CrossProduct') == 1
    if numel(firstComponent) + numel(secondComponent) ~= 6
        error('Check component vector sizes');
    end
    if numel(firstComponent) + numel(secondComponent) ~= numel(firstComponentErrors) + numel(secondComponentErrors)
        error('Check component error sizes');
    end
    
    [a1,a2,a3] = vector2components(firstComponent);
    [b1,b2,b3] = vector2components(secondComponent);
    [e_a1,e_a2,e_a3] = vector2components(firstComponentErrors);
    [e_b1,e_b2,e_b3] = vector2components(secondComponentErrors);
    
    outputError = [sqrt((b3*e_a2)^2+(b2*e_a3)^2+(a3*e_b2)^2+(a2*e_b3)^2);...
                   sqrt((b3*e_a1)^2+(b1*e_a3)^2+(a3*e_b1)^2+(a1*e_b3)^2);...
                   sqrt((b2*e_a1)^2+(b1*e_a2)^2+(a2*e_b1)^2+(a1*e_b2)^2)]; 
               
elseif strcmp(Type,'VectorAddition') == 1
    if numel(firstComponent) + numel(secondComponent) ~= 6
        error('Check component vector sizes');
    end
    if numel(firstComponent) + numel(secondComponent) ~= numel(firstComponentErrors) + numel(secondComponentErrors)
        error('Check component error sizes');
    end
    
    firstComponentErrors = reshape(firstComponentErrors,3,1);
    secondComponentErrors = reshape(secondComponentErrors,3,1);    
    
    outputError = sqrt(firstComponentErrors.^2 + secondComponentErrors.^2);
    
elseif strcmp(Type, 'RotateVectorBody2World') == 1 %input1: q, input2: vector
    if numel(firstComponent) ~= 4
        error('Check quaternion vector size (first component)');
    end
    
    if numel(firstComponentErrors) ~= 1
        error('Check quaternion error size (1 number only)');
    end    
    
    if numel(secondComponent) ~= 3
        error('Check vector size (second component)');
    end
    
    if numel(secondComponentErrors) ~= 3
        error('Check vector error size (second component)');
    end
    
    %find Rotation matrix
    rotMat = quat2rotmat(reshape(firstComponent,4,1))'; %transpose for body2world
    rotMatError = propagateError('Quat2RotMat',firstComponent,[],firstComponentErrors,[])';%transpose for body2world
    v = reshape(secondComponent,3,1);
    e_v = reshape(secondComponentErrors,3,1);
%     [v1,v2,v3] = vector2components(secondComponent);
%     [e_v1,e_v2,e_v3] = vector2components(secondComponentErrors);
        
    outputError = [sqrt(sum((v.*rotMatError(1,:)').^2)+sum((e_v.*rotMat(1,:)').^2));...
                   sqrt(sum((v.*rotMatError(2,:)').^2)+sum((e_v.*rotMat(2,:)').^2));...
                   sqrt(sum((v.*rotMatError(3,:)').^2)+sum((e_v.*rotMat(3,:)').^2))];

elseif strcmp(Type, 'Quat2RotMat') == 1;
    if numel(firstComponent) ~= 4
        error('Check quaternion vector size (first component)');
    end
    
    if numel(firstComponentErrors) ~= 1
        error('Check quaternion error size (1 number only)');
    end
    
    outputError = zeros(3,3);
    
    for iRow = 1:3
        for iColumn = 1:3
            if iRow == iColumn
                outputError(iRow,iColumn) = 2*norm(firstComponent)^2*firstComponentErrors;
            else
                outputError(iRow,iColumn) = norm(firstComponent)*firstComponentErrors;
            end
        end
    end  

elseif strcmp(Type, 'VectorMagnitude') == 1
    if numel(firstComponentErrors) ~= 3
        error('Check vector error size (first component)');
    end
    outputError = norm(firstComponentErrors);
else
    error('Invalid Type');    
end

end

function [a1,a2,a3] = vector2components(a)
    a1 = a(1);
    a2 = a(2);
    a3 = a(3);
end

