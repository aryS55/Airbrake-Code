function [dragForce] = DragFunction(coefficentOfDrag,velocity,area)
%calculate drag force using coefficient of drag and velocity
arguments (Input)
    coefficentOfDrag
    velocity
    area
end

arguments (Output)
    dragForce
    
end

dragForce = .5 * 1.225 * coefficentOfDrag * area * velocity^2;

end